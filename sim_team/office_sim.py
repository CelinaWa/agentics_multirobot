"""The two-robot office world: one MuJoCo simulation with the G1 and the R1 (sim_team/build_office_scene.py).

This process owns the physics. The G1 is driven the usual way: the SONIC deploy stack over DDS, exactly as in the
single-robot sim2sim loop this file is adapted from (r1/tools/scratch/g1cmp/sim_headless_dom.py: keyboard strings on a
ZMQ SUB socket, --spawn, --lockstep, the qpos stream). The R1's policy runs in another process (sim_team/r1_brain.py,
another Python): every control period (4 physics steps) this process sends it the whole state and gets back the R1's
joint targets and gripper commands, and applies them with the R1's own PD gains at every physics step.

  /opt/sonic/.venv_sim/bin/python office_sim.py --scene <abs office_g1_r1.xml> --spawn x,y,yaw --r1-tick-port 5695 [--lockstep]

Start r1_brain.py first: this process waits for it.
"""

import argparse
import sys
import threading
import time

import numpy as np
import os
import zmq

from gear_sonic.data.robot_model.instantiation.g1 import instantiate_g1_robot_model
from gear_sonic.utils.mujoco_sim.configs import SimLoopConfig
from gear_sonic.utils.mujoco_sim.simulator_factory import SimulatorFactory, init_channel


def main():
    ap = argparse.ArgumentParser(description=__doc__.split("\n\n")[0])
    ap.add_argument("--env-name", default="default")
    ap.add_argument("--key-port", type=int, default=5590,
                    help="ZMQ SUB port for keyboard strings ('9' = drop the robot)")
    ap.add_argument("--scene", default=None, help="absolute path of a scene xml replacing the env's ROBOT_SCENE")
    ap.add_argument("--sim-dt", type=float, default=None,
                    help="physics step (gear_sonic's default is 5 ms; 2 ms keeps finger contacts with small objects stable)")
    ap.add_argument("--lockstep", action="store_true",
                    help="advance one control period per deploy control tick (see the module docstring)")
    ap.add_argument("--state-port", type=int, default=5557, help="the deploy's state port (lockstep ticks)")
    ap.add_argument("--control-dt", type=float, default=0.02)
    ap.add_argument("--door-seal", type=float, default=0.0,
                    help="N at the fridge handle holding the door shut (a magnetic gasket): full at closed, fading to 0 "
                         "by 3 deg open (real fridges: ~20-40 N to break the seal)")
    ap.add_argument("--spawn", default=None,
                    help="x,y,yaw_deg: start the robot there (the gantry band's anchor moves with it), e.g. for task stages")
    ap.add_argument("--lockstep-idle-s", type=float, default=0.25)
    ap.add_argument("--r1-tick-port", type=int, default=5695, help="r1_brain.py's tick port (REQ/REP, one exchange per control period)")
    ap.add_argument("--qpos-port", type=int, default=5591,
                    help="ZMQ PUB port streaming the sim's qpos at 50 Hz (sim_camera_server.py mirrors it)")
    args = ap.parse_args()

    config = SimLoopConfig(env_name=args.env_name, enable_onscreen=False, enable_offscreen=False)
    wbc = config.load_wbc_yaml()
    wbc["DOMAIN_ID"] = int(os.environ.get("SIM_DDS_DOMAIN", wbc["DOMAIN_ID"]))   # g1cmp: own DDS domain
    print(f"[sim_headless] DDS domain {wbc['DOMAIN_ID']}", flush=True)
    wbc["ENV_NAME"] = args.env_name
    if args.scene:
        wbc["ROBOT_SCENE"] = args.scene
    if args.sim_dt:
        wbc["SIMULATE_DT"] = args.sim_dt
    init_channel(config=wbc)
    skip_r1_joints()
    sim = SimulatorFactory.create_simulator(config=wbc, env_name=args.env_name,
                                            onscreen=False, offscreen=False, enable_image_publish=False)

    ctx = zmq.Context.instance()
    sub = ctx.socket(zmq.SUB)
    sub.setsockopt_string(zmq.SUBSCRIBE, "")
    sub.connect(f"tcp://127.0.0.1:{args.key_port}")

    env = getattr(sim, "sim_env", sim)
    install_team_step(env, ctx, args)
    # g1cmp: "load=<kg>" hangs a box of that weight between the hands (half on each wrist_yaw_link, straight down)
    load = {"kg": 0.0}
    wrists = [env.mj_model.body(n).id for n in ("left_wrist_yaw_link", "right_wrist_yaw_link")]
    step_without_load = env.sim_step

    def step_with_load():
        for b in wrists:
            env.mj_data.xfrc_applied[b, :] = 0.0
            env.mj_data.xfrc_applied[b, 2] = -0.5 * load["kg"] * 9.81
        step_without_load()
    env.sim_step = step_with_load
    if args.door_seal > 0:
        hinge = env.mj_model.joint("fridge_hinge")
        qa, da = env.mj_model.jnt_qposadr[hinge.id], env.mj_model.jnt_dofadr[hinge.id]
        seal_tau, seal_open = args.door_seal * 0.47, np.radians(3.0)   # the handle is 0.47 m from the hinge axis
        plain_step = env.sim_step

        def step_with_seal():
            th = float(env.mj_data.qpos[qa])
            env.mj_data.qfrc_applied[da] = -seal_tau * max(0.0, 1.0 - th / seal_open) if th < seal_open else 0.0
            plain_step()
        env.sim_step = step_with_seal
        print(f"[sim_headless] fridge door seal: {args.door_seal:g} N at the handle, gone by 3 deg open", flush=True)
    if args.spawn:
        import mujoco
        x, y, yaw = (float(v) for v in args.spawn.split(","))
        a = np.radians(yaw)
        env.mj_data.qpos[0:2] = [x, y]
        env.mj_data.qpos[3:7] = [np.cos(a / 2), 0.0, 0.0, np.sin(a / 2)]
        env.mj_data.qvel[:6] = 0.0
        mujoco.mj_forward(env.mj_model, env.mj_data)
        band = getattr(env, "elastic_band", None)
        if band is not None:
            band.point = np.array([x, y, band.point[2]])
            # the band's spring also holds the orientation at identity (yaw included): hold the spawn heading
            import scipy.spatial.transform as sst
            target = sst.Rotation.from_euler("z", yaw, degrees=True)

            def advance(pose, band=band, target=target):
                pos, q, lin, ang = pose[0:3], pose[3:7], pose[7:10], pose[10:13]
                f = band.kp_pos * (band.point - pos + np.array([0, 0, band.length])) + band.kd_pos * (0 - lin)
                rot = sst.Rotation.from_quat([q[1], q[2], q[3], q[0]])
                torque = -band.kp_ang * (rot * target.inv()).as_rotvec() - band.kd_ang * ang
                return np.concatenate([f, torque])
            band.Advance = advance
        print(f"[sim_headless] robot spawned at {x}, {y}, yaw {yaw}", flush=True)
    band = getattr(env, "elastic_band", None)
    g1_home = {"qpos": env.mj_data.qpos[:G1_NQ].copy(), "band": None if band is None else band.length}

    def keys():
        while True:
            key = sub.recv_string()
            print(f"[sim_headless] key {key!r}", flush=True)
            band = getattr(env, "elastic_band", None)
            if key in ("7", "8") and band is not None:  # the viewer's glfw keys, absent headless
                band.length += 0.1 if key == "8" else -0.1
                print(f"[sim_headless] band length {band.length:+.1f} m", flush=True)
            elif key.startswith("band=") and band is not None:
                band.length = float(key[5:])
                print(f"[sim_headless] band length {band.length:+.2f} m", flush=True)
            elif key.startswith("fade=") and band is not None:  # fade=<s>: ramp the band's stiffness to 0, then release
                seconds = max(0.1, float(key[5:]))
                gains = (band.kp_pos, band.kd_pos, band.kp_ang, band.kd_ang)

                def fade():
                    steps = int(seconds / 0.05)
                    for i in range(steps, -1, -1):
                        f = i / steps
                        band.kp_pos, band.kd_pos, band.kp_ang, band.kd_ang = (g * f for g in gains)
                        time.sleep(0.05)
                    band.enable = False
                    band.kp_pos, band.kd_pos, band.kp_ang, band.kd_ang = gains
                    print(f"[sim_headless] band faded out over {seconds:g} s and released", flush=True)
                threading.Thread(target=fade, daemon=True).start()
            elif key.startswith("load="):
                load["kg"] = float(key[5:])
                print(f"[sim_headless] load {load['kg']:g} kg between the hands", flush=True)
            elif key in ("release", "attach") and band is not None:
                band.enable = key == "attach"
                print(f"[sim_headless] band {'attached' if band.enable else 'released'}", flush=True)
            elif key.startswith("joint="):  # joint=<scene joint>,value : the operator sets a door or drawer, at rest
                name, value = key[6:].split(",")
                joint = env.mj_model.joint(name)
                env.mj_data.qpos[env.mj_model.jnt_qposadr[joint.id]] = float(value)
                env.mj_data.qvel[env.mj_model.jnt_dofadr[joint.id]] = 0.0
                print(f"[sim_headless] set {name} to {value}", flush=True)
            elif key.startswith("g1_home=") and band is not None:
                # g1_home=x,y: after a fall the operator hangs the G1 back on its gantry at x,y in its spawn pose
                # (spawn heading, joints as spawned, band as at the start) and the physics runs again until the
                # restarted deploy stack's control ticks arrive
                x, y = (float(v) for v in key[8:].split(","))
                env.mj_data.qpos[:G1_NQ] = g1_home["qpos"]
                env.mj_data.qpos[0:2] = [x, y]
                env.mj_data.qvel[:G1_NQ - 1] = 0.0
                band.point = np.array([x, y, band.point[2]])
                band.length, band.enable = g1_home["band"], True
                env.lockstep_restart = True
                print(f"[sim_headless] G1 back on the gantry at {x}, {y}", flush=True)
            elif key.startswith("place="):  # place=<free body>,x,y,z[,qw,qx,qy,qz]: the operator puts an object back, at rest
                name, *values = key[6:].split(",")
                body = env.mj_model.body(name)
                joint = env.mj_model.jnt_qposadr[body.jntadr[0]]
                dof = env.mj_model.jnt_dofadr[body.jntadr[0]]
                env.mj_data.qpos[joint:joint + 3] = [float(v) for v in values[:3]]
                if len(values) == 7:  # the orientation too (an upright can), else the current one is kept
                    quat = np.array([float(v) for v in values[3:]])
                    env.mj_data.qpos[joint + 3:joint + 7] = quat / np.linalg.norm(quat)
                env.mj_data.qvel[dof:dof + 6] = 0.0
                print(f"[sim_headless] placed {name} at {values}", flush=True)
            else:
                sim.handle_keyboard_button(key)

    threading.Thread(target=keys, daemon=True).start()

    pub = ctx.socket(zmq.PUB)
    pub.bind(f"tcp://127.0.0.1:{args.qpos_port}")

    def qpos():
        while True:
            pub.send(np.asarray(env.mj_data.qpos, dtype=np.float64).tobytes())
            time.sleep(0.02)

    threading.Thread(target=qpos, daemon=True).start()

    # which hands touch the box, from the contacts the physics itself reports (for the operator's ground truth of "who
    # is holding it"): JSON on qpos_port + 1, about 20 times a second
    import json as _json
    touch_pub = ctx.socket(zmq.PUB)
    touch_pub.bind(f"tcp://127.0.0.1:{args.qpos_port + 1}")
    HANDS = {'g1': {'left': ('left_wrist_yaw_link', 'left_hand_'), 'right': ('right_wrist_yaw_link', 'right_hand_')},
             'r1': {'left': ('r1_left_gripper', 'r1_left_wrist_roll_link'), 'right': ('r1_right_gripper', 'r1_right_wrist_roll_link')}}

    def touches():
        model = env.mj_model
        try:
            box = model.body('box').id
        except Exception:
            return
        owner = {}
        for b in range(model.nbody):
            name = model.body(b).name
            for robot, sides in HANDS.items():
                for side, prefixes in sides.items():
                    if any(name.startswith(prefix) for prefix in prefixes):
                        owner[b] = (robot, side)
        while True:
            out = {robot: {'left': False, 'right': False} for robot in HANDS}
            try:
                data = env.mj_data
                for i in range(int(data.ncon)):
                    c = data.contact[i]
                    b1, b2 = int(model.geom_bodyid[c.geom1]), int(model.geom_bodyid[c.geom2])
                    other = b2 if b1 == box else b1 if b2 == box else None
                    if other in owner:
                        robot, side = owner[other]
                        out[robot][side] = True
                touch_pub.send_string(_json.dumps({'t': time.time(), 'hands_on_box': out}))
            except Exception:
                pass
            time.sleep(0.05)

    threading.Thread(target=touches, daemon=True).start()
    print(f"[sim_headless] READY (keys on tcp://127.0.0.1:{args.key_port}, qpos on {args.qpos_port}, "
          f"scene {wbc['ROBOT_SCENE']}{', lockstep' if args.lockstep else ''})", flush=True)
    sys.stdout.flush()
    if not args.lockstep:
        SimulatorFactory.start_simulator(sim, as_thread=False, enable_image_publish=False)
        return
    run_lockstep(sim, env, ctx, args)


G1_NQ = 50  # the G1's free joint and joints come first in qpos (the box's free joint follows at 50)


def skip_r1_joints():
    """The G1 simulator finds the G1's joints by words in their names (hip, knee, wrist, ...), which the R1's joints
    have too: make its scene setup pass over every joint behind the r1_ prefix."""
    import inspect, re, textwrap
    from gear_sonic.utils.mujoco_sim import base_sim
    source = textwrap.dedent(inspect.getsource(base_sim.DefaultEnv.init_scene))
    source, count = re.subn(r"^( +)name = self\.mj_model\.joint\(i\)\.name\n",
                            lambda k: k.group(0) + f"{k.group(1)}if name.startswith('r1_'):\n{k.group(1)}    continue\n",
                            source, flags=re.M)
    if count != 1:
        raise RuntimeError("the G1 simulator's init_scene changed: office_sim.py no longer knows how to patch it")
    namespace = dict(base_sim.__dict__)
    exec(source, namespace)
    base_sim.DefaultEnv.init_scene = namespace["init_scene"]


def install_team_step(env, ctx, args):
    """Make the G1 simulator's step leave the R1's actuators alone, and drive the R1 from r1_brain.py.

    The G1 simulator assigns the whole control vector from the G1's torques; here it writes only the G1's part, and
    the R1's part comes from PD on the targets the brain returns once per control period."""
    import inspect, json, re, textwrap, types
    import mujoco
    m, d = env.mj_model, env.mj_data
    source = textwrap.dedent(inspect.getsource(type(env).sim_step))
    source, count = re.subn(r"self\.mj_data\.ctrl = (.+)", r"_v = \1; self.mj_data.ctrl[:len(_v)] = _v", source)
    if count == 0 or "mujoco.mj_step(" not in source:
        raise RuntimeError("the G1 simulator's sim_step changed: office_sim.py no longer knows how to patch it")
    source = source.replace("mujoco.mj_step(", "_r1_step(self); mujoco.mj_step(", 1)
    req = ctx.socket(zmq.REQ)
    req.setsockopt(zmq.LINGER, 0)
    req.connect(f"tcp://127.0.0.1:{args.r1_tick_port}")
    print(f"[office_sim] waiting for r1_brain.py on port {args.r1_tick_port} ...", flush=True)
    req.send(b"hello")
    if not req.poll(120000):
        raise RuntimeError("r1_brain.py did not answer within 120 s: start it first")
    hello = json.loads(req.recv())
    qadr, vadr, act = (np.array(hello[k]) for k in ("qadr", "vadr", "act"))
    kp, kd, effort = (np.array(hello[k]) for k in ("kp", "kd", "effort"))
    grip_act = np.array(hello["grip_act"])
    m.dof_armature[vadr] = np.array(hello["armature"])
    for adr, value in hello["init_qpos"]:
        d.qpos[adr] = value
    mujoco.mj_forward(m, d)
    r1 = {"n": 0, "target": np.array(hello["default"]), "grip": np.array(hello["init_grip"]), "late": 0}
    period = int(hello["substeps"])

    def _r1_step(self):
        if r1["n"] % period == 0:
            req.send(np.concatenate(([d.time], d.qpos, d.qvel)).astype(np.float64).tobytes())
            if req.poll(2000):
                reply = np.frombuffer(req.recv(), dtype=np.float64)
                r1["target"], r1["grip"] = reply[:len(act)], reply[len(act):len(act) + len(grip_act)]
            else:
                raise RuntimeError("r1_brain.py stopped answering: the R1 has no controller")
        r1["n"] += 1
        d.ctrl[act] = np.clip(kp * (r1["target"] - d.qpos[qadr]) - kd * d.qvel[vadr], -effort, effort)
        d.ctrl[grip_act] = r1["grip"]

    namespace = dict(sys.modules[type(env).__module__].__dict__)
    namespace["_r1_step"] = _r1_step
    exec(source, namespace)
    env.sim_step = types.MethodType(namespace["sim_step"], env)
    print(f"[office_sim] R1 attached: {len(act)} joints from r1_brain.py every {period} physics steps", flush=True)


def run_lockstep(sim, env, ctx, args):
    """One control period of physics per deploy control tick; real time while the deploy sends none."""
    ticks = ctx.socket(zmq.SUB)
    ticks.setsockopt(zmq.SUBSCRIBE, b"g1_debug")
    # no CONFLATE: every deploy tick must advance the physics by one control period. With CONFLATE, ticks that
    # arrived while the sim was busy were merged, the physics fell behind the deploy's reference timeline, and the
    # robot lagged its walking plan until the policy lunged to catch up (falls on a busy host).
    ticks.connect(f"tcp://127.0.0.1:{args.state_port}")
    bunched = late = 0
    ever_locked = False
    steps = max(1, int(round(args.control_dt / sim.sim_dt)))
    last_tick, locked, waits, period_start = time.monotonic() - 10.0, False, [], time.monotonic()
    while sim._running:
        got = ticks.poll(timeout=int(sim.sim_dt * 1000))
        now = time.monotonic()
        if getattr(env, "lockstep_restart", False):  # the G1 hangs on the gantry again: real time until new ticks
            env.lockstep_restart, ever_locked, locked = False, False, False
            print("[sim_headless] lockstep: waiting for a restarted deploy stack; physics in real time", flush=True)
        if got:
            n = 0
            while True:  # every pending tick
                try:
                    ticks.recv(zmq.NOBLOCK)
                    n += 1
                except zmq.Again:
                    break
            if not locked:
                print("[sim_headless] lockstep: following the deploy's control ticks", flush=True)
            locked = ever_locked = True
            waits.append(now - last_tick)
            last_tick = now
            bunched += n > 1
            # this tick's command reaches us over DDS (the deploy writes LowCmd at 500 Hz): wait for two fresh ones
            # after the tick instead of a fixed 3 ms, which a busy host exceeds (a step on the previous command)
            bridge = getattr(env, "unitree_bridge", None)
            if bridge is not None and hasattr(bridge, "new_low_cmd"):
                deadline, fresh = time.monotonic() + 0.025, 0
                bridge.new_low_cmd = False
                while fresh < 2 and time.monotonic() < deadline:
                    if bridge.new_low_cmd:
                        fresh += 1
                        bridge.new_low_cmd = False
                    else:
                        time.sleep(0.0005)
                late += fresh < 2
            else:
                time.sleep(0.003)
            for _ in range(steps * n):
                env.sim_step()
        elif not ever_locked and now - last_tick > args.lockstep_idle_s:
            env.sim_step()  # before the deploy's control runs (the gantry phase): about real time
        elif ever_locked and now - last_tick > 1.0 and locked:
            # once following the deploy, physics waits for it: a deploy stalled on a busy GPU (seconds) used to
            # leave the sim running on stale commands, and the robot fell
            print(f"[sim_headless] lockstep: no tick for {now - last_tick:.1f} s; physics paused", flush=True)
            locked = False
        if waits and now - period_start > 30.0:
            w = np.array(waits)
            print(f"[sim_headless] lockstep: {len(w)} ticks in 30 s, gap mean {w.mean() * 1000:.0f} ms, "
                  f"max {w.max() * 1000:.0f} ms (real-time factor {len(w) * args.control_dt / 30.0:.2f}), "
                  f"{bunched} bunched, {late} without a fresh command", flush=True)
            waits, period_start, bunched, late = [], now, 0, 0


if __name__ == "__main__":
    main()
