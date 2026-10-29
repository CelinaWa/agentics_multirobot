"""Headless MuJoCo G1 for the SONIC sim2sim loop (no window, no keyboard).

Same simulator as `gear_sonic/scripts/run_sim_loop.py --env-name default`, but
onscreen rendering is off and the sim's keyboard shortcuts are driven by a ZMQ
SUB socket instead of the MuJoCo window: any string received on
tcp://localhost:<port> is passed to `handle_keyboard_button`, so "9" releases
the elastic band (the drop step), "backspace" resets the sim; "7" / "8" lower /
raise the band by 0.1 m like the viewer's keys; "band=<m>" sets the band's
length and "release" / "attach" set its state outright (the gantry uses these,
which repeat safely, unlike the "9" toggle). `--scene` swaps
the scene xml and `--qpos-port` streams qpos for sim_camera_server.py.
With --lockstep the sim advances one control period (20 ms) per control tick the deploy stack
publishes on its state port (`g1_debug`, sent only while it is in CONTROL), so a deploy that
runs late on a shared GPU slows the sim down instead of letting the robot fall; before CONTROL,
and whenever no tick arrives for --lockstep-idle-s, it runs in real time as usual.
Adapted from MA-Teleop's inference/g1/sim_headless.py. Started by run_sim2sim.sh:

  $SONIC_ROOT/.venv_sim/bin/python sim_headless.py [--scene <abs xml>]   # prints READY
  python -c "import zmq,time;s=zmq.Context().socket(zmq.PUB);s.bind('tcp://127.0.0.1:5590');time.sleep(.5);s.send_string('9')"

Needs the CycloneDDS loopback export like every sim-side terminal.
"""

import argparse
import sys
import threading
import time

import numpy as np
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
    ap.add_argument("--qpos-port", type=int, default=5591,
                    help="ZMQ PUB port streaming the sim's qpos at 50 Hz (sim_camera_server.py mirrors it)")
    args = ap.parse_args()

    config = SimLoopConfig(env_name=args.env_name, enable_onscreen=False, enable_offscreen=False)
    wbc = config.load_wbc_yaml()
    wbc["ENV_NAME"] = args.env_name
    if args.scene:
        wbc["ROBOT_SCENE"] = args.scene
    if args.sim_dt:
        wbc["SIMULATE_DT"] = args.sim_dt
    init_channel(config=wbc)
    sim = SimulatorFactory.create_simulator(config=wbc, env_name=args.env_name,
                                            onscreen=False, offscreen=False, enable_image_publish=False)

    ctx = zmq.Context.instance()
    sub = ctx.socket(zmq.SUB)
    sub.setsockopt_string(zmq.SUBSCRIBE, "")
    sub.connect(f"tcp://127.0.0.1:{args.key_port}")

    env = getattr(sim, "sim_env", sim)
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
            elif key in ("release", "attach") and band is not None:
                band.enable = key == "attach"
                print(f"[sim_headless] band {'attached' if band.enable else 'released'}", flush=True)
            elif key.startswith("joint="):  # joint=<scene joint>,value : the operator sets a door or drawer, at rest
                name, value = key[6:].split(",")
                joint = env.mj_model.joint(name)
                env.mj_data.qpos[env.mj_model.jnt_qposadr[joint.id]] = float(value)
                env.mj_data.qvel[env.mj_model.jnt_dofadr[joint.id]] = 0.0
                print(f"[sim_headless] set {name} to {value}", flush=True)
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
    print(f"[sim_headless] READY (keys on tcp://127.0.0.1:{args.key_port}, qpos on {args.qpos_port}, "
          f"scene {wbc['ROBOT_SCENE']}{', lockstep' if args.lockstep else ''})", flush=True)
    sys.stdout.flush()
    if not args.lockstep:
        SimulatorFactory.start_simulator(sim, as_thread=False, enable_image_publish=False)
        return
    run_lockstep(sim, env, ctx, args)


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
