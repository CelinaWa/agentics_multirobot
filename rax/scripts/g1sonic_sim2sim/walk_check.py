"""Show that the G1 walks under SONIC: start control through the g1sonic backend, send the agent's
stepping gait (skill stepping-gait-sinusoid: sinusoidal swing, sawtooth stride, forward shin lean)
for N calls, and record from the sim's own physics state (qpos stream) the pelvis path and both
feet's heights at 20 Hz plus the overhead camera at 10 fps. Prints when each foot leaves and
touches the floor, so stepping is measured, not inferred. Needs run_sim2sim.sh with SCENE running.

    $SONIC_ROOT/.venv_sim/bin/python walk_check.py --out DIR [--calls 4]
"""
import argparse
import math
import os
import sys
import threading
import time
from pathlib import Path

os.environ.setdefault("MUJOCO_GL", "osmesa")
os.environ.setdefault("PYOPENGL_PLATFORM", "osmesa")
import cv2  # noqa: E402
import mujoco  # noqa: E402
import numpy as np  # noqa: E402
import zmq  # noqa: E402

sys.path.insert(0, str(Path(__file__).resolve().parents[2]))
os.environ.setdefault("RAX_ARTIFACT_ROOT", "/tmp/rax-walkcheck")
from services import arming  # noqa: E402
from services.controller.config import Configuration  # noqa: E402
from services.g1sonic import robot as R  # noqa: E402
from services.g1sonic import tools as T  # noqa: E402
from services.g1sonic.backend import G1SonicBackend  # noqa: E402

SCENE = "/opt/sonic/gear_sonic/data/robot_model/model_data/g1/scene_office.xml"
KNEE_PER_LIFT = 2.0


def gait_rows(joints, cycles=2, period=0.9, lift=0.25, stride=0.12, lean=0.15):
    """The agent's gait (stepping-gait-sinusoid v2), heading 0."""
    idx = {n.replace("_joint", ""): i for i, n in enumerate(R.JOINT_NAMES)}
    n = int(cycles * period * 50)
    rows = []
    for i in range(1, n + 1):
        t = i / 50.0
        ph = 2 * math.pi * t / period
        env = max(0.0, min(1.0, t / (0.5 * period), (n / 50.0 - t) / (0.5 * period)))
        s = math.sin(ph)
        q = np.array(joints, dtype=float)
        for side, sw, sign in (("left", max(0, s), 1), ("right", max(0, -s), -1)):
            dh = env * (-lift * sw + stride * math.cos(ph) * sign)
            dk = env * KNEE_PER_LIFT * lift * sw
            q[idx[f"{side}_hip_pitch"]] += dh + env * lean
            q[idx[f"{side}_knee"]] += dk
            q[idx[f"{side}_ankle_pitch"]] += -(dh + dk) - env * lean
        row = np.zeros(R.ACTION_DIM)
        row[2] = 0.75
        row[R.ROT6D] = R.IDENTITY_ROT6D
        row[R.JOINTS] = q
        rows.append(row)
    final = rows[-1].copy()
    final[R.JOINTS] = joints
    return rows[:-1] + [final] * 6


def record(stop, out, log, qposes):
    """Log the sim's full physics state at 50 Hz (rendering happens afterwards, so nothing is skipped)."""
    sub = zmq.Context.instance().socket(zmq.SUB)
    sub.setsockopt(zmq.SUBSCRIBE, b"")
    sub.setsockopt(zmq.CONFLATE, 1)
    sub.connect("tcp://127.0.0.1:5591")
    while not stop.is_set():
        started = time.time()
        qposes.append((started, np.frombuffer(sub.recv(), dtype=np.float64).copy()))
        time.sleep(max(0.0, 0.02 - (time.time() - started)))


def render_and_measure(qposes, out, log):
    """Feet heights, foot contact slip and a side view of every logged state."""
    model = mujoco.MjModel.from_xml_path(SCENE)
    data = mujoco.MjData(model)
    renderer = mujoco.Renderer(model, height=360, width=480)
    ids = [model.body(n).id for n in ("pelvis", "left_ankle_roll_link", "right_ankle_roll_link")]
    cam = mujoco.MjvCamera()  # a side view that follows the pelvis
    cam.type = mujoco.mjtCamera.mjCAMERA_TRACKING
    cam.trackbodyid = ids[0]
    cam.distance, cam.azimuth, cam.elevation = 2.6, 150.0, -12.0
    np.savez_compressed(out / "qpos.npz", stamp=np.array([q[0] for q in qposes]), qpos=np.array([q[1] for q in qposes]))
    for stamp, qpos in qposes:
        data.qpos[:] = qpos
        mujoco.mj_kinematics(model, data)
        log.append((stamp, *data.xpos[ids[0]], data.xpos[ids[1]][2], data.xpos[ids[2]][2],
                    *data.xpos[ids[1]][:2], *data.xpos[ids[2]][:2]))
    summarize(np.array(log))
    for k, (stamp, qpos) in enumerate(qposes):
        data.qpos[:] = qpos
        mujoco.mj_forward(model, data)
        renderer.update_scene(data, camera=cam)
        cv2.imwrite(str(out / f"f{k:04d}.png"), renderer.render()[:, :, ::-1])


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--out", required=True)
    ap.add_argument("--calls", type=int, default=4)
    ap.add_argument("--planner", action="store_true", help="walk with g1_walk (SONIC's planner) instead of the joint gait")
    ap.add_argument("--walks", default=None, help='JSON list of g1_walk arguments, e.g. [{"forward_m": -0.4}]')
    args = ap.parse_args()
    out = Path(args.out)
    out.mkdir(parents=True, exist_ok=True)
    root = Path(os.environ["RAX_ARTIFACT_ROOT"]) / "sessions" / "walk_check"
    (root / "agent-workspace").mkdir(parents=True, exist_ok=True)
    backend = G1SonicBackend(Configuration(root=root, task="walk", robot="g1sonic", task_description="walk check",
                                           backend_options={"zmq_host": "127.0.0.1", "pose_port": 5556,
                                                            "state_host": "127.0.0.1", "state_port": 5557,
                                                            "camera": None, "state_max_age_s": 0.5}))
    log, qposes, stop = [], [], threading.Event()
    try:
        arming.write(root, True, by="walk_check")
        value, _ = backend.call(T.HOLD, {})
        print(f"control started: {value.get('start_error') or 'ok'}, tilt {value.get('tilt_deg')}", flush=True)
        time.sleep(9.0)  # the gantry releases 8 s after control starts
        recorder = threading.Thread(target=record, args=(stop, out, log, qposes), daemon=True)
        recorder.start()
        time.sleep(1.0)
        joints = np.array(backend.status()["proprio"]["joint_positions_rad"])
        if args.planner:
            import json
            walks = json.loads(args.walks) if args.walks else [{"forward_m": 0.6, "speed_mps": 0.3}, {"turn_deg": 45},
                                                               {"forward_m": 0.4, "left_m": 0.2, "speed_mps": 0.3}]
            for call, walk in enumerate(walks[:args.calls]):
                before = backend.state.latest()
                p0 = qposes[-1][1][:3].copy() if qposes else None
                value, _ = backend.call(T.WALK, walk)
                fb = value["transition"]["feedback"]
                time.sleep(0.3)
                moved = ""
                if p0 is not None and qposes:
                    yaw = before.yaw_rad()
                    dx, dy = (qposes[-1][1][:2] - p0[:2])
                    fwd, left = dx * math.cos(yaw) + dy * math.sin(yaw), -dx * math.sin(yaw) + dy * math.cos(yaw)
                    moved = f", pelvis moved forward {fwd:+.2f} m, left {left:+.2f} m"
                print(f"walk {call + 1} {walk}: turned {fb['measured_turn_deg']} deg, max tilt {fb['max_tilt_deg']}{moved}", flush=True)
        else:
            for call in range(args.calls):
                value, _ = backend.call(T.STEP, {"actions": [r.tolist() for r in gait_rows(joints)]})
                fb = value["transition"]["feedback"]
                print(f"gait call {call + 1}: {fb['ticks']} ticks, tilt {fb['tilt_deg']} deg", flush=True)
                time.sleep(0.5)
        time.sleep(1.5)
    finally:
        stop.set()
        backend.close()
    render_and_measure(qposes, out, log)


def summarize(a):
    """Steps and slip from a log of (time, pelvis xyz, foot z L/R, foot xy L/R)."""
    t, pelvis, lz, rz = a[:, 0] - a[0, 0], a[:, 1:3], a[:, 4], a[:, 5]
    print(f"\n{len(a)} physics samples over {t[-1]:.1f} s (sim at ~0.8x real time)")
    print(f"pelvis moved {np.linalg.norm(pelvis[-1] - pelvis[0]):.2f} m; largest jump between samples "
          f"{np.max(np.linalg.norm(np.diff(pelvis, axis=0), axis=1)) * 100:.1f} cm")
    for name, z, xy in (("left", lz, a[:, 6:8]), ("right", rz, a[:, 8:10])):
        up = z > z.min() + 0.015
        lifts = np.flatnonzero(np.diff(up.astype(int)) == 1)
        moved = np.linalg.norm(np.diff(xy, axis=0), axis=1)
        swing, stance = moved[up[1:]].sum(), moved[~up[1:]].sum()
        print(f"{name} foot: {len(lifts)} lift-offs above 1.5 cm, highest {100 * (z.max() - z.min()):.1f} cm; travelled "
              f"{swing:.2f} m in the air and {stance:.2f} m while down (sliding)")


if __name__ == "__main__":
    main()
