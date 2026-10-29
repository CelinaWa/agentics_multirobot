"""Drive the SONIC deploy stack (MuJoCo sim2sim, run_sim2sim.sh) through the RAX g1sonic backend.

Assumes sim_headless.py and g1_deploy_onnx_ref (--input-type zmq_manager with a planner, pose 5556,
state 5557) are running in this container. The stack publishes state only once control runs, so
the backend's first armed motion starts it (planner mode -> measured pose -> streamed motion).
Steps: arm, hold (starts control), drop the robot ('9' to the sim), then: raise the right arm,
lower it, turn the waist, turn the heading, hold; with --camera, observe through sim_camera_server.py
at the end. Prints the feedback of every step. Set RAX_ARTIFACT_ROOT to keep the session's frames.
"""
import json
import math
import os
import sys
import time
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[2]))
os.environ.setdefault("RAX_ARTIFACT_ROOT", "/tmp/rax-sim2sim-artifacts")

from services import arming  # noqa: E402
from services.controller.config import Configuration  # noqa: E402
from services.g1sonic import robot as R  # noqa: E402
from services.g1sonic import tools as T  # noqa: E402
from services.g1sonic.backend import G1SonicBackend, MotionStopped  # noqa: E402
from services.g1sonic.bridge import yaw_of  # noqa: E402
from sim_keys import lower_and_release  # noqa: E402


def log(*parts):
    print(time.strftime("%H:%M:%S"), *parts, flush=True)


def frame(joints, yaw=0.0, hands=(0, 0)):
    row = np.zeros(R.ACTION_DIM)
    row[2] = 0.75
    row[R.ROT6D] = R.rot6d_from_yaw(yaw)
    row[R.JOINTS] = joints
    row[38], row[39] = hands
    return row


def ramp(start, end, rows, yaw0=0.0, yaw1=None, tail=10):
    yaw1 = yaw0 if yaw1 is None else yaw1
    out = [frame(start + (end - start) * (i + 1) / rows, yaw0 + (yaw1 - yaw0) * (i + 1) / rows) for i in range(rows)]
    return out + [frame(end, yaw1)] * tail


def drop_robot():
    lower_and_release()


def main():
    root = Path(os.environ["RAX_ARTIFACT_ROOT"]) / "sessions" / "sim2sim"
    (root / "agent-workspace").mkdir(parents=True, exist_ok=True)
    options = {"zmq_host": "127.0.0.1", "pose_port": 5556, "state_host": "127.0.0.1", "state_port": 5557,
               "camera": None, "state_max_age_s": 0.5, "max_first_step_rad": 0.6}
    if "--camera" in sys.argv:  # sim_camera_server.py serving the scene's head_cam
        options.update(camera="network", camera_host="127.0.0.1", camera_port=7790)
    config = Configuration(root=root, task="stand", robot="g1sonic", task_description="sim2sim probe",
                           backend_options=options)
    backend = G1SonicBackend(config)
    try:
        snap = backend.state.wait(2.0)
        log("state before start: " + ("none (expected: the stack publishes only in CONTROL)" if snap is None else f"index {snap.index}"))
        arming.write(root, True, by="rax_sim_probe")

        def step(rows, label):
            try:
                value, _ = backend.call(T.STEP, {"actions": [r.tolist() for r in rows]})
            except MotionStopped as exc:
                log(f"{label}: STOPPED {exc}")
                return None
            fb = value["transition"]["feedback"]
            keys = ("ticks", "joint_tracking_rmse_rad", "max_joint_tracking_error_rad", "reference_error_rmse_rad",
                    "max_root_orientation_projection_deg", "tilt_deg", "fallen", "wall_seconds")
            log(f"{label}: " + json.dumps({k: (round(fb[k], 4) if isinstance(fb[k], float) else fb[k]) for k in keys if k in fb}))
            return value

        t0 = time.monotonic()
        value, _ = backend.call(T.HOLD, {})
        log(f"hold -> control started in {time.monotonic() - t0:.2f} s; stack: " + json.dumps(backend.status()["stack"]))
        snap = backend.state.latest()
        log(f"state: index {snap.index}, tilt {snap.tilt_deg():.1f} deg, yaw {math.degrees(snap.yaw_rad()):.1f} deg")
        log("joints (IsaacLab order after the MuJoCo->IsaacLab permutation):")
        for name, value in zip(R.JOINT_NAMES, snap.q):
            print(f"   {name:28s} {value:+.3f}")
        if "--no-drop" not in sys.argv:
            log("lowering the robot to the floor and releasing the band ('7','7','9' to sim_headless)")
            drop_robot()
            time.sleep(2.0)
        s = backend.state.latest()
        if s.last_action is not None:
            error = s.q - s.last_action
            log(f"after drop: tilt {s.tilt_deg():.1f} deg, |q - last_action| max {np.abs(error).max():.3f} rad, rmse {np.sqrt(np.mean(error**2)):.3f}")
        q0 = backend.state.latest().q.copy()
        yaw0 = 0.0
        step([frame(q0, yaw0)] * 25, "hold 0.5 s")
        shoulder = R.JOINT_NAMES.index("right_shoulder_pitch_joint")
        raised = q0.copy()
        raised[shoulder] = q0[shoulder] - 0.8
        step(ramp(q0, raised, 60, yaw0), "raise right arm 0.8 rad over 1.2 s")
        time.sleep(0.5)
        s = backend.state.latest()
        log(f"   measured right_shoulder_pitch {s.q[shoulder]:+.3f} (reference {raised[shoulder]:+.3f})")
        step(ramp(raised, q0, 60, yaw0), "lower right arm")
        waist = R.JOINT_NAMES.index("waist_yaw_joint")
        turned = q0.copy()
        turned[waist] = q0[waist] + 0.4
        step(ramp(q0, turned, 50, yaw0), "waist yaw +0.4 rad")
        time.sleep(0.5)
        log(f"   measured waist_yaw {backend.state.latest().q[waist]:+.3f} (reference {turned[waist]:+.3f})")
        step(ramp(turned, q0, 50, yaw0), "waist back")
        before = backend.state.latest()
        step(ramp(q0, q0, 60, yaw0, math.radians(30), tail=40), "heading +30 deg over 1.2 s (stepping turn expected)")
        time.sleep(1.5)
        after = backend.state.latest()
        log(f"   measured heading change {math.degrees(after.yaw_rad() - before.yaw_rad()):+.1f} deg, tilt {after.tilt_deg():.1f} deg")
        step([frame(q0, math.radians(30))] * 25, "hold")
        status = backend.status()
        log("status.stack: " + json.dumps(status["stack"]))
        if "--camera" in sys.argv:
            log("sensors: " + json.dumps(status["sensors"]))
            value, images = backend.call(T.OBSERVE, {})
            cam = value["camera"]
            log(f"observe: {len(images)} attachments, frame {cam['frame_id']}, intrinsics {value['camera_intrinsics']['cam_front']}")
            log("camera pose (pelvis frame): " + json.dumps(value["camera_poses"]["cam_front"]))
            for attachment in value["attachments"]:
                log(f"   {attachment['kind']}: {attachment.get('path', attachment)}")
            log(f"frames dir: {backend.frames_root / cam['frame_id']}")
        log("done; the stream keeps holding the last frame until this process exits")
        time.sleep(1.0)
    finally:
        backend.close()


if __name__ == "__main__":
    main()
