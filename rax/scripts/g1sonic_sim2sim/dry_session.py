"""A dry RAX session against the sim2sim stack: the robot service as the agent sees it, without the agent.

Configures a g1sonic session (scripts/configure_session.py), starts the robot service
(services.controller.frontend) on stdio and drives it over MCP JSON-RPC the way Claude Code's
bridge does: initialize, tools/list, status, g1_status before arming, then arm, g1_hold (starts the
stack's control), g1_status, a small g1_step, g1_observe, g1_hold. Prints every result's summary and
where the session recorded (mcp.jsonl, runtime/frames). Needs run_sim2sim.sh running (with SCENE and
its camera for --camera). Run with the sim venv: RAX_ARTIFACT_ROOT=... python dry_session.py [--camera]
"""
import base64
import json
import os
import subprocess
import sys
import time
from pathlib import Path

RAX = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(RAX))
os.environ.setdefault("RAX_ARTIFACT_ROOT", "/tmp/rax-sim2sim-artifacts")

import numpy as np  # noqa: E402

from services import arming  # noqa: E402
from services.g1sonic import robot as R  # noqa: E402
from sim_keys import lower_and_release  # noqa: E402


def log(*parts):
    print(time.strftime("%H:%M:%S"), *parts, flush=True)


class Client:
    def __init__(self, process):
        self.process, self.ident = process, 0

    def request(self, method, params=None):
        self.ident += 1
        line = json.dumps({"jsonrpc": "2.0", "id": self.ident, "method": method, "params": params or {}}) + "\n"
        self.process.stdin.write(line)
        self.process.stdin.flush()
        reply = self.process.stdout.readline()
        if not reply:
            raise RuntimeError("the robot service closed its stdout; see its stderr above")
        reply = json.loads(reply)
        if "error" in reply:
            raise RuntimeError(f"{method}: {reply['error']}")
        return reply["result"]

    def call(self, name, arguments=None):
        result = self.request("tools/call", {"name": name, "arguments": arguments or {}})
        texts = [c["text"] for c in result["content"] if c["type"] == "text"]
        images = [c for c in result["content"] if c["type"] == "image"]
        value = json.loads(texts[0]) if texts else None
        if result.get("isError"):
            raise RuntimeError(f"{name}: {value}")
        return value, images


def frame(joints, yaw=0.0):
    row = np.zeros(R.ACTION_DIM)
    row[2] = 0.75
    row[R.ROT6D] = R.rot6d_from_yaw(yaw)
    row[R.JOINTS] = joints
    return row


def main():
    camera = "--camera" in sys.argv
    options = {"zmq_host": "127.0.0.1", "pose_port": 5556, "state_host": "127.0.0.1", "state_port": 5557,
               "state_max_age_s": 0.5, "max_first_step_rad": 0.6,
               **({"camera": "network", "camera_host": "127.0.0.1", "camera_port": 7790} if camera else {"camera": None})}
    options_path = Path(os.environ["RAX_ARTIFACT_ROOT"]) / "dry_session_backend_options.json"
    options_path.parent.mkdir(parents=True, exist_ok=True)
    options_path.write_text(json.dumps(options))
    configured = subprocess.run([sys.executable, str(RAX / "scripts/configure_session.py"), "--task", "sim2sim_dry",
                                 "--task-text", "Stand still, look around and report what you see.", "--robot", "g1sonic",
                                 "--backend-options", str(options_path), "--session-hours", "0.5"],
                                cwd=RAX, env={**os.environ, "PYTHONPATH": str(RAX)}, capture_output=True, text=True)
    if configured.returncode:
        sys.exit(f"configure_session failed:\n{configured.stdout}\n{configured.stderr}")
    config_path = Path(configured.stdout.strip().splitlines()[-1])
    root = config_path.parent
    log(f"session {root}")
    service = subprocess.Popen([sys.executable, "-m", "services.controller.frontend", "--config", str(config_path)],
                               cwd=RAX, env={**os.environ, "PYTHONPATH": str(RAX)}, stdin=subprocess.PIPE,
                               stdout=subprocess.PIPE, stderr=sys.stderr, text=True, bufsize=1)
    client = Client(service)
    try:
        info = client.request("initialize", {"protocolVersion": "2025-06-18", "capabilities": {},
                                             "clientInfo": {"name": "dry_session", "version": "0"}})
        log(f"initialize: {info['serverInfo']}")
        tools = [t["name"] for t in client.request("tools/list")["tools"]]
        log(f"tools/list: {tools}")
        status, _ = client.call("status")
        log(f"status: task={status['task']} robot={status['robot']} profile={status['observation_profile']}")
        robot, _ = client.call("g1_status")
        log(f"g1_status before arming: mode={robot['stack']['mode']} proprio={'proprio' in robot} motion={robot['motion']}")
        if "proprio" in robot:
            log("WARNING: the stack is already in CONTROL from an earlier session (state flows before any start); "
                "restart run_sim2sim.sh for a fresh standing pose, as a hanging robot left in control drifts")
        try:
            client.call("g1_step", {"actions": [frame(np.zeros(29)).tolist()]})
        except RuntimeError as exc:
            log(f"g1_step unarmed -> refused as expected: {str(exc)[:140]}")
        arming.write(root, True, by="dry_session")
        held, _ = client.call("g1_hold")
        log(f"g1_hold armed: primed={held['streaming']['primed']} tilt={held.get('tilt_deg')} start_error={held.get('start_error')}")
        if "--no-drop" not in sys.argv:
            log("lowering the robot to the floor and releasing the band (the gantry)")
            lower_and_release()
            time.sleep(2.5)
        robot, _ = client.call("g1_status")
        q0 = np.array(robot["proprio"]["joint_positions_rad"])
        lower, upper = (np.array(robot["joint_limits_rad"][k]) for k in ("lower", "upper"))
        log(f"g1_status: mode={robot['stack']['mode']} tilt={robot['tilt_deg']} standing joints by name:")
        for name, value in zip(robot["joint_names_canonical29"], q0):
            print(f"   {name:28s} {value:+.3f}", flush=True)
        margin = robot["limits"]["joint_limit_margin_rad"] + 0.01
        q0 = np.clip(q0, lower + margin, upper - margin)  # a measured joint can sit on its limit; a reference may not
        shoulder = R.JOINT_NAMES.index("right_shoulder_pitch_joint")
        target = q0.copy()
        target[shoulder] -= 0.3
        rows = [frame(q0 + (target - q0) * (i + 1) / 30) for i in range(30)] + [frame(target)] * 5
        value, images = client.call("g1_step", {"actions": [r.tolist() for r in rows]})
        fb = value["transition"]["feedback"]
        log(f"g1_step: step_id={value['step_id']} ticks={fb['ticks']} tilt={fb['tilt_deg']} "
            f"tracking_rmse={fb.get('joint_tracking_rmse_rad', 0):.3f} images={len(images)}")
        value, images = client.call("g1_observe")
        kinds = [a["kind"] for a in value["attachments"]]
        log(f"g1_observe: attachments={kinds} images={len(images)} proprio_joints={len(value['proprio']['joint_positions_rad'])} "
            f"camera={value.get('camera', {}).get('frame_id')} sensors={value.get('sensors')}")
        if images:
            raw = base64.b64decode(images[0]["data"])
            out = root / "dry_session_rgb.jpg"
            out.write_bytes(raw)
            log(f"first image: {images[0]['mimeType']} {len(raw)} bytes -> {out}")
        held, _ = client.call("g1_hold")
        log(f"g1_hold: dropped_rows={held['dropped_rows']} step_id={held['step_id']}")
        audit = (root / "mcp.jsonl").read_text().splitlines()
        log(f"recorded {len(audit)} audit lines in {root / 'mcp.jsonl'}; frames under {root / 'agent-workspace/runtime/frames'}")
    finally:
        arming.write(root, False, by="dry_session", reason="dry session over")
        service.stdin.close()
        service.terminate()
        try:
            service.wait(10)
        except subprocess.TimeoutExpired:
            service.kill()
        log(f"robot service exited {service.returncode}")


if __name__ == "__main__":
    main()
