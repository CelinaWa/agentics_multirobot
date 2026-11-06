"""Real-robot bridge skeleton: serves the r1 stack protocol (JSON over ZMQ REQ/REP, see stack.py) that the RAX
r1sonic backend speaks, on top of the real SONIC_R1 deployment (gear_sonic_deploy, zmq_manager wire format).

STATUS: skeleton. The protocol server (this file) is complete; the `RealStack` adapter methods marked TODO must be
implemented against the deploy stack next to the robot, then validated with r1/tools/diag/carry_chain_test.py pointed
at this bridge's port BEFORE any RAX session on the robot.

Ops the backend uses (rax/services/r1sonic/backend.py, stack.py):
  state      -> {"ok", "t", "tick", "joint_names", "q", "dq", "base_quat_wxyz", "gyro", "gripper_width_m": {left,right},
                 "command_row", "queued", "measured_points_row", "fallen", "tilt_deg", "mode", "resets"}
  stream     <- {"rows": [[24 floats] * n]} at 50 Hz: for k in (left wrist, right wrist, chest): pos xyz + quat wxyz in
                 the REFERENCE frame (heading at session start = 0), then left gripper width m, right gripper width m,
                 head pitch rad. Replaces any queued rows. Reply {"ok": true}.
  hold       -> drop queued rows, hold the current reference. {"ok": true, "held": true}
  walk       <- {"forward_m", "turn_deg"}: blocks until the robot is standing still; reply {"walked_m", "turned_deg",
                 "lateral_m", "pelvis": [x, y, z] (odometry), "tilt_deg", "fallen", "passes"}.
  grasp_paw, place_in_box -> may answer {"ok": true, "refused": "not available on the real robot", "held": false}.
  head       <- {"pitch"} (the sim also accepts head pitch inside stream rows; keep both).
  shutdown   -> stop serving.
Run:  python -m services.r1sonic.bridge_real --port 5591
"""
from __future__ import annotations

import argparse
import json
import threading
import time

import numpy as np
import zmq

JOINT_NAMES_ISAAC_ORDER = None   # TODO: the 26 names from SONIC_R1_100k/validation/r1_deployment_contract.json


class RealStack:
    """Adapter onto the deploy stack. Every method here is what the operator must wire on the robot PC."""

    def __init__(self):
        self.resets = 0
        self.row = np.zeros(24)          # last commanded reference row
        self.queue: list[np.ndarray] = []
        self.lock = threading.Lock()
        # TODO: connect to zmq_manager topics (telemetry subscriber, reference publisher, locomotion command publisher)

    # ---- telemetry ----
    def state(self) -> dict:
        """Latest telemetry in the backend's field names. Must be fresh (< 40 ms) and never block."""
        raise NotImplementedError('TODO: joint q/dq in Isaac order, base quat relative to the reference heading, gyro, '
                                  'gripper widths (m), fall flag, tilt (deg)')

    # ---- 3-point teleop reference ----
    def stream(self, rows: list[np.ndarray]) -> None:
        """Queue reference rows; a 50 Hz loop publishes one per tick (reference frame -> deploy stack frame)."""
        with self.lock:
            self.queue = rows

    def hold(self) -> None:
        with self.lock:
            self.queue = []

    def tick(self) -> None:
        """Called at 50 Hz: publish the next queued row (or re-publish the last one)."""
        with self.lock:
            if self.queue:
                self.row = self.queue.pop(0)
        # TODO: publish self.row as the stack's teleop reference (3 points + grippers + head)

    # ---- locomotion ----
    def walk(self, forward_m: float, turn_deg: float) -> dict:
        """Command the stack's locomotion for the given displacement/turn, block until still, measure with odometry."""
        raise NotImplementedError('TODO: locomotion command + odometry; return walked_m, turned_deg, lateral_m, pelvis, '
                                  'tilt_deg, fallen, passes')


def serve(stack: RealStack, port: int) -> None:
    ctx = zmq.Context.instance()
    sock = ctx.socket(zmq.REP)
    sock.bind(f'tcp://127.0.0.1:{port}')
    stop = threading.Event()

    def loop():
        period = 1.0 / 50.0
        while not stop.is_set():
            t0 = time.monotonic()
            try:
                stack.tick()
            except Exception as exc:  # noqa: BLE001  (keep ticking; the operator reads the log)
                print('tick error:', exc, flush=True)
            time.sleep(max(0.0, period - (time.monotonic() - t0)))
    threading.Thread(target=loop, daemon=True).start()
    print(json.dumps({'ready': True, 'port': port, 'bridge': 'real'}), flush=True)
    while not stop.is_set():
        req = json.loads(sock.recv())
        op = req.get('op')
        try:
            if op == 'state':
                out = {**stack.state(), 'resets': stack.resets, 'mode': 'teleop'}
            elif op == 'stream':
                stack.stream([np.asarray(r, dtype=float) for r in req['rows']]); out = {}
            elif op == 'hold':
                stack.hold(); out = {'held': True}
            elif op == 'walk':
                out = stack.walk(float(req['forward_m']), float(req.get('turn_deg', 0.0)))
            elif op == 'head':
                with stack.lock:
                    stack.row[23] = float(req['pitch'])
                out = {'head_pitch': float(req['pitch'])}
            elif op in ('grasp_paw', 'place_in_box'):
                out = {'refused': 'not available on the real robot: use r1_leg_lift / r1_carry_release', 'held': False, 'in_box': False}
            elif op == 'shutdown':
                stop.set(); out = {}
            else:
                raise ValueError(f'unknown op {op!r}')
            sock.send(json.dumps({'ok': True, **out}).encode())
        except Exception as exc:  # noqa: BLE001
            sock.send(json.dumps({'ok': False, 'error': f'{type(exc).__name__}: {exc}'}).encode())


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('--port', type=int, default=5591)
    a = ap.parse_args()
    serve(RealStack(), a.port)


if __name__ == '__main__':
    main()
