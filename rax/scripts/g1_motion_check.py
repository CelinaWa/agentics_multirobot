"""Operator-supervised motion check of the G1 under SONIC through the real G1SonicBackend (the agent's
tools and guards), one step per run. The RB-Y1 port's `rby1_motion_check.py` is the template.

    $PY scripts/configure_session.py --task motion_check --task-text "motion check" --robot g1sonic \\
        --backend-options <options.json>
    $PY scripts/g1_motion_check.py <session_root> <step> [--repeat N] [--forward M] [--turn DEG] \\
        [--pelvis-z M] [--desk-z M] [--self-disarm] [--no-prompt]

Run `scripts/g1_console.py <session_root>` in another terminal: `s` stands the robot, SPACE arms it. The
step waits up to 5 minutes for the arming, runs, prints its numbers and appends them as JSON to
<session_root>/runtime/characterization/g1_motion_check.jsonl (one line per step, with the date, for the
robot's G1.md). Every motion is small, starts from the measured pose and returns to it. A person stays
at the stop (console `o`, the deploy's `O`, the gamepad's Select).

Steps, in the order to run them (the robot stands on its feet, straps slack, from `s` on):
  status      connect and print the backend status: the stack link, the state age, armed or not. Moves nothing.
  disarmed    a g1_step while disarmed must be refused (prints the refusal text). Moves nothing.
  hold        10 s standing hold: joint drift, tilt range, the controller's own tracking RMSE (the baseline
              the agent must not mistake for contact).
  arms        each arm: shoulder pitch -0.3 rad, elbow +0.3 rad, wrist pitch +0.3 rad, one at a time, 1.5 s
              out, 0.5 s hold, 1.5 s back. Per move: reference error, hand displacement commanded vs measured
              (FK of the measured joints, pelvis frame, cm), tilt.
  waist       waist yaw +0.2 rad and back; waist pitch +0.15 rad (a small lean) and back.
  hands       each hand closed (fraction 1) over 1 s, held 1 s, opened: finger joints reached vs the targets.
  pause       a 1.8 s shoulder motion; the operator presses SPACE (pause) as soon as it moves (or --self-disarm writes
              the arming file itself at 1.0 s). Measures the stop latency, the overrun after the stop and
              the hold drift over the next 3 s. Leaves the session disarmed: SPACE arms it again.
Feet on the floor for the rest (the robot must stand on its own: `s` done, gantry slack):
  turn        g1_walk turn +DEG then -DEG (default 30), --repeat N: IMU yaw before/after each, duration, tilt.
  walk        g1_walk forward M (default 1.0) at 0.3 m/s, --repeat N; then asks for the tape measure (m).
  camera      one observation: depth validity, intrinsics, the camera pose the backend assumes, and the
              heights (level frame) of the horizontal surfaces in view. With --pelvis-z (tape: pelvis above
              the floor) the floor's error, with --desk-z (tape: desk above the floor) the desk's error.
"""
import argparse
import json
import math
import sys
import threading
import time
from datetime import datetime, timezone
from pathlib import Path

import numpy as np

PROJECT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(PROJECT))
sys.path.insert(0, str(PROJECT / 'agent_skills/perception/depth-g1sonic/robot-interface/scripts'))
from services.controller.config import Configuration  # noqa: E402
from services.g1sonic import robot as R  # noqa: E402
from services.g1sonic import tools as T  # noqa: E402
from services.g1sonic.backend import G1SonicBackend, MotionStopped  # noqa: E402
from services.g1sonic.bridge import yaw_of  # noqa: E402
from services import arming  # noqa: E402
import g1_model  # noqa: E402

J = {name: i for i, name in enumerate(R.JOINT_NAMES)}
HZ = R.CONTROL_HZ
TAIL = 10


def log(*parts):
    print(time.strftime('%H:%M:%S'), *parts, flush=True)


def wrap_deg(a):
    return (a + 180.0) % 360.0 - 180.0


class Check:
    def __init__(self, root, step, args):
        self.root, self.step, self.args = Path(root), step, args
        self.config = Configuration.load(f'{root}/operator-input.json')
        self.backend = G1SonicBackend(self.config)
        self.model = g1_model.Model(json.loads((Path(g1_model.__file__).parent / 'robot.json').read_text()))
        self.out = self.root / 'runtime' / 'characterization'
        self.record = {'step': step, 'date': datetime.now(timezone.utc).isoformat(timespec='seconds'),
                       'robot': R.ROBOT, 'backend_options': self.config.backend_options}
        # the operator console (scripts/g1_console.py) talks to the backend through operator/stack.json, which the
        # robot service polls; here the backend is in-process, so poll it ourselves while this step runs
        self.polling = True
        threading.Thread(target=self._poll_operator, daemon=True).start()

    def _poll_operator(self):
        while self.polling:
            try:
                self.backend.poll_operator()
            except Exception as exc:
                log('poll_operator:', str(exc)[:120])
            time.sleep(0.2)

    # ----- plumbing ---------------------------------------------------------------------------
    def call(self, name, arguments=None):
        self.backend.validate(name, arguments or {})
        value, _ = self.backend.call(name, arguments or {})
        return value

    def snap(self):
        snap = self.backend.state.latest()
        if snap is None:
            raise RuntimeError('no state from the deploy stack')
        return snap

    def save(self, **fields):
        self.record.update(fields)
        self.out.mkdir(parents=True, exist_ok=True)
        with (self.out / 'g1_motion_check.jsonl').open('a') as f:
            f.write(json.dumps(self.record, default=str) + '\n')
        log(f'saved -> {self.out / "g1_motion_check.jsonl"}')

    def wait_armed(self):
        state = arming.read(self.root)
        if not state['armed']:
            log(f'waiting for the operator to arm {self.root.name}: feet carrying the robot (straps slack), then console `s` (stand), then SPACE (arm) ...')
            deadline = time.time() + 300
            while not arming.read(self.root)['armed']:
                if time.time() > deadline:
                    raise SystemExit('not armed within 5 minutes; nothing moved')
                time.sleep(0.5)
        # a stack already in control publishes state: let the first message arrive, so the hold takes the robot over
        # where it is instead of sending a fresh start (a switch through planner mode, where the robot may step)
        deadline = time.time() + 2.0
        while self.backend.state.latest() is None and time.time() < deadline:
            time.sleep(0.05)
        log('armed; first hold (takes over a running stack; starts control if the console did not)')
        value = self.call(T.HOLD)
        if value.get('start_error'):
            raise SystemExit(f"start failed: {value['start_error']}")
        if self.backend.control_mode != 'streamed':
            log(f'control mode is {self.backend.control_mode}: one no-op row to hand the hold over')
            self.go(self.base_row(), secs=0.5)
        time.sleep(0.5)

    # ----- rows -------------------------------------------------------------------------------
    def base_row(self):
        """The row the backend holds now (its last row), else the measured joints at the anchored heading."""
        last = self.backend.last_row
        row = np.zeros(R.ACTION_DIM)
        row[2] = 0.74
        if last is not None:
            row[:] = last
            return row
        snap = self.snap()
        anchor = self.backend.converter.anchor
        yaw = 0.0 if anchor is None or np.ndim(anchor) == 0 else yaw_of(snap.base_quat) - yaw_of(anchor)
        row[R.ROT6D] = R.rot6d_from_yaw(yaw)
        row[R.JOINTS] = self.backend._within_limits(snap.q)  # a joint resting past its limit (hanging) is pulled inside it
        fractions = self.backend._command_report().get('hands_fraction')
        if fractions:
            row[R.HANDS] = fractions
        return row

    def ramp(self, start, goal, secs, tail=TAIL):
        n = max(2, int(round(secs * HZ)))
        rows = [start + (goal - start) * (i / (n - 1)) for i in range(n)]
        rows += [goal.copy() for _ in range(tail)]
        return rows

    def go(self, goal, secs=1.5, start=None):
        """Stream a ramp from the held row to GOAL, return the reply's feedback and the measured joints."""
        start = self.base_row() if start is None else start
        rows = self.ramp(start, goal, secs)
        t0 = time.time()
        value = self.call(T.STEP, {'actions': [r.tolist() for r in rows]})
        fb = value['transition']['feedback']
        fb = {k: (round(v, 4) if isinstance(v, float) else v) for k, v in fb.items()}
        fb['call_seconds'] = round(time.time() - t0, 2)
        return fb, np.asarray(value['proprio']['joint_positions_rad'], float)

    def hand_center(self, q, side):
        pose = self.model.fk(joints29=list(map(float, q)))
        return np.asarray(pose.point(f'{side}_hand_center', frame='pelvis')['position_m'], float)

    # ----- steps ------------------------------------------------------------------------------
    def status(self):
        value = self.backend.status()
        keep = {k: value.get(k) for k in ('motion', 'stack', 'tilt_deg', 'fault', 'events', 'cameras', 'depth', 'sensors')}
        keep['proprio_present'] = 'proprio' in value
        print(json.dumps(keep, indent=1, default=str), flush=True)
        self.save(result=keep)

    def disarmed(self):
        if arming.read(self.root)['armed']:
            raise SystemExit('the session is armed: disarm it (SPACE) before this step')
        snap = self.backend.state.latest()
        row = np.zeros(R.ACTION_DIM)
        row[2], row[R.ROT6D] = 0.74, R.IDENTITY_ROT6D
        row[R.JOINTS] = R.DEFAULT_STANDING if snap is None else snap.q
        try:
            self.backend.validate(T.STEP, {'actions': [row.tolist()]})
            print('FAIL: the motion was not refused while disarmed', flush=True)
            self.save(result='FAIL: not refused')
        except PermissionError as exc:
            print('OK refused while disarmed:', exc, flush=True)
            self.save(result='OK', refusal=str(exc))

    def hold(self):
        self.wait_armed()
        secs = 10.0
        log(f'standing hold for {secs:g} s')
        qs, tilts, track = [], [], []
        t0 = time.time()
        while time.time() - t0 < secs:
            s = self.snap()
            qs.append(s.q.copy()); tilts.append(s.tilt_deg())
            if s.last_action is not None:
                track.append(float(np.sqrt(np.mean((s.q - s.last_action) ** 2))))
            time.sleep(0.1)
        qs = np.asarray(qs)
        drift = np.abs(qs - qs[0]).max(axis=0)
        j = int(drift.argmax())
        result = {'seconds': secs, 'tilt_deg_min_max': [round(min(tilts), 2), round(max(tilts), 2)],
                  'joint_drift_max_rad': round(float(drift[j]), 4), 'joint_drift_max_name': R.JOINT_NAMES[j],
                  'joint_tracking_rmse_rad_mean': round(float(np.mean(track)), 4) if track else None}
        print(json.dumps(result), flush=True)
        self.save(result=result)

    def arms(self):
        self.wait_armed()
        moves = [('shoulder_pitch', -0.3), ('elbow', 0.3), ('wrist_pitch', 0.3)]
        results = []
        for side in ('right', 'left'):
            for joint, delta in moves:
                name = f'{side}_{joint}_joint'
                base = self.base_row()
                q0 = self.snap().q.copy()
                goal = base.copy(); goal[R.JOINTS][J[name]] += delta
                fb_out, q_out = self.go(goal, 1.5)
                time.sleep(0.5)
                q_peak = self.snap().q.copy()
                fb_back, q_back = self.go(base, 1.5, start=goal)
                time.sleep(0.3)
                cmd_disp = self.hand_center(goal[R.JOINTS], side) - self.hand_center(base[R.JOINTS], side)
                meas_disp = self.hand_center(q_peak, side) - self.hand_center(q0, side)
                r = {'move': f'{name} {delta:+.2f} rad',
                     'joint_reached_rad': round(float(q_peak[J[name]] - q0[J[name]]), 4),
                     'hand_cmd_cm': np.round(cmd_disp * 100, 1).tolist(), 'hand_meas_cm': np.round(meas_disp * 100, 1).tolist(),
                     'hand_error_cm': round(float(np.linalg.norm(cmd_disp - meas_disp) * 100), 1),
                     'out': fb_out, 'back': fb_back,
                     'back_residual_rad': round(float(np.abs(q_back - q0).max()), 4)}
                print(json.dumps(r), flush=True)
                results.append(r)
        self.save(result=results)

    def waist(self):
        self.wait_armed()
        results = []
        for name, delta in (('waist_yaw_joint', 0.2), ('waist_pitch_joint', 0.15)):
            base = self.base_row()
            q0 = self.snap().q.copy()
            yaw0 = yaw_of(self.snap().base_quat)
            # hanging, the policy can rest the waist near a limit: move into the room that is there (sign flipped if need be)
            margin = self.backend.options['joint_limit_margin_rad']
            room_up = float(self.backend.upper[J[name]] - margin - base[R.JOINTS][J[name]])
            room_down = float(base[R.JOINTS][J[name]] - self.backend.lower[J[name]] - margin)
            if delta > room_up:
                delta = room_up if room_up >= 0.05 else -min(delta, room_down)
                log(f'{name}: only {room_up:.2f} rad of room towards the upper limit; moving {delta:+.2f} rad instead')
            goal = base.copy(); goal[R.JOINTS][J[name]] += delta
            fb_out, _ = self.go(goal, 1.5)
            time.sleep(0.5)
            s = self.snap()
            fb_back, q_back = self.go(base, 1.5, start=goal)
            time.sleep(0.3)
            r = {'move': f'{name} {delta:+.2f} rad', 'joint_reached_rad': round(float(s.q[J[name]] - q0[J[name]]), 4),
                 'tilt_deg_at_peak': round(s.tilt_deg(), 2),
                 'heading_change_deg': round(wrap_deg(math.degrees(yaw_of(s.base_quat) - yaw0)), 1),
                 'out': fb_out, 'back': fb_back, 'back_residual_rad': round(float(np.abs(q_back - q0).max()), 4)}
            print(json.dumps(r), flush=True)
            results.append(r)
        self.save(result=results)

    def crouch(self):
        """Crouch deeper and deeper (g1_crouch), standing back up between levels; stops at the first level that tilts."""
        self.wait_armed()
        results = []
        for drop in self.args.drops:
            s0 = self.snap()
            log(f'crouch {drop:.2f} m')
            down = self.call(T.CROUCH, {'pelvis_drop_m': drop, 'secs': 3.0})
            time.sleep(2.0)
            s = self.snap()
            held_height = self.backend.pelvis_height_estimate(s.q, s.base_quat)
            up = self.call(T.CROUCH, {'pelvis_drop_m': 0.0, 'secs': 3.0})
            time.sleep(1.0)
            s1 = self.snap()
            r = {'drop_m': drop, 'knee_flexion_rad': down['transition']['crouch']['knee_flexion_rad'],
                 'pelvis_height_est_m': {'standing': round(self.backend.pelvis_height_estimate(s0.q, s0.base_quat), 3),
                                         'held': round(held_height, 3),
                                         'after_rise': round(self.backend.pelvis_height_estimate(s1.q, s1.base_quat), 3)},
                 'tilt_deg_held': round(s.tilt_deg(), 2), 'tilt_deg_after_rise': round(s1.tilt_deg(), 2),
                 'down': down['transition']['feedback'], 'up': up['transition']['feedback'],
                 'rise_residual_rad': round(float(np.abs(s1.q - s0.q).max()), 4)}
            print(json.dumps(r), flush=True)
            results.append(r)
            if r['tilt_deg_held'] > 15 or r['tilt_deg_after_rise'] > 15:
                log('tilt over 15 deg: no deeper level')
                break
        self.save(result=results)

    def hands(self):
        self.wait_armed()
        results = []
        for side, index in (('right', 39), ('left', 38)):
            base = self.base_row()
            goal = base.copy(); goal[index] = 1.0
            fb_close, _ = self.go(goal, 1.0)
            time.sleep(1.0)
            s = self.snap()
            measured = (s.right_hand_q if side == 'right' else s.left_hand_q).copy()
            target = np.asarray(self.backend.converter.hand(side, 1.0), float)
            fb_open, _ = self.go(base, 1.0, start=goal)
            time.sleep(1.0)
            s2 = self.snap()
            opened = (s2.right_hand_q if side == 'right' else s2.left_hand_q).copy()
            open_target = np.asarray(self.backend.converter.hand(side, 0.0), float)
            r = {'hand': side, 'closed_target_rad': np.round(target, 3).tolist(), 'closed_measured_rad': np.round(measured, 3).tolist(),
                 'closed_max_error_rad': round(float(np.abs(measured - target).max()), 3),
                 'open_max_error_rad': round(float(np.abs(opened - open_target).max()), 3),
                 'close': fb_close, 'open': fb_open}
            print(json.dumps(r), flush=True)
            results.append(r)
        self.save(result=results)

    def pause(self):
        self.wait_armed()
        name = 'right_shoulder_pitch_joint'
        base = self.base_row()
        goal = base.copy(); goal[R.JOINTS][J[name]] -= 0.5
        rows = self.ramp(base, goal, 1.8)   # 90 rows + the tail: one call holds at most 100
        samples = []   # (t, q[name], armed)
        stop_flag = threading.Event()

        def sampler():
            while not stop_flag.is_set():
                s = self.backend.state.latest()
                if s is not None:
                    samples.append((time.time(), float(s.q[J[name]]), arming.read(self.root)['armed']))
                time.sleep(0.02)

        def self_disarm():
            time.sleep(1.0)
            arming.write(self.root, False, by='g1_motion_check', reason='pause test')
            log('arming file written: disarmed')

        threads = [threading.Thread(target=sampler, daemon=True)]
        if self.args.self_disarm:
            threads.append(threading.Thread(target=self_disarm, daemon=True))
        else:
            log('>>> press SPACE on the console about 1 s after the arm starts moving <<<')
        for t in threads:
            t.start()
        t0 = time.time()
        stopped = None
        try:
            self.call(T.STEP, {'actions': [r.tolist() for r in rows]})
            log('the motion finished without a pause: nothing measured; run again')
        except MotionStopped as exc:
            stopped = time.time()
            log('MotionStopped:', str(exc)[:120])
        time.sleep(3.0)
        stop_flag.set()
        if stopped is None:
            self.save(result='no pause happened')
            return
        disarm_t = next((t for t, _, armed in samples if not armed), None)
        at_disarm = next((q for t, q, _ in samples if disarm_t is not None and t >= disarm_t), None)
        after = [(t, q) for t, q, _ in samples if t >= stopped]
        rest = [q for t, q in after if t >= stopped + 0.5]
        result = {'joint': name, 'stop_reply_after_disarm_s': None if disarm_t is None else round(stopped - disarm_t, 3),
                  'motion_started_before_disarm_s': None if disarm_t is None else round(disarm_t - t0, 2),
                  'overrun_rad': None if at_disarm is None or not rest else round(float(rest[0] - at_disarm), 4),
                  'hold_drift_0.5_to_3s_rad': round(float(max(rest) - min(rest)), 4) if rest else None,
                  'tilt_deg_after': round(self.snap().tilt_deg(), 2),
                  'armed_now': arming.read(self.root)['armed']}
        print(json.dumps(result), flush=True)
        log('the session is disarmed: SPACE arms it again before the next step')
        self.save(result=result)

    def turn(self):
        self.wait_armed()
        deg = self.args.turn
        results = []
        for i in range(self.args.repeat):
            for sign in (1, -1):
                yaw0 = yaw_of(self.snap().base_quat)
                t0 = time.time()
                value = self.call(T.WALK, {'forward_m': 0.0, 'turn_deg': sign * deg, 'speed_mps': 0.3})
                fb = value['transition']['feedback']
                time.sleep(1.0)
                yaw1 = yaw_of(self.snap().base_quat)
                r = {'commanded_deg': sign * deg, 'measured_deg_imu': round(wrap_deg(math.degrees(yaw1 - yaw0)), 1),
                     'backend_measured_turn_deg': fb.get('measured_turn_deg'), 'max_tilt_deg': fb.get('max_tilt_deg'),
                     'seconds': round(time.time() - t0, 1), 'tilt_deg_after': round(self.snap().tilt_deg(), 2)}
                print(json.dumps(r), flush=True)
                results.append(r)
        self.save(result=results)

    def walk(self):
        self.wait_armed()
        results = []
        for i in range(self.args.repeat):
            if not self.args.no_prompt:
                input(f'walk {i + 1}/{self.args.repeat}: mark the toes on the floor, clear {self.args.forward + 0.5:.1f} m ahead, ENTER ... ')
            yaw0 = yaw_of(self.snap().base_quat)
            t0 = time.time()
            value = self.call(T.WALK, {'forward_m': self.args.forward, 'turn_deg': 0.0, 'speed_mps': 0.3})
            fb = value['transition']['feedback']
            time.sleep(1.0)
            r = {'commanded_m': self.args.forward, 'planned_advance_m': fb.get('planned_advance_m'),
                 'max_tilt_deg': fb.get('max_tilt_deg'), 'seconds': round(time.time() - t0, 1),
                 'heading_drift_deg': round(wrap_deg(math.degrees(yaw_of(self.snap().base_quat) - yaw0)), 1),
                 'tape_m': None}
            if not self.args.no_prompt:
                text = input('tape measure, toes to toes (m), ENTER to skip: ').strip()
                if text:
                    r['tape_m'] = float(text)
                    r['ratio_measured_over_commanded'] = round(r['tape_m'] / self.args.forward, 2)
            print(json.dumps(r), flush=True)
            results.append(r)
        self.save(result=results)

    def camera(self):
        self.wait_armed()
        value = self.call(T.OBSERVE)
        cam = value['camera']
        if 'frame_id' not in cam:   # no camera in this session: say why instead of a KeyError on frame_id
            result = {'camera': cam.get('error', cam), 'tilt_deg': value.get('tilt_deg')}
            log('no camera frame:', str(result['camera'])[:200])
            print(json.dumps(result, indent=1, default=str), flush=True)
            self.save(result=result)
            return
        frame_dir = Path(self.backend.frames_root) / cam['frame_id']
        result = {'frame_dir': str(frame_dir), 'depth': value.get('depth'), 'tilt_deg': value['tilt_deg']}
        if 'camera_intrinsics' not in value:
            result['geometry'] = 'none: the observation profile is rgb-only, so pixel_to_position is unavailable'
            print(json.dumps(result, indent=1, default=str), flush=True)
            self.save(result=result)
            return
        K = np.asarray(value['camera_intrinsics']['cam_front']['matrix'], float)
        pose = value['camera_poses']['cam_front']
        result.update({'intrinsics_fx_fy_cx_cy': [round(K[0, 0], 1), round(K[1, 1], 1), round(K[0, 2], 1), round(K[1, 2], 1)],
                       'camera_pose_pelvis': pose})
        depth_path = frame_dir / 'cam_front_depth.png'
        if depth_path.exists():
            import cv2
            depth = cv2.imread(str(depth_path), cv2.IMREAD_UNCHANGED).astype(np.float32) / 1000.0
            valid = np.isfinite(depth) & (depth > 0.2) & (depth < 4.0)
            result['depth_valid_fraction'] = round(float(valid.mean()), 3)
            v, u = np.nonzero(valid)
            z = depth[v, u]
            x_cv, y_cv = (u - K[0, 2]) / K[0, 0] * z, (v - K[1, 2]) / K[1, 1] * z
            pts_cam = np.stack([z, -x_cv, -y_cv], axis=1)           # camera X forward, Y left, Z up
            Rc = g1_model._quat_matrix(pose['quaternion_wxyz'])
            pts_pelvis = pts_cam @ np.asarray(Rc, float).T + np.asarray(pose['position_m'], float)
            w, x, y, zq = np.asarray(value['proprio']['root_orientation_world_wxyz'], float)
            Rw = np.asarray(g1_model._quat_matrix([w, x, y, zq]), float)
            heading = math.atan2(2 * (w * zq + x * y), 1 - 2 * (y * y + zq * zq))
            c, s = math.cos(-heading), math.sin(-heading)
            Rh = np.array([[c, -s, 0], [s, c, 0], [0, 0, 1]])
            level = pts_pelvis @ Rw.T @ Rh.T
            # horizontal surfaces: the peaks of the z histogram (1 cm bins) among points 0.3..3 m ahead
            sel = (level[:, 0] > 0.3) & (level[:, 0] < 3.0)
            zs = level[sel, 2]
            hist, edges = np.histogram(zs, bins=np.arange(-1.2, 0.6, 0.01))
            peaks = [(float(edges[i] + 0.005), int(hist[i])) for i in np.argsort(hist)[::-1][:6] if hist[i] > 0.01 * len(zs)]
            peaks.sort(key=lambda p: -p[1])
            result['surface_z_level_m_top_peaks'] = [[round(z, 3), n] for z, n in peaks]
            if peaks:
                floor_z = min(z for z, _ in peaks)
                result['floor_z_level_m'] = round(floor_z, 3)
                if self.args.pelvis_z:
                    result['floor_error_cm'] = round((floor_z + self.args.pelvis_z) * 100, 1)
                if self.args.desk_z and self.args.pelvis_z:
                    want = self.args.desk_z - self.args.pelvis_z
                    nearest = min((z for z, _ in peaks), key=lambda z: abs(z - want))
                    result['desk_z_level_m'] = round(nearest, 3)
                    result['desk_error_cm'] = round((nearest - want) * 100, 1)
        else:
            result['depth'] = 'no depth image (rgb-only profile?)'
        print(json.dumps(result, indent=1, default=str), flush=True)
        self.save(result=result)


STEPS = ('status', 'disarmed', 'hold', 'arms', 'waist', 'hands', 'pause', 'turn', 'walk', 'crouch', 'camera', 'report')


def report(root):
    """The latest record of every step as a Markdown table (for G1.md). Touches no robot."""
    path = Path(root) / 'runtime' / 'characterization' / 'g1_motion_check.jsonl'
    if not path.exists():
        raise SystemExit(f'no results yet: {path}')
    latest = {}
    for line in path.read_text().splitlines():
        if line.strip():
            rec = json.loads(line)
            latest[rec['step']] = rec
    rows = []
    for step in STEPS[:-1]:
        rec = latest.get(step)
        if rec is None:
            rows.append((step, 'not run', ''))
            continue
        r = rec.get('result')
        date = rec['date'][:10]
        if step == 'status':
            rows.append((step, 'stack ' + ('linked' if (r or {}).get('stack', {}).get('state_messages') else 'no state yet'), date))
        elif step == 'disarmed':
            rows.append((step, str(r)[:60], date))
        elif step == 'hold' and isinstance(r, dict):
            rows.append((step, f"drift {r['joint_drift_max_rad']} rad ({r['joint_drift_max_name']}), tilt {r['tilt_deg_min_max']} deg, "
                               f"tracking RMSE {r['joint_tracking_rmse_rad_mean']} rad", date))
        elif step == 'arms' and isinstance(r, list):
            for m in r:
                rows.append((f"arms: {m['move']}", f"joint reached {m['joint_reached_rad']:+.3f} rad, hand error {m['hand_error_cm']} cm, "
                                                   f"ref RMSE {m['out'].get('reference_error_rmse_rad')} rad, tilt {m['out'].get('tilt_deg')}", date))
        elif step == 'waist' and isinstance(r, list):
            for m in r:
                rows.append((f"waist: {m['move']}", f"joint reached {m['joint_reached_rad']:+.3f} rad, tilt at peak {m['tilt_deg_at_peak']} deg, "
                                                    f"heading change {m['heading_change_deg']} deg", date))
        elif step == 'hands' and isinstance(r, list):
            for m in r:
                rows.append((f"hands: {m['hand']}", f"closed max error {m['closed_max_error_rad']} rad, open max error {m['open_max_error_rad']} rad", date))
        elif step == 'pause' and isinstance(r, dict):
            rows.append((step, f"reply {r['stop_reply_after_disarm_s']} s after the disarm, overrun {r['overrun_rad']} rad, "
                               f"hold drift {r['hold_drift_0.5_to_3s_rad']} rad over 3 s", date))
        elif step == 'turn' and isinstance(r, list):
            for m in r:
                rows.append((f"turn {m['commanded_deg']:+.0f} deg", f"IMU {m['measured_deg_imu']} deg, {m['seconds']} s, max tilt {m['max_tilt_deg']} deg", date))
        elif step == 'walk' and isinstance(r, list):
            for m in r:
                tape = f"tape {m['tape_m']} m ({m.get('ratio_measured_over_commanded')} x)" if m.get('tape_m') else 'no tape value'
                rows.append((f"walk {m['commanded_m']} m", f"{tape}, planner advance {m['planned_advance_m']} m, {m['seconds']} s, max tilt {m['max_tilt_deg']} deg, "
                                                           f"heading drift {m['heading_drift_deg']} deg", date))
        elif step == 'crouch' and isinstance(r, list):
            for m in r:
                h = m['pelvis_height_est_m']
                rows.append((f"crouch {m['drop_m']} m", f"knees +{m['knee_flexion_rad']} rad, pelvis est {h['standing']} -> {h['held']} -> {h['after_rise']} m, "
                                                        f"tilt held {m['tilt_deg_held']} / after rise {m['tilt_deg_after_rise']} deg, "
                                                        f"ref RMSE {m['down'].get('reference_error_rmse_rad')} rad", date))
        elif step == 'camera' and isinstance(r, dict):
            rows.append((step, f"depth valid {r.get('depth_valid_fraction')}, floor error {r.get('floor_error_cm', 'n/a')} cm, "
                               f"desk error {r.get('desk_error_cm', 'n/a')} cm, fx {r.get('intrinsics_fx_fy_cx_cy', ['?'])[0]}", date))
        else:
            rows.append((step, str(r)[:80], date))
    print('| Step | Measured | Date |')
    print('|---|---|---|')
    for name, text, date in rows:
        print(f'| {name} | {text} | {date} |')


def main():
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument('root')
    parser.add_argument('step', choices=STEPS)
    parser.add_argument('--repeat', type=int, default=1)
    parser.add_argument('--forward', type=float, default=1.0, help='walk distance (m)')
    parser.add_argument('--turn', type=float, default=30.0, help='turn angle (deg)')
    parser.add_argument('--drops', type=float, nargs='+', default=[0.1, 0.2, 0.3], help='crouch: pelvis drops to try (m)')
    parser.add_argument('--pelvis-z', type=float, default=None, help='tape: pelvis height above the floor (m)')
    parser.add_argument('--desk-z', type=float, default=None, help='tape: desk top above the floor (m)')
    parser.add_argument('--self-disarm', action='store_true', help='pause: write the arming file at 1.0 s instead of waiting for SPACE')
    parser.add_argument('--no-prompt', action='store_true', help='walk: no tape-measure prompts (unattended / sim)')
    args = parser.parse_args()
    if args.step == 'report':
        report(args.root)
        return
    check = Check(args.root, args.step, args)
    try:
        getattr(check, args.step)()
    except MotionStopped as exc:
        print(f'STOPPED: {exc}', flush=True)
        print('robot_state', json.dumps(getattr(exc, 'robot_state', None), default=str), flush=True)
    except Exception as exc:
        print(f'FAILED: {type(exc).__name__}: {exc}', flush=True)
        raise
    finally:
        check.polling = False
        check.backend.close()
        log('closed (the robot keeps holding its last reference; the stack stays up)')


if __name__ == '__main__':
    main()
