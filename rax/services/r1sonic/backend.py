"""MCP backend for the Unitree R1 with PiPER parallel grippers under SONIC_R1 in teleop mode (encoder mode 1).

The robot side is an "r1 stack" (services/r1sonic/stack.py): the MuJoCo sim server (r1/sim/r1_sim_server.py) or the
real-robot bridge. Every 50 Hz tick it gives the frozen SONIC_R1 policy the default standing lower body and three point
targets (left wrist, right wrist, torso, in the robot's reference frame) and holds the head at a commanded pitch; the
PiPER grippers are position-commanded beside it. This backend turns grasp-point goals into those targets: inverse
kinematics on the 5-joint arm (the policy tracks a pose some arm configuration has; a pose none has makes it lean and
step), joint-space interpolation, then a bounded servo on the measured joints.

Safety layering, outermost first: the operator at the robot; operator arming (services.arming; motion tools refuse
unless armed); this backend (joint limits minus a margin, reachability before motion, a bounded servo, a stop on
stepping, a fall watch that holds and disarms); the stack's own limits.
"""
from __future__ import annotations

import base64
import io
import json
import threading
import time
import uuid
from pathlib import Path

import numpy as np
from scipy.spatial.transform import Rotation

from services import arming
from . import tools as T
from .kin import Kinematics
from .stack import StackClient, StackError
from .tools import RejectedBeforeAction

WORKSPACE_NAME = 'agent-workspace'
MODEL = Path(__file__).with_name('model') / 'r1_piper_kin.xml'
ROBOT = 'r1sonic'
CAMERA = 'cam_head'
# SONIC_R1 100K default standing pose (r1_deployment_contract.json) for the joints this backend commands
DEFAULT_UPPER = {'left_shoulder_pitch_joint': 0.35, 'left_shoulder_roll_joint': 0.18, 'left_shoulder_yaw_joint': 0.0,
                 'left_elbow_joint': 0.87, 'left_wrist_roll_joint': 0.0,
                 'right_shoulder_pitch_joint': 0.35, 'right_shoulder_roll_joint': -0.18, 'right_shoulder_yaw_joint': 0.0,
                 'right_elbow_joint': 0.87, 'right_wrist_roll_joint': 0.0, 'waist_roll_joint': 0.0, 'waist_yaw_joint': 0.0}
DEFAULTS = {
    'stack_host': '127.0.0.1', 'stack_port': 5591, 'stack_timeout_s': 3.0,
    'control_hz': 50.0,
    'joint_limit_margin_rad': 0.05,
    'max_ik_error_m': 0.03,           # r1_move_hand refuses a goal the best IK solution misses by more
    'servo_iterations': 4, 'servo_gain': 0.8, 'servo_tolerance_m': 0.006,
    'max_servo_correction_m': 0.05,   # total correction the servo may add to a target (an unbounded servo chased a robot
                                      # that stepped away and walked it 1.2 m sideways until it fell, sim 2026-11-04)
    'step_lift_m': 0.025,             # a foot this much higher than the other (leg kinematics, level frame) is a step
    'max_steps_per_motion': 2,        # more steps than this during one motion: the motion stops and holds
    'fall_tilt_deg': 30.0,            # above this the session holds and disarms (fault 'fall')
    'runaway_steps': 6, 'runaway_window_s': 6.0,   # between calls: this many steps in the window disarms (fault 'runaway')
    'head_pitch': 0.5,                # the head pitch held from the start (looking down at a table in front)
    'agent_image_width': 640,
}


def _rot(q_wxyz):
    return Rotation.from_quat(np.asarray(q_wxyz, dtype=float)[[1, 2, 3, 0]])


def _wxyz(r):
    return r.as_quat()[[3, 0, 1, 2]]


class MotionStopped(RuntimeError):
    handled_stop = True


class R1SonicBackend:
    mutating = T.MUTATING

    def __init__(self, config, *, stack=None, clock=time.monotonic, wall=time.time):
        self.config = config
        unknown = set(config.backend_options) - set(DEFAULTS)
        if unknown:
            raise ValueError(f'unknown r1sonic backend options: {sorted(unknown)}')
        self.options = o = {**DEFAULTS, **config.backend_options}
        self.clock, self.wall = clock, wall
        self.workspace = config.root / WORKSPACE_NAME
        self.frames_root = self.workspace / 'runtime' / 'frames'
        self.depth = config.observation_profile == 'rgbd'
        self.kin = Kinematics(MODEL)
        self.watch_kin = Kinematics(MODEL)     # the fall watch thread's own copy (Kinematics is not thread-safe)
        self.stack = stack if stack is not None else StackClient(o['stack_host'], o['stack_port'], o['stack_timeout_s'])
        self.cmd = dict(DEFAULT_UPPER)        # commanded upper-body joints (the targets are their FK)
        self.correction = {'left': np.zeros(3), 'right': np.zeros(3)}   # servo offsets on the wrist points (pelvis frame)
        self.grip = {'left': 0.07, 'right': 0.07}
        self.head_pitch = o['head_pitch']
        self.step_id = 0
        self.last_feedback = None
        self._raw = None
        self.events = []
        self.fault = None
        self.fault_disarmed_at = None
        self.lock = threading.Lock()
        self._motion = False
        self._watching = True
        self._watch = threading.Thread(target=self._watch_falls, name='r1sonic-fall-watch', daemon=True)
        self._watch.start()

    # ----- tools ----------------------------------------------------------------------------------------------------
    def tools(self):
        return T.tool_definitions(geometry=self.depth)

    def validate(self, name, arguments):
        T.validate_request(name, arguments)
        if name in T.MUTATING and not (name == T.MOVE_HAND and arguments.get('preview_only')):
            self._require_armed()
        if name == T.PIXEL_TO_POSITION:
            if self._raw is None:
                raise RejectedBeforeAction('observe first; there is no current observation')
            if self._raw[0] != arguments['observation_step_id'] or self._raw[0] != self.step_id:
                raise RejectedBeforeAction(f"stale pixel selection: observation_step_id={arguments['observation_step_id']}, "
                                           f'current step_id={self.step_id}; observe again and reselect')

    def call(self, name, arguments):
        self._acknowledge_fault()
        if name == T.STATUS:
            return self.status(), []
        if name == T.OBSERVE:
            return self._observation()
        if name == T.PIXEL_TO_POSITION:
            return self._pixel_to_position(arguments), []
        if name == T.HOLD:
            self.stack.hold()
            return {'held': True, 'command': self._command_report()}, []
        with self.lock:
            self._motion = True
            try:
                if name == T.MOVE_HAND:
                    return self._move_hand(**arguments), []
                if name == T.MOVE_JOINTS:
                    return self._move_joints(arguments['joints'], arguments.get('secs', 2.0)), []
                if name == T.GRIPPER:
                    return self._gripper(arguments['arm'], arguments['width_m'], arguments.get('secs', 1.0)), []
                if name == T.HEAD:
                    return self._head(arguments['pitch'], arguments.get('secs', 1.0)), []
                if name == T.WALK:
                    return self._walk(arguments.get('forward_m', 0.0), arguments.get('turn_deg', 0.0)), []
                if name == T.GRASP_PAW:
                    return self._grasp_paw(), []
                if name == T.PLACE_IN_BOX:
                    return self._place_in_box(arguments.get('side', 'auto')), []
            finally:
                self._motion = False
                self.step_id += 1
        raise ValueError(f'unknown tool {name}')

    def _walk(self, forward_m, turn_deg=0.0):
        """Walk on the legs via the stack's gait; the arms hold their last command. Returns the measured outcome."""
        reply = self.stack.walk(float(forward_m), float(turn_deg))
        out = {k: reply.get(k) for k in ('walked_m', 'turn_deg', 'pelvis', 'tilt_deg', 'fallen') if k in reply}
        return out

    def _grasp_paw(self):
        """Grasp the nearest hind paw with the side-appropriate arm and partially lift it (stack primitive, blocks)."""
        reply = self.stack.grasp_paw()
        side = reply.get('side')
        if side in self.grip:   # the primitive commands the gripper itself: keep the stored command in step, or the next
            self.grip[side] = 0.0 if reply.get('held') else 0.07   # motion row re-opens the hand (RAX session 2 dropped a carried puppy)
        return {k: reply.get(k) for k in ('paw', 'side', 'held', 'clamped_width_m', 'lifted_m', 'tilt_deg', 'fallen') if k in reply}

    def _place_in_box(self, side='auto'):
        """With a paw held, lower over the box interior and release so the puppy drops in (stack primitive, blocks)."""
        reply = self.stack.place_in_box(side)
        self.grip = {a: 0.07 for a in self.grip}   # the primitive releases: stored command back to open
        return {k: reply.get(k) for k in ('in_box', 'dog', 'box', 'tilt_deg', 'fallen') if k in reply}

    # ----- state ----------------------------------------------------------------------------------------------------
    def _state(self):
        try:
            s = self.stack.state()
        except StackError as exc:
            raise RuntimeError(str(exc)) from None
        s['qd'] = dict(zip(s['joint_names'], s['q']))
        return s

    def _require_armed(self):
        state = arming.read(self.config.root)
        if not state['armed']:
            raise PermissionError(f"motion is not armed ({state['reason']}); the operator arms motion")
        if not self._acknowledge_fault(state):
            raise PermissionError('the backend disarmed the session for a fault; the operator arms again on purpose')
        s = self._state()
        if s.get('fallen'):
            raise PermissionError('the robot is down; ask the operator (request_operator kind help)')
        return s

    def _acknowledge_fault(self, state=None):
        state = state or arming.read(self.config.root)
        if self.fault_disarmed_at is None:
            return True
        if not state['armed'] or (state.get('since') or 0) <= self.fault_disarmed_at:
            return False
        self._event('fault acknowledged by re-arming: ' + self.fault['what'])
        self.fault, self.fault_disarmed_at = None, None
        return True

    def _event(self, what, **details):
        self.events = (self.events + [{'at': round(self.wall(), 2), 'what': what, **details}])[-20:]

    def _disarm(self, reason, fault=None):
        now = self.wall()
        if fault is not None:
            self.fault = {'at': round(now, 2), 'kind': fault, 'what': reason}
            self.fault_disarmed_at = now
        self._event('disarmed: ' + reason)
        if arming.read(self.config.root)['armed']:
            arming.write(self.config.root, False, by='r1sonic-backend', reason=reason)

    @staticmethod
    def _tilt(quat):
        return float(np.degrees(np.arccos(np.clip(_rot(quat).as_matrix()[2, 2], -1, 1))))

    def _feet_lift(self, s, kin=None):
        """Height of the higher foot above the lower one, in the level frame (leg kinematics + IMU tilt)."""
        kin = kin or self.kin
        kin.set(s['qd'])
        R = _rot(s['base_quat_wxyz']).as_matrix()
        z = [float((R @ kin.body(f'{side}_ankle_roll_link')[0])[2]) for side in ('left', 'right')]
        return abs(z[0] - z[1])

    def _watch_falls(self):
        history = []
        while self._watching:
            time.sleep(0.1)
            try:
                s = self.stack.state()
            except Exception:
                continue
            s['qd'] = dict(zip(s['joint_names'], s['q']))
            tilt = s.get('tilt_deg', self._tilt(s['base_quat_wxyz']))
            if self.fault is None and (s.get('fallen') or tilt > self.options['fall_tilt_deg']):
                try:
                    self.stack.hold()
                except Exception:
                    pass
                self._disarm(f'tilt {tilt:.0f} deg: the robot is falling or down', fault='fall')
                continue
            if self._motion or self.fault is not None:
                history = []
                continue
            now = self.clock()
            lifted = self._feet_lift(s, self.watch_kin) > self.options['step_lift_m']
            history = [h for h in history if now - h[0] <= self.options['runaway_window_s']] + [(now, lifted)]
            steps = sum(1 for a, b in zip(history, history[1:]) if b[1] and not a[1])
            if steps >= self.options['runaway_steps']:
                self._disarm(f'the robot stepped {steps} times in {self.options["runaway_window_s"]:g} s by itself: '
                             'operator, check its footing and the space around it, then arm again', fault='runaway')
                history = []

    # ----- targets --------------------------------------------------------------------------------------------------
    def _full(self, upper):
        q = dict(upper)
        q['head_pitch_joint'] = self.head_pitch
        return q

    def _row(self, upper, base_quat, grip=None, head=None, correction=None):
        """A stream row from upper-body joint angles: the FK of the three points (pelvis frame), plus the servo
        corrections, in the reference frame (the measured base orientation relative to the reference heading)."""
        pts = self.kin.points(self._full(upper))
        Rrel = _rot(base_quat)
        corr = correction or self.correction
        row = np.zeros(24)
        for k, (p, q) in enumerate(pts):
            if k < 2:
                p = p + corr[T.ARMS[k]]
            row[7 * k:7 * k + 3] = Rrel.apply(p)
            row[7 * k + 3:7 * k + 7] = _wxyz(Rrel * _rot(q))
        g = grip or self.grip
        row[21], row[22] = g['left'], g['right']
        row[23] = self.head_pitch if head is None else head
        return row

    def _limits(self, q):
        out = {}
        m = self.options['joint_limit_margin_rad']
        for n, v in q.items():
            out[n] = float(np.clip(v, self.kin.lower[n] + m, self.kin.upper[n] - m))
        return out

    def _execute(self, upper_to, secs, s0, grip_to=None, head_to=None, correction_to=None):
        """Stream a smooth joint-space interpolation from the commanded pose to upper_to, then wait until it ran,
        watching tilt and stepping. Returns the trace summary."""
        n = max(1, int(round(secs * self.options['control_hz'])))
        start = dict(self.cmd); g0 = dict(self.grip); h0 = self.head_pitch
        c0 = {a: self.correction[a].copy() for a in T.ARMS}
        c1 = correction_to or c0
        g1 = grip_to or g0; h1 = h0 if head_to is None else head_to
        rows = []
        for k in range(n):
            u = (k + 1) / n; u = u * u * (3 - 2 * u)
            q = {j: start[j] + u * (upper_to.get(j, start[j]) - start[j]) for j in start}
            grip = {a: g0[a] + u * (g1[a] - g0[a]) for a in T.ARMS}
            corr = {a: c0[a] + u * (c1[a] - c0[a]) for a in T.ARMS}
            rows.append(self._row(q, s0['base_quat_wxyz'], grip, h0 + u * (h1 - h0), corr))
        self.stack.stream(rows)
        self.cmd = {j: upper_to.get(j, start[j]) for j in start}
        self.grip, self.head_pitch, self.correction = dict(g1), h1, {a: c1[a].copy() for a in T.ARMS}
        return self._wait(n / self.options['control_hz'])

    def _wait(self, duration):
        o = self.options
        deadline = self.clock() + duration + 3.0
        tilt_max, steps, lifted, lift_max = 0.0, 0, False, 0.0
        time.sleep(min(0.1, duration))
        while True:
            s = self._state()
            tilt = s.get('tilt_deg', self._tilt(s['base_quat_wxyz']))
            tilt_max = max(tilt_max, tilt)
            lift = self._feet_lift(s); lift_max = max(lift_max, lift)
            now_lifted = lift > o['step_lift_m']
            if now_lifted and not lifted:
                steps += 1
            lifted = now_lifted
            if s.get('fallen') or tilt > o['fall_tilt_deg']:
                self.stack.hold()
                self._disarm(f'tilt {tilt:.0f} deg during a motion', fault='fall')
                raise MotionStopped(f'the robot is falling (tilt {tilt:.0f} deg): held and disarmed; ask the operator')
            if not arming.read(self.config.root)['armed']:
                self.stack.hold()
                raise MotionStopped('disarmed during the motion: held where it was')
            if steps > o['max_steps_per_motion']:
                self.stack.hold()
                self._event(f'motion stopped: {steps} steps')
                return {'stopped': f'the robot took {steps} steps during the motion: held. Its pelvis moved, so '
                                   're-observe before the next move; a reach that makes it step is too far or loads it',
                        'tilt_max_deg': round(tilt_max, 1), 'steps': steps, 'foot_lift_max_m': round(lift_max, 3), 'state': s}
            if s['queued'] == 0 or self.clock() > deadline:
                break
            time.sleep(0.1)
        time.sleep(0.3)                 # the controller settles on the last target
        s = self._state()
        return {'tilt_max_deg': round(max(tilt_max, s['tilt_deg']), 1), 'steps': steps, 'foot_lift_max_m': round(lift_max, 3),
                'state': s}

    def _grasp_points(self, qd):
        self.kin.set(qd)
        return {a: np.round(self.kin.site(f'{a}_grasp')[0], 4).tolist() for a in T.ARMS}

    # ----- motions --------------------------------------------------------------------------------------------------
    def _move_hand(self, arm, position, approach=None, fingers=None, use_waist=False, secs=2.0, gripper_width_m=None,
                   servo=True, preview_only=False):
        o = self.options
        s0 = self._state()
        goal = np.asarray(position, dtype=float)
        seed = {**s0['qd'], **self.cmd}
        q, perr, aerr = self.kin.ik(arm, goal, approach, q0=self._full(seed), waist=use_waist, fingers=fingers,
                                    margin=o['joint_limit_margin_rad'])
        q = self._limits(q)
        self.kin.set(self._full({**seed, **q}))
        R = self.kin.site(f'{arm}_grasp')[1]
        plan = {'arm': arm, 'goal_pelvis_m': np.round(goal, 4).tolist(), 'ik_error_m': round(perr, 4),
                'approach_error_deg': round(aerr, 1) if approach is not None else None,
                'approach_achieved': np.round(R[:, 2], 3).tolist(), 'fingers_axis_achieved': np.round(R[:, 1], 3).tolist(),
                'joints': {k: round(v, 4) for k, v in q.items()}}
        if preview_only:
            return {'preview': plan, 'reachable': perr <= o['max_ik_error_m']}
        if perr > o['max_ik_error_m']:
            raise RejectedBeforeAction(f'unreachable: the best arm solution misses the goal by {perr * 100:.1f} cm '
                                       f'(limit {o["max_ik_error_m"] * 100:.0f} cm); nothing moved. Preview: '
                                       + json.dumps(plan))
        grip = dict(self.grip)
        if gripper_width_m is not None:
            grip[arm] = float(gripper_width_m)
        zero = {**self.correction, arm: np.zeros(3)}
        trace = self._execute(q, secs, s0, grip_to=grip, correction_to=zero)
        servo_log = []
        if servo and 'stopped' not in trace:
            for _ in range(o['servo_iterations']):
                s = trace['state']
                err = goal - np.asarray(self._grasp_points(s['qd'])[arm])
                servo_log.append(round(float(np.linalg.norm(err)), 4))
                if np.linalg.norm(err) < o['servo_tolerance_m']:
                    break
                corr = self.correction[arm] + o['servo_gain'] * err
                norm = np.linalg.norm(corr)
                if norm > o['max_servo_correction_m']:
                    corr *= o['max_servo_correction_m'] / norm
                if np.linalg.norm(corr - self.correction[arm]) < 1e-3:
                    break               # at the bound: the rest is not the controller's tracking error
                sub = self._execute({}, 0.4, s, correction_to={**self.correction, arm: corr})
                trace = {**sub, 'tilt_max_deg': max(trace['tilt_max_deg'], sub['tilt_max_deg']),
                         'steps': trace['steps'] + sub['steps']}
                if 'stopped' in sub:
                    break
        return self._report('move_hand', trace, plan=plan, goal=(arm, goal), servo_errors_m=servo_log)

    def _move_joints(self, joints, secs):
        s0 = self._state()
        q = self._limits(joints)
        clipped = {k: round(q[k], 4) for k in joints if abs(q[k] - joints[k]) > 1e-6}
        trace = self._execute(q, secs, s0)
        return self._report('move_joints', trace, clipped_to_limits=clipped or None)

    def _gripper(self, arm, width, secs):
        s0 = self._state()
        trace = self._execute({}, secs, s0, grip_to={**self.grip, arm: float(width)})
        time.sleep(0.3)
        s = self._state()
        measured = s['gripper_width_m'][arm]
        return self._report('gripper', {**trace, 'state': s}, gripper={
            'arm': arm, 'commanded_m': round(float(width), 4), 'measured_m': measured,
            'blocked_m': round(measured - float(width), 4) if measured - float(width) > 0.005 else 0.0})

    def _head(self, pitch, secs):
        s0 = self._state()
        trace = self._execute({}, secs, s0, head_to=float(pitch))
        return self._report('head', trace)

    def _report(self, kind, trace, goal=None, **extra):
        s = trace.pop('state')
        points = self._grasp_points(s['qd'])
        fb = {'motion': kind, **{k: v for k, v in trace.items()}, **{k: v for k, v in extra.items() if v is not None},
              'grasp_points_pelvis_m': points, 'gripper_width_m': s['gripper_width_m'],
              'tilt_deg': s['tilt_deg'], 'servo_correction_m': {a: np.round(self.correction[a], 4).tolist() for a in T.ARMS}}
        if goal is not None:
            arm, g = goal
            fb['grasp_error_m'] = round(float(np.linalg.norm(g - np.asarray(points[arm]))), 4)
        self.last_feedback = fb
        return fb

    # ----- observation ----------------------------------------------------------------------------------------------
    def _camera_pose(self, qd):
        self.kin.set(qd)
        p, R = self.kin.camera()
        return {'frame': 'pelvis', 'position_m': np.round(p, 5).tolist(), 'quaternion_wxyz': np.round(_wxyz(Rotation.from_matrix(R)), 6).tolist(),
                'axes': 'camera X forward (optical axis), Y left, Z up'}

    def _scaled(self, rgb, depth, K):
        import cv2
        width = self.options['agent_image_width']
        scale = width / rgb.shape[1]
        if scale >= 1.0:
            return rgb, depth, np.asarray(K, dtype=float), 1.0
        size = (int(width), int(round(rgb.shape[0] * scale)))
        small = cv2.resize(rgb, size, interpolation=cv2.INTER_AREA)
        small_depth = None if depth is None else cv2.resize(np.where(np.isfinite(depth), depth, 0.0).astype(np.float32), size,
                                                            interpolation=cv2.INTER_NEAREST)
        K = np.asarray(K, dtype=float).copy(); K[0, :] *= size[0] / rgb.shape[1]; K[1, :] *= size[1] / rgb.shape[0]
        return small, small_depth, K, scale

    @staticmethod
    def _jpeg(rgb, quality=85):
        import cv2
        ok, data = cv2.imencode('.jpg', np.ascontiguousarray(rgb[:, :, ::-1]), [cv2.IMWRITE_JPEG_QUALITY, quality])
        if not ok:
            raise RuntimeError('JPEG encoding failed')
        return data.tobytes()

    @staticmethod
    def _depth_png(depth_m):
        from PIL import Image
        depth = np.asarray(depth_m, dtype=np.float32)
        valid = np.isfinite(depth) & (depth > 0)
        mm = np.zeros(depth.shape, dtype=np.uint16)
        mm[valid] = np.rint(np.clip(depth[valid] * 1000.0, 0, 65535)).astype(np.uint16)
        stream = io.BytesIO(); Image.fromarray(mm).save(stream, format='PNG')
        values = depth[valid]
        return stream.getvalue(), {'shape': [int(depth.shape[1]), int(depth.shape[0])], 'encoding': 'uint16_png',
                                   'unit': 'millimeters', 'valid_fraction': round(float(valid.mean()), 4),
                                   'min_m': round(float(values.min()), 3) if values.size else None,
                                   'max_m': round(float(values.max()), 3) if values.size else None}

    def _proprio(self, s):
        return {'joint_names': s['joint_names'], 'joint_positions_rad': np.round(s['q'], 4).tolist(),
                'imu_orientation_wxyz': np.round(s['base_quat_wxyz'], 5).tolist(),
                'gripper_width_m': s['gripper_width_m'], 'grasp_points_pelvis_m': self._grasp_points(s['qd'])}

    def _observation(self):
        content, packet = [], {}
        try:
            meta, rgb, depth = self.stack.camera()
        except StackError as exc:
            raise RuntimeError(str(exc)) from None
        s = self._state()
        names = s['joint_names']
        qd = dict(zip(names, meta['q']))           # the joints when the image was taken
        pose = self._camera_pose(qd)
        small, small_depth, K, scale = self._scaled(rgb, depth if self.depth else None, meta['intrinsics'])
        frame_id = time.strftime('%Y%m%d%H%M%S') + uuid.uuid4().hex[:6]
        directory = self.frames_root / frame_id
        directory.mkdir(parents=True, exist_ok=True)
        (directory / f'{CAMERA}.jpg').write_bytes(self._jpeg(rgb, 95))
        content.append({'type': 'image', 'mimeType': 'image/jpeg', 'data': base64.b64encode(self._jpeg(small)).decode()})
        info = {'frame_id': frame_id, 'preview_scale': round(scale, 4), 'intrinsics_full': meta['intrinsics'],
                'camera_pose_pelvis': pose, 'joints': {n: round(v, 5) for n, v in qd.items()}}
        height, width = small.shape[:2]
        packet['camera'] = {'frame_id': frame_id, 'directory': f'runtime/frames/{frame_id}', 'width': int(width),
                            'height': int(height), 'preview_scale': round(scale, 4)}
        if self.depth:
            png, stats = self._depth_png(small_depth)
            (directory / f'{CAMERA}_depth.png').write_bytes(png)
            content.append({'type': 'image', 'mimeType': 'image/png', 'data': base64.b64encode(png).decode()})
            packet['depth'] = {CAMERA: {**stats, 'file': f'{CAMERA}_depth.png'}}
            info['depth'] = f'{CAMERA}_depth.png (uint16 mm, 0 invalid, at the preview scale)'
        (directory / 'meta.json').write_text(json.dumps(info, indent=1))
        K = np.asarray(K).tolist()
        packet['camera_intrinsics'] = {CAMERA: {'matrix': K, 'width': int(width), 'height': int(height),
                                                'depth': 'distance along the optical axis (m); 0 = invalid'}}
        packet['camera_poses'] = {CAMERA: pose}
        self._raw = (self.step_id, small_depth, packet['camera_intrinsics'][CAMERA], pose, s['base_quat_wxyz'])
        packet.update({'step_id': self.step_id, 'robot': ROBOT, 'task': self.config.task, 'images': [CAMERA],
                       'proprio': self._proprio(s), 'tilt_deg': s['tilt_deg'], 'motion': arming.read(self.config.root)})
        return packet, content

    def _pixel_to_position(self, arguments):
        from services.g1sonic.tools import level_from_pelvis, pixel_to_pelvis
        step, depth, intrinsics, pose, root = self._raw
        if depth is None:
            raise ValueError('the current observation has no depth')
        results = []
        for item in arguments['selections']:
            point = pixel_to_pelvis(depth, intrinsics, pose, float(item['u']), float(item['v']),
                                    int(arguments.get('depth_window_radius', 1)))
            if point['valid']:
                point['position_level_m'] = np.round(level_from_pelvis(point['position_pelvis_m'], root), 4).tolist()
                point['position_pelvis_m'] = np.round(point['position_pelvis_m'], 4).tolist()
            results.append({'u': item['u'], 'v': item['v'], **point})
        return {'observation_step_id': step, 'camera': CAMERA,
                'frames': {'pelvis': 'robot pelvis', 'level': 'origin at the pelvis, z up, x along the measured heading'},
                'selections': results}

    # ----- status ---------------------------------------------------------------------------------------------------
    def _command_report(self):
        return {'upper_joints': {k: round(v, 4) for k, v in self.cmd.items()}, 'gripper_width_m': dict(self.grip),
                'head_pitch': round(self.head_pitch, 3),
                'servo_correction_m': {a: np.round(self.correction[a], 4).tolist() for a in T.ARMS}}

    def status(self):
        o = self.options
        value = {'robot': ROBOT, 'motion': arming.read(self.config.root),
                 'control': 'SONIC_R1 100K teleop mode: three point targets (left wrist, right wrist, torso) from the IK '
                            'of the commanded arm/waist joints; default standing legs; head held at the commanded pitch; '
                            'PiPER grippers position-commanded',
                 'stack': {'endpoint': self.stack.endpoint, 'requests': self.stack.requests, 'failures': self.stack.failures},
                 'commandable_joints': list(T.UPPER),
                 'limits': {k: o[k] for k in ('joint_limit_margin_rad', 'max_ik_error_m', 'max_servo_correction_m',
                                              'max_steps_per_motion', 'fall_tilt_deg')},
                 'joint_limits_rad': {n: [round(self.kin.lower[n], 4), round(self.kin.upper[n], 4)] for n in T.UPPER},
                 'command': self._command_report(), 'step_id': self.step_id, 'last_feedback': self.last_feedback,
                 'fault': self.fault, 'events': self.events[-5:], 'cameras': [CAMERA], 'depth': self.depth}
        try:
            s = self._state()
            value['proprio'] = self._proprio(s)
            value['tilt_deg'] = s['tilt_deg']
            value['stack']['controller'] = s.get('mode')
        except RuntimeError as exc:
            value['stack']['error'] = str(exc)
        return value

    def close(self):
        self._watching = False
        try:
            self.stack.hold()
        except Exception:
            pass
        self.stack.close()
