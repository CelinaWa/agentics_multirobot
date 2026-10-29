"""MCP backend for the real Unitree G1 + Dex3 under NVIDIA's SONIC whole-body controller.

The robot is balanced by the official C++ deploy stack (`gear_sonic_deploy/g1_deploy_onnx_ref`,
started by the operator with `--input-type zmq --output-type zmq` and the joint-reference encoder
configuration), which runs the frozen SONIC encoder and decoder and its own PD loop and writes the
motor commands over DDS. This service never touches DDS: it streams semantic40 reference frames to
the stack's pose socket, 50 per second, and reads the state map the stack publishes every tick.

Safety layering, outermost first:
  1. The physical emergency stop and the operator beside the robot. The stack's own stop
     (keyboard `O`, the gamepad's Select, or a stop command on the wire) damps every joint.
  2. Operator arming (services.arming): the motion tools refuse unless armed, and a running
     motion stops within one tick once the operator disarms. A stopped motion holds: the stack
     keeps tracking the last reference it received, so the robot stays where it was told.
  3. This backend: joint references inside the URDF limits minus a margin, a cap on the step
     between consecutive references and between the measured pose and a call's first row, a
     call that ends in a pose to hold, a root-rotation jump limit (the same 26 degrees per tick
     HumanoidArena applies), a fresh state link, and a tilt guard that tells the stack to stop
     (damp) and disarms the session when the robot is falling.
  4. The deploy stack's safety check every tick and the motors' own limits.
"""
from __future__ import annotations

import base64
import io
import json
import math
import os
import sys
import threading
import time
import uuid
from pathlib import Path

import numpy as np

from services import arming

from . import robot as R
from . import tools as T
from .bridge import ReferenceConverter, Streamer, ZmqPublisher, quat_from_matrix
from .robot import RejectedBeforeAction
from .state import StateSubscriber
from .zmq_protocol import IDLE, SLOW_WALK, UPPER_BODY_INDICES

WORKSPACE_NAME = 'agent-workspace'

DEFAULTS = {
    'zmq_host': '127.0.0.1',          # this service binds the pose socket here; the stack's --zmq-host points at it
    'pose_port': 5556,                # the stack's --zmq-port
    'pose_topic': 'pose',
    'protocol_version': 1,            # joint_pos + joint_vel + body_quat + frame_index (+ Dex3 hand joints)
    'state_host': '127.0.0.1',        # the machine running the stack (its --output-type zmq publisher)
    'state_port': 5557,               # the stack's --zmq-out-port
    'state_topic': 'g1_debug',
    'state_joint_order': 'mujoco',    # the stack publishes joints in MuJoCo order; 'isaaclab' if a build does not
    'control_hz': 50.0,
    'wall_time_scale': 1.0,           # sim only: wall seconds per second of robot time (a deploy started with the same
                                      # SONIC_TIME_SCALE); streaming and walk phases are paced by it
    'state_max_age_s': 0.2,           # a state older than this is a lost link: motion refused or stopped
    'hands': True,                    # send Dex3 targets with every frame
    'joint_limit_margin_rad': 0.05,
    'max_joint_step_rad': 0.15,       # between consecutive references (7.5 rad/s at 50 Hz)
    'max_first_step_rad': 0.35,       # between the pose the robot holds and a call's first row
    'max_root_delta_deg': 26.0,       # a larger root-rotation jump per tick is slerped down, as in HumanoidArena
    'rest_tail_rows': 5,              # a call longer than this must end with this many identical rows
    'fall_tilt_deg': 60.0,            # above this the stack is told to stop (damp) and the session disarms
    'fall_damp': True,
    'walk_turn_rate_deg_s': 30.0,     # g1_walk: how fast the commanded facing turns (a 170 deg turn at 45 deg/s
                                      # spun the waist at 7 rad/s and the robot fell)
    'walk_settle_s': 1.5,             # g1_walk: standing still under the planner before the hold is primed again
    'walk_quiet_timeout_s': 6.0,      # ... and then up to this long until it is quiet (settle_* thresholds); if it is
                                      # not, the planner keeps balancing it and the next armed call hands over
    'walk_warmup': True,              # g1_walk: the first walk after a start begins with two short turns (planner warm-up)
    'walk_stuck_recover_s': 20.0,     # a walk left the robot stepping under the planner: the next armed call realigns the
                                      # planner's facing with the measured heading, then takes one slow step and idles
                                      # again (gait reset), waiting for quiet at most this long after the walk ended
    'walk_stuck_min_wait_s': 3.0,     # ... and at least this long for quiet after the gait reset
    'walk_stuck_reset_s': 1.0,        # ... the gait reset's slow step (0.1 m/s; about 0.1 m forward); 0: none
    'walk_stuck_force_handover': False,  # still stepping after that: False = the planner keeps balancing the robot and the
                                      # call fails asking for the operator; True = prime the hold at the least-moving
                                      # instant of walk_stuck_pick_s and ease to the default stance (sim: fell 1 in 2)
    'walk_stuck_pick_s': 1.5,
    'walk_segment_m': 0.5,            # g1_walk: longer walks run as segments of at most this (planner re-started each)
    'walk_ramp_s': 0.0,               # g1_walk: ramp the commanded speed between 0.1 m/s and speed_mps over this long
                                      # at the start and before the stop (robot time)
    'walk_closed_below_m': 0.3,       # g1_walk: walks shorter than this run until the planned base has advanced the
                                      # distance (longer ones are timed, distance / speed: their calibration holds)
    'walk_extend_max_s': 2.0,         # g1_walk: how much longer than distance / speed a walk may run until the planned
                                      # base has advanced the distance
    'walk_turn_speed_mps': 0.1,       # turns step forward this slowly; 0 pivots in place (fine in chains of turns,
                                      # but a pivot right after a walk, feet 24 cm staggered, fell)
    'start_timeout_s': 8.0,           # state must flow this soon after the backend starts the stack's control
    'crouch_arms_home_s': 2.0,        # g1_crouch: the arms return to the standing pose over this before the legs rise
    'stand_ramp_s': 1.5,              # after a fresh start, the hold eases from the measured stance to SONIC's default
                                      # standing pose over this long (0: hold the measured stance)
    'planner_settle_s': 3.0,          # after a fresh start, the least time the robot stands under the stack's planner
                                      # before the hold stream is primed (the gantry releases meanwhile) ...
    'settle_quiet_s': 1.0,            # ... and then until it has been quiet this long: tilt under settle_tilt_deg and
    'settle_tilt_deg': 10.0,          # every joint slower than settle_joint_speed (a robot still catching itself after
    'settle_joint_speed': 1.0,        # the release must not be frozen mid-step) ...
    'settle_static_tilt_deg': 14.0,   # ... or, motionless for settle_static_s, a tilt up to this (a robot leaning a hand
    'settle_static_s': 3.0,           # on a desk stood still at 10.6 deg and the hold never took over)
    'settle_timeout_s': 15.0,         # ... giving up after this long (the start then fails and the session can retry)
    'stack_restart_s': 2.0,           # state older than this while streaming: the stack stopped or restarted, so the
                                      # hold is forgotten and the next armed motion starts it again
    'console_start_mode': 'stand',    # console START: 'planner' = stand under the walking planner until quiet (it
                                      # steps in place first); 'stand' = switch to streamed mode right after the first
                                      # state, where the stack holds its standing reference (frame 0, paused): no steps
    'console_start_timeout_s': 180.0, # console START then gives the robot this long to come to rest on its feet (the
                                      # operator lowers the gantry meanwhile); it stays under the planner if it never does
    'camera': 'network',              # 'network': HomeBody's camera_server.py on the robot computer; 'ma-teleop': the
                                      # lab's Jetson publisher (ZMQ 5620, colour only, needs camera_intrinsics); None
    'camera_host': '192.168.123.164',
    'camera_port': 7790,
    'camera_max_age_s': 1.0,
    'camera_pose_override': None,     # {'position_m': [x, y, z], 'quaternion_wxyz': [...]} in the pelvis frame;
                                      # None: the URDF's d435_link, where HumanoidArena mounts the front camera
    'camera_intrinsics': None,        # 3x3 for a camera whose stream carries none (ma-teleop: the ZED calibration)
    'agent_image_width': 640,         # the observation (and its depth) is scaled to this width
}


class MotionStopped(RuntimeError):
    """A motion the backend stopped on purpose; the reply carries the measured state."""
    handled_stop = True


class G1SonicBackend:
    mutating = T.MUTATING

    def __init__(self, config, *, streamer=None, state=None, camera=None, clock=time.monotonic, wall=time.time):
        self.config = config
        unknown = set(config.backend_options) - set(DEFAULTS)
        if unknown:
            raise ValueError(f'unknown g1sonic backend options: {sorted(unknown)}')
        self.options = {**DEFAULTS, **config.backend_options}
        o = self.options
        self.clock, self.wall = clock, wall
        self.workspace = config.root / WORKSPACE_NAME
        self.frames_root = self.workspace / 'runtime' / 'frames'
        self.depth = config.observation_profile == 'rgbd'
        from .g1 import model as g1model
        self.model = g1model.model()
        limits = self.model.limits(list(R.JOINT_NAMES))
        self.lower = np.array([limits[name][0] for name in R.JOINT_NAMES])
        self.upper = np.array([limits[name][1] for name in R.JOINT_NAMES])
        self._ankle_z_standing = self._ankle_height(R.DEFAULT_STANDING)  # the leg kinematics' reference for crouch depth
        self.state = state if state is not None else StateSubscriber(
            o['state_host'], o['state_port'], o['state_topic'], joint_order=o['state_joint_order'], clock=clock).start()
        self.publisher = None
        if streamer is None:
            converter = ReferenceConverter(max_root_delta_deg=o['max_root_delta_deg'], control_dt=1.0 / o['control_hz'])
            self.publisher = ZmqPublisher(o['zmq_host'], o['pose_port'])
            streamer = Streamer(self.publisher.send, converter, hz=o['control_hz'] / o['wall_time_scale'], clock=clock,
                                topic=o['pose_topic'],
                                version=o['protocol_version'], hands=o['hands'])
        self.streamer = streamer.start()
        self.converter = streamer.converter
        self.camera, self.camera_error = camera, None
        if camera is None and o['camera'] in ('network', 'ma-teleop'):
            if o['camera'] == 'ma-teleop' and self.depth:
                raise ValueError("camera 'ma-teleop' (the Jetson's ZED stream) has no depth: use the rgb-only "
                                 'observation profile')
            try:
                if o['camera'] == 'network':  # HomeBody's camera server on the robot computer
                    from .camera import NetworkCamera
                    self.camera = NetworkCamera(o['camera_host'], o['camera_port'], clock=wall).start()
                else:  # MA-Teleop's camera publisher on the Jetson
                    from .camera import MaTeleopCamera
                    self.camera = MaTeleopCamera(o['camera_host'], o['camera_port'], o['camera_intrinsics'],
                                                 clock=wall).start()
            except Exception as exc:  # the session still runs; observe says why there is no image
                self.camera_error = str(exc)[:300]
        self.cameras = R.CAMERAS if self.camera is not None else ()
        self.step_id = 0
        self.last_row = None          # the last semantic40 row on the wire
        self.last_feedback = None
        self._raw = None              # the current observation's depth and geometry, for pixel_to_position
        self.plans = {}
        self.events = []
        self.fault = None
        self.last_fault = None
        self.fault_disarmed_at = None
        self.control_mode = 'idle'    # 'idle' until the first armed motion starts the stack; then 'streamed'
        self._last_index = None       # the stack's state index last seen; a smaller one means it restarted
        self.lock = threading.Lock()  # one motion at a time
        try:  # scripts/g1_console.py commands (poll_operator): one left in operator/stack.json by an earlier robot
            # service is history, not an order (a stale STOP would damp the robot the moment this service starts)
            self._operator_nonce = json.loads((Path(self.config.root)/'operator'/'stack.json').read_text()).get('nonce')
        except (OSError, ValueError, AttributeError):
            self._operator_nonce = None
        self._operator_status_at = 0.0
        self._watching = True         # the fall watch also covers the time between calls (the robot holds a frame)
        self._watch = threading.Thread(target=self._watch_falls, name='g1sonic-fall-watch', daemon=True)
        self._watch.start()

    # ----- tools ----------------------------------------------------------------------------
    def tools(self):
        return T.tool_definitions(geometry=self.depth)

    def validate(self, name, arguments):
        T.validate_request(name, arguments)
        if name == T.PIXEL_TO_POSITION and not self.depth:
            raise RejectedBeforeAction('pixel_to_position needs the rgbd observation profile')
        if name == T.STEP:
            snap = self._require_armed(start=False)
            self._check_rows(np.asarray(arguments['actions'], dtype=float), snap)
        elif name == T.FREE_SPACE_MOVE:
            self._require_armed(start=False) if not arguments.get('preview_only') else self._snapshot()
        elif name == T.WALK:
            self._require_armed(start=False)
        elif name == T.CROUCH:
            self._require_armed(start=False)
        elif name == T.EXECUTE_PLAN:
            self._require_armed(start=False)
            plan = self.plans.get(arguments['motion_plan_id'])
            if plan is None:
                raise RejectedBeforeAction('unknown or expired motion_plan_id; plan again')
            if plan['step_id'] != self.step_id:
                raise RejectedBeforeAction('the robot moved since this plan was made; plan again from the current state')
        elif name == T.PIXEL_TO_POSITION:
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
            return self._observation_packet()
        if name == T.HOLD:
            return self._hold()
        if name == T.PIXEL_TO_POSITION:
            return self._pixel_to_position(arguments), []
        if name in (T.STEP, T.FREE_SPACE_MOVE, T.EXECUTE_PLAN, T.WALK, T.CROUCH):
            try:
                if name == T.STEP:
                    return self._step(np.asarray(arguments['actions'], dtype=float), 'semantic40')
                if name == T.FREE_SPACE_MOVE:
                    return self._free_space_move(**arguments)
                if name == T.EXECUTE_PLAN:
                    return self._execute_plan(arguments['motion_plan_id'])
                if name == T.CROUCH:
                    return self._crouch(**arguments)
                return self._walk(**arguments)
            finally:
                self._motion_ended = self.wall()  # observations wait for an image captured after this
        if name == T.HAND_SHAPE:
            return self._hand_shape(**arguments), []
        raise ValueError(f'unknown tool {name}')

    def _hand_shape(self, side, open=None, closed=None):
        """Set (or with neither, restore) the Dex3 targets SIDE's closing fraction interpolates between."""
        with self.lock:
            if open is None:
                self.converter.hand_shapes[side] = None
            else:
                self.converter.hand_shapes[side] = (np.asarray(open, dtype=np.float32), np.asarray(closed, dtype=np.float32))
            self._event(f'hand shape {side}: ' + ('default' if open is None else f'open {list(open)} closed {list(closed)}'))
            return {'side': side, 'hand_shapes': self._hand_shapes_report(),
                    'note': 'moves nothing; the next g1_step rows use this shape for the closing fraction'}

    def _hand_shapes_report(self):
        report = {}
        for side in ('left', 'right'):
            report[side] = {'open': np.round(self.converter.hand(side, 0.0), 4).tolist(),
                            'closed': np.round(self.converter.hand(side, 1.0), 4).tolist(),
                            'default': self.converter.hand_shapes.get(side) is None}
        return report

    # ----- state and arming -----------------------------------------------------------------
    def _snapshot(self, fresh=True):
        snap = self.state.latest()
        if snap is None:
            if self.control_mode == 'idle':
                raise RuntimeError("the SONIC deploy stack publishes no state until its control runs; an armed "
                                   f'{T.HOLD} or {T.STEP} starts it (the robot stands)')
            raise RuntimeError(f"no state from the SONIC deploy stack at {self.options['state_host']}:"
                               f"{self.options['state_port']} (is g1_deploy_onnx_ref running with --output-type zmq?)")
        age = self.clock() - snap.received
        if fresh and age > self.options['state_max_age_s']:
            raise RuntimeError(f'the state from the SONIC deploy stack is {age:.2f} s old; the link is lost or the stack stopped')
        return snap

    def _require_armed(self, start=True):
        """The operator's arming and fault acknowledgement; with `start`, the stack's control running too."""
        state = arming.read(self.config.root)
        if not state['armed']:
            raise PermissionError(f"motion is not armed ({state['reason']}); the operator arms motion")
        if not self._acknowledge_fault(state):
            raise PermissionError('the backend disarmed the session for a fault; the operator arms again on purpose')
        self._check_stack()
        try:
            if start:
                self._ensure_control()
            elif self.state.latest() is None:
                return None
            return self._snapshot()
        except RuntimeError as exc:
            raise PermissionError(str(exc)) from None

    def _ensure_control(self):
        """Start the stack's control if it is idle and stream a hold of the measured pose.

        The stack publishes state only once control runs, and a start over ZMQ takes effect only in
        its planner mode (in streamed-motion mode the start is the keyboard's), so the first armed
        motion starts it the way MA-Teleop's clients do: start under the planner, let the robot stand
        there for `planner_settle_s` (the operator lowers and releases it meanwhile), prime the hold
        from the measured stance, switch the stack to streamed motion, and ease the hold to SONIC's
        default standing pose over `stand_ramp_s`. Right after a release the measured pose is
        whatever step the robot was taking: holding that wide, twisted stance made the robot keep
        turning, and jumping to the default stance in one frame tipped it over. A stack that was
        already in control is held where it is.
        The deploy needs its planner loaded (`--planner-file`): without one the first start stops it.
        """
        if self.streamer.primed:
            return
        o = self.options
        if self.control_mode == 'planner':  # a walk left the robot under the planner: hand over, within a bounded time
            self._recover_from_stepping()
            return
        started = self.clock()
        snap = self.state.latest()
        fresh_start = snap is None or started - snap.received > o['state_max_age_s']  # none, or left by a stack that is gone
        if fresh_start:
            self._planner_warm = False
            self._event('start sent to the stack (planner mode, standing)')
            deadline = started + o['start_timeout_s']
            snap = None
            while snap is None and self.clock() < deadline:
                self.streamer.command(start=True, planner=True)  # repeated: a PUB drops what no one has joined yet
                snap = self.state.wait_after(started, 0.5)
            if snap is None:
                raise RuntimeError(f"no state from the SONIC deploy stack {o['start_timeout_s']:g} s after start: it is not "
                                   'running with --input-type zmq_manager --output-type zmq, is still in INIT, or was '
                                   'launched without --planner-file (a start over ZMQ needs planner mode, and planner '
                                   'mode without a planner stops the deploy)')
            if o['planner_settle_s'] > 0:
                self._event(f"standing under the planner for at least {o['planner_settle_s']:g} s, until quiet")
                time.sleep(o['planner_settle_s'])
                snap = self._wait_until_quiet()
        self._last_index = snap.index
        self.streamer.prime(self._within_limits(snap.q), snap.base_quat)
        self.streamer.command(start=True, planner=False)
        self.control_mode = 'streamed'
        self._event('streamed motion on; the hold stream starts from the measured pose')
        if fresh_start and o['stand_ramp_s'] > 0:
            self._ease_to_default_stance(snap.q)

    def _recover_from_stepping(self, budget_s=None):
        """A walk left the robot stepping under the planner (it never met the quiet thresholds): try, within
        `walk_stuck_recover_s` of the walk's end, to bring it to rest so the streamed hold can take over. (1) The
        keepalive's IDLE facing is realigned with the measured heading: an IDLE facing the robot never quite reaches
        keeps the planner turning it (sim: 15 min of stepping in place after a warm-up turn). (2) One slow step
        forward (`walk_stuck_reset_s` at 0.1 m/s), then idle again: a fresh short motion ends in a stand far more
        reliably than a stuck idle (sim: 84 of 84 short planner motions handed over). Still stepping after that, the
        planner keeps balancing the robot and the caller is told to involve the operator: the hold is not forced onto
        a stepping robot unless `walk_stuck_force_handover` is on (priming the least-moving instant and easing to the
        default stance fell 1 time in 2 in sim)."""
        o = self.options
        since = getattr(self, '_stepping_since', None)
        since = self.clock() if since is None else since
        budget = o['walk_stuck_recover_s'] if budget_s is None else budget_s
        deadline = max(since + budget, self.clock() + (o['walk_stuck_min_wait_s'] if budget_s is None else 0.0))
        extra = getattr(self, '_planner_extra', None) or {}
        frame_yaw = getattr(self, '_planner_frame_yaw', None)

        def facing_now():
            snap = self.state.latest()
            if snap is None or frame_yaw is None:
                return [1.0, 0.0, 0.0], None
            d = snap.yaw_rad() - frame_yaw
            return [math.cos(d), math.sin(d), 0.0], math.degrees(math.atan2(math.sin(d), math.cos(d)))

        def keep(facing):
            self._planner_keepalive = lambda: self.streamer.planner(IDLE, [0.0, 0.0, 0.0], facing, -1.0, **extra)
            self.streamer.keepalive = self._planner_keepalive

        facing, d = facing_now()
        if d is not None:
            keep(facing)
            self._event(f'still stepping {self.clock() - since:.0f} s after the walk: the planner facing is realigned with '
                        f'the measured heading ({d:+.0f} deg in its frame)')
        half = self.clock() + max(0.0, (deadline - self.clock()) / 2)
        quiet = self._wait_until_quiet(timeout=max(0.0, half - self.clock()), raise_on_timeout=False, strict=True)
        if quiet is None and o['walk_stuck_reset_s'] > 0:
            facing, _ = facing_now()
            self._event(f"still stepping: one slow step forward ({o['walk_stuck_reset_s']:g} s at 0.1 m/s) to reset the gait, then idle")
            self.streamer.keepalive = None
            for phase_end, mode, speed in ((self.clock() + o['walk_stuck_reset_s'], SLOW_WALK, 0.1),
                                           (self.clock() + o['walk_stuck_reset_s'] + o['walk_settle_s'] * o['wall_time_scale'], IDLE, -1.0)):
                while self.clock() < phase_end:
                    self.streamer.planner(mode, facing if mode == SLOW_WALK else [0.0, 0.0, 0.0], facing, speed, **extra)
                    time.sleep(0.05)
            keep(facing)
            quiet = self._wait_until_quiet(timeout=max(o['walk_stuck_min_wait_s'], deadline - self.clock()),
                                           raise_on_timeout=False, strict=True)
        if quiet is not None:
            self._hand_over_to_hold(quiet)
            self._event('quiet: the streamed hold took over from the planner')
            return
        if not o['walk_stuck_force_handover']:
            self._event(f'still stepping {self.clock() - since:.0f} s after the walk: the planner keeps balancing the robot (operator)')
            raise RuntimeError(f'the robot is still stepping under the walking planner {self.clock() - since:.0f} s after the '
                               'walk (facing realigned and one gait reset tried); the planner keeps it balanced, which is safe. '
                               'Ask the operator (request_operator): they watch it settle, or STOP the stack with the band on. '
                               'Then call g1_hold again. The hold is not forced onto a stepping robot (that fell in sim)')
        best, pick_end = None, self.clock() + o['walk_stuck_pick_s']
        while self.clock() < pick_end:
            keepalive = getattr(self, '_planner_keepalive', None)
            if keepalive is not None:
                keepalive()
            snap = self.state.wait_after(self.clock() - 0.01, 0.5)
            if snap is None:
                continue
            speed = float(np.abs(snap.dq).max()) if snap.dq is not None else 0.0
            if best is None or speed < best[0]:
                best = (speed, snap)
        if best is None:
            raise RuntimeError('no state from the deploy stack while the robot was stepping under the planner')
        speed, snap = best
        if snap.tilt_deg() > o['settle_static_tilt_deg']:
            raise RuntimeError(f'the robot is still stepping under the walking planner {self.clock() - since:.0f} s after '
                               f'the walk, tilted {snap.tilt_deg():.0f} deg: too far from upright for the hold to take over; '
                               'the planner keeps balancing it. The operator checks the robot; then call g1_hold again')
        carry = getattr(self, '_carry_hands', None)
        self._event(f'the robot never settled ({self.clock() - since:.0f} s): walk_stuck_force_handover: the hold is primed at '
                    f'the least-moving instant (fastest joint {speed:.1f} rad/s, tilt {snap.tilt_deg():.1f} deg) and eased to '
                    'the default stance' + (' (arms and hands kept: carrying)' if carry is not None and carry[0] is not None else ''))
        self._hand_over_to_hold(snap)
        if o['stand_ramp_s'] > 0:
            self._ease_to_default_stance(snap.q, keep_upper=carry is not None and carry[0] is not None)

    def _hand_over_to_hold(self, snap, hands=None):
        """From the walking planner back to the streamed hold of the measured (quiet) pose (with `hands`, the
        Dex3 targets a carrying walk held; else those of a carry the next call hands over from, else open)."""
        keepalive, self._planner_keepalive = getattr(self, '_planner_keepalive', None), None
        self.streamer.keepalive = None
        if hands is None:
            hands = getattr(self, '_carry_hands', None)
        self._carry_hands = None
        if hands is None or hands[0] is None:
            hands = (False, False)
        self.streamer.prime(self._within_limits(snap.q), snap.base_quat, hands=hands)
        for _ in range(4):  # the measured pose is on the wire before the stack switches back to it
            if keepalive is not None:
                keepalive()  # (a carrying walk: keep the arms held meanwhile)
            time.sleep(0.05)
        if getattr(self, '_upper_latched', None) is not None:
            # the deploy keeps a planner message's upper body (has_upper_body_control_) across the switch to streamed
            # motion (cleared only on a stop or a 1 s planner gap in planner mode), so streamed arm references were
            # ignored after a carrying walk. Clearing it with a bare planner message let the arms sag 5-17 cm and
            # jolted a carried can loose; instead the streamer mirrors the streamed upper body into planner messages
            self.streamer.mirror_upper = True
            self._upper_latched = None
        self.streamer.command(start=True, planner=False)
        self.control_mode = 'streamed'
        self._last_index = snap.index
        self._stepping_since = None

    def _wait_until_quiet(self, timeout=None, raise_on_timeout=True, strict=False):
        """The first state after the robot has stood still for `settle_quiet_s` under the planner."""
        o = self.options
        deadline = self.clock() + (o['settle_timeout_s'] if timeout is None else timeout)
        quiet_since, static_since, worst = None, None, None
        while self.clock() < deadline:
            keepalive = getattr(self, '_planner_keepalive', None)
            if keepalive is not None:  # the deploy drops a planner message's upper body after 1 s without one
                keepalive()
            snap = self.state.wait_after(self.clock() - 0.01, 0.5)
            if snap is None:
                continue
            speed = float(np.abs(snap.dq).max()) if snap.dq is not None else 0.0
            worst = (round(snap.tilt_deg(), 1), round(speed, 2))
            if snap.tilt_deg() < o['settle_tilt_deg'] and speed < o['settle_joint_speed']:
                quiet_since = quiet_since or self.clock()
                if self.clock() - quiet_since >= o['settle_quiet_s']:
                    return snap
            else:
                quiet_since = None
            if snap.tilt_deg() < o['settle_static_tilt_deg'] and speed < 0.3 * o['settle_joint_speed']:
                static_since = static_since or self.clock()
                if self.clock() - static_since >= o['settle_static_s']:
                    return snap
            else:
                static_since = None
            time.sleep(0.02)
        if not raise_on_timeout:
            return None if strict else self.state.latest()
        raise RuntimeError(f"the robot did not come to rest under the stack's planner within {o['settle_timeout_s']:g} s "
                           f'(last tilt {worst[0] if worst else "?"} deg, fastest joint {worst[1] if worst else "?"} rad/s): '
                           'it may be hanging unevenly or falling; the operator checks it before the next start')

    def _ease_to_default_stance(self, measured, keep_upper=False):
        """A smoothstep from the measured joints to SONIC's default standing pose at the anchored heading (with
        `keep_upper`, the legs only: the waist, arms and hands stay as held, for a carried object)."""
        o = self.options
        ticks = max(2, int(round(o['stand_ramp_s'] * o['control_hz'])))
        start = np.clip(np.asarray(measured, dtype=float), self.lower + o['joint_limit_margin_rad'],
                        self.upper - o['joint_limit_margin_rad'])
        goal = R.DEFAULT_STANDING.copy()
        hands = (0.0, 0.0)
        if keep_upper:
            goal[list(UPPER_BODY_INDICES)] = start[list(UPPER_BODY_INDICES)]
            held = self.converter.last
            if held is not None:
                hands = tuple(self.converter.hand_fraction(side, getattr(held, f'{side}_hand')) for side in ('left', 'right'))
        rows = []
        for i in range(1, ticks + 1):
            u = i / ticks
            row = np.zeros(R.ACTION_DIM)
            row[R.ACTION_SLICES['root_height_m']] = 0.75
            row[R.ROT6D] = R.IDENTITY_ROT6D
            row[R.JOINTS] = start + (goal - start) * (u * u * (3 - 2 * u))
            row[38], row[39] = hands
            rows.append(row)
        rows += [rows[-1].copy() for _ in range(o['rest_tail_rows'])]
        self.streamer.execute(rows, None)
        self.last_row = rows[-1].copy()
        self._event(f"eased to the default standing pose over {o['stand_ramp_s']:g} s")

    def _check_stack(self):
        """A stopped or relaunched deploy stack: forget the hold, so the next armed motion starts it again."""
        snap = self.state.latest()
        if snap is None or not self.streamer.primed:
            return
        age = self.clock() - snap.received
        if self._last_index is not None and snap.index < self._last_index:
            self._reset_control(f'the deploy stack restarted (state index {snap.index} after {self._last_index})')
        elif age > self.options['stack_restart_s']:
            self._reset_control(f'no state from the deploy stack for {age:.1f} s: it stopped or restarted')
        else:
            self._last_index = snap.index

    def _reset_control(self, reason):
        self.streamer.keepalive = None
        self.streamer.hold()
        self.converter.clear()
        self.control_mode = 'idle'
        self.last_row = None
        self.plans = {}
        self._last_index = None
        self._event('control reset: ' + reason + '; an armed hold or step starts the stack again')

    def _acknowledge_fault(self, state=None):
        """True unless a fault still waits for the operator; an arming made after the fault clears it.

        Cleared, the fault moves to `last_fault`, so feedback stops reporting it and the next fall is
        caught, damped and disarmed like the first.
        """
        state = state or arming.read(self.config.root)
        if self.fault_disarmed_at is None:
            return True
        if not state['armed'] or (state.get('since') or 0) <= self.fault_disarmed_at:
            return False
        if self.fault is not None:
            self._event('fault acknowledged by re-arming: ' + self.fault['what'])
            self.last_fault, self.fault = self.fault, None
        self.fault_disarmed_at = None
        return True

    def _event(self, what, **details):
        self.events = (self.events + [{'at': round(self.wall(), 2), 'what': what, **details}])[-20:]

    def _disarm(self, reason, fault=None):
        self.streamer.keepalive = None
        now = self.wall()
        if fault is not None:
            self.fault = {'at': round(now, 2), 'kind': fault, 'what': reason}
            self.fault_disarmed_at = now
        self._event('disarmed: ' + reason)
        if arming.read(self.config.root)['armed']:
            arming.write(self.config.root, False, by='g1sonic-backend', reason=reason)

    # ----- checks ---------------------------------------------------------------------------
    def _within_limits(self, q):
        """Measured joints clipped inside the limits minus the margin: a hold primed on a joint resting at (or, in the
        gantry, slightly past) its limit would otherwise make every row built from it fail _check_rows."""
        margin = self.options['joint_limit_margin_rad']
        return np.clip(np.asarray(q, dtype=float), self.lower + margin, self.upper - margin)

    def _check_rows(self, rows, snap):
        o = self.options
        if rows.ndim != 2 or rows.shape[1] != R.ACTION_DIM:
            raise RejectedBeforeAction(f'actions must be N x {R.ACTION_DIM}')
        joints = rows[:, R.JOINTS]
        margin = o['joint_limit_margin_rad']
        low, high = joints < self.lower + margin, joints > self.upper - margin
        if low.any() or high.any():
            bad = sorted({R.JOINT_NAMES[j] for j in np.where(low.any(axis=0) | high.any(axis=0))[0]})
            raise RejectedBeforeAction(f'joint references outside the limits minus {margin:g} rad: {bad}')
        for row_index, six in enumerate(rows[:, R.ROT6D]):
            if np.linalg.norm(six[0::2]) < 0.5 or np.linalg.norm(six[1::2]) < 0.5:
                raise RejectedBeforeAction(f'actions[{row_index}][3:9] is not a rotation (degenerate rot6d)')
        reference = self.last_row[R.JOINTS] if self.last_row is not None else (None if snap is None else self._within_limits(snap.q))
        first = None if reference is None else np.abs(joints[0] - reference)  # None before control runs: checked again at the step
        if first is not None and first.max() > o['max_first_step_rad']:
            j = int(first.argmax())
            raise RejectedBeforeAction(f'the first row moves {R.JOINT_NAMES[j]} by {first[j]:.2f} rad from the held pose; '
                                       f"at most {o['max_first_step_rad']:g} rad: start from the measured joints")
        if len(rows) > 1:
            steps = np.abs(np.diff(joints, axis=0))
            if steps.max() > o['max_joint_step_rad']:
                row_index, j = np.unravel_index(int(steps.argmax()), steps.shape)
                raise RejectedBeforeAction(f'row {row_index + 1} moves {R.JOINT_NAMES[j]} by {steps[row_index, j]:.2f} rad '
                                           f"in one tick; at most {o['max_joint_step_rad']:g} rad")
        tail = o['rest_tail_rows']
        if len(rows) > tail:
            last = rows[-tail:]
            if not (np.allclose(last[:, R.JOINTS], last[-1, R.JOINTS], atol=1e-6)
                    and np.allclose(last[:, R.HANDS], last[-1, R.HANDS])
                    and np.allclose(last[:, R.ROT6D], last[-1, R.ROT6D], atol=1e-6)):
                raise RejectedBeforeAction(f'end the call in a pose to hold: the last {tail} rows must be identical '
                                           '(the robot holds the last row until the next call)')

    # ----- motion ---------------------------------------------------------------------------
    def _step(self, rows, kind, extra=None, record_steps=True):
        """Stream ROWS and reply with the observation; `record_steps` False keeps the per-frame action list out of the
        reply (a crouch streams up to 500 synthesized rows the agent did not write: listing them only truncates its view)."""
        o = self.options
        with self.lock:
            snap = self._require_armed()
            self._check_rows(np.asarray(rows, dtype=float), snap)  # the first row against the pose the stack now holds
            trace = []
            started = self.clock()

            def on_tick(reference):
                now = self.clock()
                state = arming.read(self.config.root)
                if not state['armed']:
                    raise MotionStopped(f"the operator disarmed ({state['reason']}); the robot holds its last reference")
                latest = self.state.latest()
                age = math.inf if latest is None else now - latest.received
                if age > o['state_max_age_s']:
                    raise MotionStopped(f'no fresh state from the deploy stack for {age:.2f} s; the robot holds its last reference')
                tilt = latest.tilt_deg()
                trace.append({'frame_index': reference.frame_index, 'tilt_deg': tilt, 'projection_deg': reference.projection_deg,
                              'q': latest.q, 'last_action': latest.last_action, 'reference': reference.joint_pos})
                if tilt > o['fall_tilt_deg']:
                    self._fall(tilt)
                    raise MotionStopped(f"tilt {tilt:.0f} deg is over {o['fall_tilt_deg']:g}: "
                                        + ('the stack was told to stop and every joint is damped' if o['fall_damp']
                                           else 'the robot holds its last reference')
                                        + '; the session is disarmed')

            try:
                executed = self.streamer.execute(rows, on_tick)
            except MotionStopped as exc:
                self.streamer.hold()
                count = self._account(rows, self.streamer.executed)
                self._event('motion stopped: ' + str(exc), rows_sent=count)
                exc.robot_state = {**self._state_report(), 'rows_sent': count}
                raise
            except Exception as exc:
                self.streamer.hold()
                count = self._account(rows, self.streamer.executed)
                exc.robot_state = {**self._state_report(), 'rows_sent': count}
                raise
            count = self._account(rows, executed)
            feedback = self._feedback(trace, started)
            self.last_feedback = feedback
            steps = [{'step_id': self.step_id - count + i + 1, 'observation_step_id': self.step_id - count + i,
                      'executed_action': rows[i].tolist()} for i in range(count)]
            if not record_steps:  # first and last frame only, with the count
                steps = [steps[0], steps[-1]] if count > 1 else steps
            transition = {'kind': kind, 'control_frequency_hz': o['control_hz'], 'feedback': feedback, 'steps': steps,
                          'rows_executed': count, **(extra or {})}
            return self._observation_packet(transition=transition)

    # ----- crouch ---------------------------------------------------------------------------
    def _ankle_height(self, q, base_quat=None):
        """Mean vertical offset of the two ankle links from the pelvis (negative, metres), from the leg kinematics;
        with `base_quat`, measured along the world's vertical instead of the pelvis's."""
        joints = dict(zip(R.JOINT_NAMES, np.asarray(q, dtype=float).tolist()))
        points = [self.model.link_transform(f'{side}_ankle_roll_link', joints)[:3, 3] for side in ('left', 'right')]
        if base_quat is not None:
            from .g1.model import quat_to_matrix
            rotation = quat_to_matrix(np.asarray(base_quat, dtype=float))
            points = [rotation @ point for point in points]
        return float(np.mean([point[2] for point in points]))

    def pelvis_height_estimate(self, q, base_quat=None):
        """Pelvis height above the floor (m), assuming both feet flat: the standing height measured in simulation
        minus how far the ankles have risen toward the pelvis."""
        return T.STANDING_PELVIS_M + (self._ankle_z_standing - self._ankle_height(q, base_quat))

    def _crouch(self, pelvis_drop_m=0.0, lean_rad=0.0, forward_rad=0.0, secs=4.0, arms_home=True):
        """Legs-only posture change from the held frame: a smoothstep to the symmetric squat of CROUCH_TABLE (knee +d,
        ankle down by d/2 within its margin, hip taking the rest) at the given waist lean, every other joint as held.
        Rising with arms out first takes the arms home: a 1.5 s stand from a squat with a lean and an arm out fell twice
        in simulation, standing with the arms home first did not."""
        o = self.options
        hz = o['control_hz']
        J = {name: index for index, name in enumerate(R.JOINT_NAMES)}
        base = np.asarray(R.DEFAULT_STANDING, dtype=float)
        snap = self._require_armed()  # starts the stack's control, or takes over a running one, like g1_step
        if self.control_mode != 'streamed':
            raise RejectedBeforeAction(f'{T.CROUCH} starts from a held frame; the robot is {self.control_mode}: '
                                       f'call {T.HOLD} first')
        held = self._held_frame(snap)  # the last row on the wire, or the measured pose a fresh hold primed
        q0 = held[R.JOINTS].copy()
        d = float(np.interp(pelvis_drop_m, [drop for drop, _ in T.CROUCH_TABLE], [knee for _, knee in T.CROUCH_TABLE]))
        margin = o['joint_limit_margin_rad']
        ankle_floor = float(max(self.lower[J[f'{side}_ankle_pitch_joint']] + margin - base[J[f'{side}_ankle_pitch_joint']]
                                for side in ('left', 'right')))  # the ankle pitches down at most to its limit's margin
        ankle = max(-d / 2, ankle_floor)
        goal = q0.copy()
        for side in ('left', 'right'):
            goal[J[f'{side}_knee_joint']] = base[J[f'{side}_knee_joint']] + d
            goal[J[f'{side}_ankle_pitch_joint']] = base[J[f'{side}_ankle_pitch_joint']] + ankle - forward_rad
            goal[J[f'{side}_hip_pitch_joint']] = base[J[f'{side}_hip_pitch_joint']] - d - ankle + forward_rad
            for name in ('hip_roll', 'hip_yaw', 'ankle_roll'):  # a walk may have left these anywhere
                goal[J[f'{side}_{name}_joint']] = base[J[f'{side}_{name}_joint']]
        goal[J['waist_pitch_joint']] = lean_rad
        goal[J['waist_roll_joint']] = base[J['waist_roll_joint']]
        clipped_goal = np.clip(goal, self.lower + margin, self.upper - margin)
        clipped = [R.JOINT_NAMES[j] for j in np.where(np.abs(clipped_goal - goal) > 1e-9)[0]]
        goal = clipped_goal
        knees = [J['left_knee_joint'], J['right_knee_joint']]
        arms = [J[f'{side}_{name}_joint'] for side in ('left', 'right')
                for name in ('shoulder_pitch', 'shoulder_roll', 'shoulder_yaw', 'elbow', 'wrist_roll', 'wrist_pitch', 'wrist_yaw')]
        rising = float(np.mean(q0[knees] - base[knees])) > d + 0.1
        arms_out = float(np.abs(q0[arms] - base[arms]).max()) > 0.05

        def smooth(start, end, ticks):
            rows = []
            for i in range(1, max(2, ticks) + 1):
                u = i / max(2, ticks)
                row = held.copy()
                row[R.JOINTS] = start + (end - start) * (u * u * (3 - 2 * u))
                rows.append(row)
            return rows

        rows, phases = [], []
        if arms_home and rising and arms_out:
            arm_goal = q0.copy()
            arm_goal[arms] = base[arms]
            goal[arms] = base[arms]
            rows += smooth(q0, arm_goal, int(round(o['crouch_arms_home_s'] * hz)))
            phases.append('arms home')
            q0 = arm_goal
        rows += smooth(q0, goal, int(round(secs * hz)))
        phases.append('rise' if rising else 'crouch')
        rows += [rows[-1].copy() for _ in range(o['rest_tail_rows'])]
        report = {'pelvis_drop_m': pelvis_drop_m, 'knee_flexion_rad': round(d, 4), 'ankle_pitch_rad': round(ankle - forward_rad, 4),
                  'lean_rad': lean_rad, 'forward_rad': forward_rad, 'phases': phases, 'clipped_to_limits': clipped,
                  'pelvis_height_est_m': {'before': round(self.pelvis_height_estimate(snap.q, snap.base_quat), 3),
                                          'goal': round(self.pelvis_height_estimate(goal), 3)}}
        value, images = self._step(np.asarray(rows), 'crouch', {'crouch': report}, record_steps=False)
        latest = self.state.latest()
        if latest is not None:
            report['pelvis_height_est_m']['after'] = round(self.pelvis_height_estimate(latest.q, latest.base_quat), 3)
        return value, images

    def _walk(self, forward_m=0.0, turn_deg=0.0, speed_mps=0.3, carry=False):
        """Long walks run as segments of at most `walk_segment_m`, each with its own planner start and stop: the
        planner's reference runs ahead of a lagging robot over a long walk, and the policy then lunges to catch up
        (sim: falls and runaways in 0.9 m walks, none in 99 walks of 0.5 m)."""
        warmed_up = False
        if self.options['walk_warmup'] and not getattr(self, '_planner_warm', False):
            # the first planner use after a start lags its own reference by 0.2-0.7 m and then lunges to catch up
            # (sim: falls); two short turns first warm it up (first-walk lag 0.04-0.07 m afterwards)
            warmed_up = True
            self._event('walk warm-up: +15 / -15 deg turns before the first walk')
            for d in (15.0, -15.0):
                self._walk_segment(0.0, d, speed_mps, carry)
                if self.control_mode == 'planner':
                    quiet = self._wait_until_quiet(timeout=self.options['walk_quiet_timeout_s'], raise_on_timeout=False,
                                                   strict=True)
                    if quiet is not None:
                        with self.lock:
                            self._hand_over_to_hold(quiet)
            self._planner_warm = True
        seg = self.options['walk_segment_m']
        if seg <= 0 or forward_m <= seg * 1.2:
            packet = self._walk_segment(forward_m, turn_deg, speed_mps, carry)
            self._walk_notes(packet, warmed_up)
            return packet
        count = int(math.ceil(forward_m / seg))
        packet = None
        start = self.state.latest()
        for k in range(count):
            packet = self._walk_segment(forward_m / count, turn_deg if k == 0 else 0.0, speed_mps, carry)
            if self.control_mode == 'planner':  # still stepping: let the next segment's start hand over after quiet
                quiet = self._wait_until_quiet(timeout=self.options['walk_quiet_timeout_s'], raise_on_timeout=False, strict=True)
                if quiet is not None:
                    with self.lock:
                        self._hand_over_to_hold(quiet)
        fb = packet[0]['transition']['feedback'] if isinstance(packet, tuple) else packet['transition']['feedback']
        fb['commanded'] = {'forward_m': forward_m, 'turn_deg': turn_deg, 'speed_mps': speed_mps, 'segments': count}
        end = self.state.latest()
        if start is not None and end is not None:
            d = end.yaw_rad() - start.yaw_rad()
            fb['measured_turn_deg'] = round(math.degrees(math.atan2(math.sin(d), math.cos(d))), 1)
        self._walk_notes(packet, warmed_up)
        return packet

    def _walk_notes(self, packet, warmed_up):
        """What else the walk did: the first-walk warm-up turns, and whether the hold took over or the planner still
        balances a stepping robot (the next armed call then hands over within walk_stuck_recover_s)."""
        fb = packet[0]['transition']['feedback'] if isinstance(packet, tuple) else packet['transition']['feedback']
        if warmed_up:
            fb['warm_up'] = 'first walk after a start: two 15 deg turns (left, right) ran before the commanded motion'
        if self.control_mode == 'planner':
            fb['control'] = (f"the robot was still stepping {self.options['walk_quiet_timeout_s']:g} s after the walk, so "
                             "SONIC's planner keeps balancing it (safe); call g1_hold: it tries for up to "
                             f"{self.options['walk_stuck_recover_s']:g} s to bring the robot to rest and take over, and says "
                             'so if it could not (then ask the operator)')
        else:
            fb['control'] = 'standing in the streamed hold'

    def _walk_segment(self, forward_m=0.0, turn_deg=0.0, speed_mps=0.3, carry=False):
        """Walk under SONIC's planner, then hold the measured pose in streamed motion again.

        The stack's planner frame has x along the heading the robot had when the planner took over, so the
        movement and facing directions are relative to the start of this walk. Facing turns at
        `walk_turn_rate_deg_s`; translation runs at `speed_mps` in the slow-walk mode; then the planner idles for
        `walk_settle_s`; turns pivot in place, or step slowly forward at `walk_turn_speed_mps` > 0. The streamed
        hold takes over only once the robot stands still; otherwise the planner keeps balancing it and the next armed
        call hands over.
        """
        o = self.options
        with self.lock:
            snap = self._require_armed()
            start_yaw, started = snap.yaw_rad(), self.clock()
            distance = forward_m  # the planner steps forward only (measured: backward and sideways commands do not move it)
            direction = 0.0
            turn = math.radians(turn_deg)
            move_s = distance / speed_mps
            if o['walk_ramp_s'] > 0 and distance > 0:
                ramp = min(o['walk_ramp_s'], move_s)
                move_s += ramp * (speed_mps - 0.1) / speed_mps  # the two ramps cover less ground than full speed
            turn_s = abs(turn_deg) / o['walk_turn_rate_deg_s']
            total = max(move_s, turn_s)
            held = self.converter.last
            upper, hands = None, (None, None)
            if held is not None:
                hands = (held.left_hand.copy(), held.right_hand.copy())
                if carry:
                    # the MEASURED upper body: streamed references are tracked only at the hand (SONIC keeps its own arm
                    # posture), while the planner tracks these joints literally; holding the last reference made the
                    # carrying arm drop 10-25 cm as the walk began (a held paper ball fell)
                    upper = np.asarray(snap.q, dtype=float)[list(UPPER_BODY_INDICES)].copy()
                    # the waist from the reference: a partly tracked lean (measured) held by the planner left the robot
                    # tilted 12 deg and stepping, so the hold never took over
                    waist = [k for k, j in enumerate(UPPER_BODY_INDICES) if 'waist' in R.JOINT_NAMES[j]]
                    upper[waist] = np.asarray(held.joint_pos, dtype=float)[[UPPER_BODY_INDICES[k] for k in waist]]
            extra = {'upper_body': upper, 'left_hand': hands[0], 'right_hand': hands[1]} if carry else {}
            self._planner_keepalive, psi = None, 0.0
            self.streamer.keepalive = None  # (the walk sends its own planner messages)
            # no mirrored idle planner messages during a walk (they would fight its own); a plain walk's bare messages
            # also clear the deploy's upper-body flag, a carrying walk sends its own upper body
            self.streamer.mirror_upper = False
            self.streamer.hold()
            self.streamer.command(start=True, planner=True)
            self.control_mode = 'planner'
            self._planner_frame_yaw, self._planner_extra, self._stepping_since = start_yaw, extra, None
            self._event(f'walk: forward {forward_m:g} m, turn {turn_deg:g} deg at {speed_mps:g} m/s'
                        + (' carrying (arms, waist and hands held)' if carry else ''))
            trace, stop_reason = [], None
            dt = 0.05
            try:
                begin = self.clock()
                scale = o['wall_time_scale']  # phases are in robot time
                trans0, advanced = None, None
                for phase, moving in (('stand', False), ('move', True), ('settle', False)):
                    while True:
                        t = (self.clock() - begin) / scale - 0.5  # 0.5 s of standing under the planner first
                        if moving and 0 < distance and t >= 0:
                            # walk until the stack's planned base has advanced `distance` (base_trans_target), not
                            # for distance / speed: the plan starts from standing, and short timed walks (under 0.2 m
                            # commanded) moved nothing (median 0 of 28); capped at walk_extend_max_s more. Longer walks
                            # stay timed; their planned advance is only reported (carrying walks stalled ~1 in 10)
                            latest_tt = self.state.latest().raw.get('base_trans_target') if self.state.latest() else None
                            if latest_tt is not None:
                                tt = np.asarray(latest_tt, dtype=float)[:2]
                                trans0 = tt if trans0 is None else trans0
                                advanced = float(np.linalg.norm(tt - trans0))
                                closed = distance < o['walk_closed_below_m']
                                if closed and t >= move_s and advanced < distance and t < move_s + o['walk_extend_max_s']:
                                    move_s = t + dt
                                    total = max(move_s, turn_s)
                                elif closed and advanced >= distance and t < move_s:
                                    move_s = t
                                    total = max(move_s, turn_s)
                        phase_end = {'stand': 0.5, 'move': 0.5 + total, 'settle': 0.5 + total + o['walk_settle_s']}[phase]
                        if t + 0.5 >= phase_end:
                            break
                        psi = turn * min(1.0, max(0.0, t) / turn_s) if turn_s > 0 else 0.0
                        facing = [math.cos(psi), math.sin(psi), 0.0]
                        if moving and distance > 0 and 0 <= t < move_s:
                            heading = psi + direction
                            ramp = o['walk_ramp_s']
                            # ramp up at the start and down before the stop: stopping mid-stride at full speed and
                            # starting at full speed from standing made SONIC's planner lunge and fall (~3% of walks)
                            v = speed_mps if ramp <= 0 else 0.1 + (speed_mps - 0.1) * min(1.0, t / ramp, (move_s - t) / ramp)
                            self.streamer.planner(SLOW_WALK, [math.cos(heading), math.sin(heading), 0.0], facing, v,
                                                  **extra)
                        elif moving and turn_s > 0 and 0 <= t < turn_s + 0.5 and o['walk_turn_speed_mps'] > 0:
                            # turn by stepping slowly forward (0: pivot in place under IDLE with the facing turning)
                            self.streamer.planner(SLOW_WALK, facing, facing, o['walk_turn_speed_mps'], **extra)
                        else:
                            self.streamer.planner(IDLE, [0.0, 0.0, 0.0], facing, -1.0, **extra)
                        latest = self.state.latest()
                        state = arming.read(self.config.root)
                        age = math.inf if latest is None else self.clock() - latest.received
                        if not state['armed']:
                            stop_reason = f"the operator disarmed ({state['reason']}); the robot stops and stands"
                        elif age > o['state_max_age_s']:
                            stop_reason = f'no fresh state from the deploy stack for {age:.2f} s; the robot stops and stands'
                        elif latest.tilt_deg() > o['fall_tilt_deg']:
                            self._fall(latest.tilt_deg())
                            raise MotionStopped(f"tilt {latest.tilt_deg():.0f} deg is over {o['fall_tilt_deg']:g} during the walk: "
                                                'the stack was told to stop and every joint is damped; the session is disarmed')
                        if stop_reason:
                            break
                        trace.append(latest.tilt_deg())
                        time.sleep(dt)
                    if stop_reason:
                        for _ in range(int(o['walk_settle_s'] * scale / dt)):  # stop walking and stand before the hold
                            self.streamer.planner(IDLE, [0.0, 0.0, 0.0], [math.cos(psi), math.sin(psi), 0.0], -1.0, **extra)
                            time.sleep(dt)
                        break
            finally:
                if self.fault is None:
                    self.last_row = None
                    self.plans = {}
                    self.step_id += 1  # the robot moved: observations and plans from before are stale
                    # the planner finishes its last steps before the hold takes over: priming a mid-step stance
                    # (an overshooting turn) left the robot on one leg, and the next walk fell from it
                    if carry:
                        self._upper_latched = [math.cos(psi), math.sin(psi), 0.0]
                    # keep the planner fed while waiting for the robot to settle and, if it is still stepping, until the
                    # next call hands over: without planner messages the deploy dropped a carry's upper body after 1 s
                    # (the held can sank 30 cm), and after any walk a 1 s gap reset it to its reference motion's frame
                    # 0 -- the robot lunged ~0.8 m and fell (planner timeouts in the deploy log, ~1 fall per 10 attempts)
                    last_facing = [math.cos(psi), math.sin(psi), 0.0]
                    self._planner_keepalive = lambda: self.streamer.planner(IDLE, [0.0, 0.0, 0.0], last_facing, -1.0,
                                                                            **extra)
                    self.streamer.keepalive = self._planner_keepalive
                    quiet = self._wait_until_quiet(timeout=o['walk_quiet_timeout_s'], raise_on_timeout=False, strict=True)
                    if quiet is not None:
                        self._hand_over_to_hold(quiet, hands=hands if carry else None)
                    else:
                        self._carry_hands = hands if carry else None
                        self._stepping_since = self.clock()
                        self.converter.clear()  # the planner keeps the robot balanced; the next armed call hands over
                        self._event(f"the robot was still stepping {o['walk_quiet_timeout_s']:g} s after the walk: the planner "
                                    f"keeps it balanced; the next armed call tries up to {o['walk_stuck_recover_s']:g} s to hand over")
            if stop_reason:
                error = MotionStopped(stop_reason)
                error.robot_state = self._state_report()
                raise error
            end = self.state.latest()
            turned = math.degrees(math.atan2(math.sin(end.yaw_rad() - start_yaw), math.cos(end.yaw_rad() - start_yaw)))
            feedback = {'commanded': {'forward_m': forward_m, 'turn_deg': turn_deg, 'speed_mps': speed_mps},
                        'measured_turn_deg': round(turned, 1), 'tilt_deg': round(end.tilt_deg(), 2),
                        'max_tilt_deg': round(max(trace), 2) if trace else None, 'fallen': False,
                        'wall_seconds': round(self.clock() - started, 2),
                        'planned_advance_m': None if advanced is None else round(advanced, 3),
                        'note': 'distance is commanded, not measured: re-localize against the scene'}
            self.last_feedback = feedback
            return self._observation_packet(transition={'kind': 'walk', 'feedback': feedback})

    def _account(self, rows, executed):
        """The rows that reached the wire move the step counter and become the held pose."""
        count = len(executed)
        if count:
            self.step_id += count
            self.last_row = rows[count - 1].copy()
            self.plans = {}
        return count

    def _feedback(self, trace, started):
        if not trace:
            return {'ticks': 0}
        last = trace[-1]
        feedback = {'ticks': len(trace), 'tilt_deg': round(last['tilt_deg'], 2),
                    'max_root_orientation_projection_deg': round(max(t['projection_deg'] for t in trace), 2),
                    'fallen': bool(self.fault and self.fault.get('kind') == 'fall'),
                    'wall_seconds': round(self.clock() - started, 3)}
        error = last['q'] - last['reference']
        feedback['reference_error_rmse_rad'] = float(np.sqrt(np.mean(error ** 2)))
        feedback['max_reference_error_rad'] = float(np.max(np.abs(error)))
        if last['last_action'] is not None:
            error = last['q'] - last['last_action']
            feedback['joint_tracking_rmse_rad'] = float(np.sqrt(np.mean(error ** 2)))
            feedback['max_joint_tracking_error_rad'] = float(np.max(np.abs(error)))
        return feedback

    def _watch_falls(self):
        """Between calls nothing else looks at the robot: a fall while it holds a frame damps and disarms too."""
        while self._watching:
            time.sleep(0.05)
            snap = self.state.latest()
            if snap is None or not self.streamer.primed or self.streamer.executing or self.fault is not None:
                continue
            if self.clock() - snap.received > self.options['state_max_age_s']:
                continue
            tilt = snap.tilt_deg()
            if tilt > self.options['fall_tilt_deg']:
                self.streamer.hold()
                self._fall(tilt)

    def _fall(self, tilt):
        if self.fault is not None and self.fault.get('kind') == 'fall':
            return  # already handled (the step's tick and the watch can both see it)
        if self.options['fall_damp']:
            try:
                self.streamer.command(stop=True)
            except Exception as exc:
                self._event('stop command failed', error=str(exc)[:200])
        self._disarm(f'tilt {tilt:.0f} deg: falling', fault='fall')

    def _console_stop_pending(self):
        """A console STOP written since the command being served (left for the next poll to execute)."""
        try:
            command = json.loads((Path(self.config.root)/'operator'/'stack.json').read_text())
        except (OSError, ValueError):
            return False
        return isinstance(command, dict) and command.get('cmd') == 'stop' and command.get('nonce') not in (None, self._operator_nonce)

    def _console_start(self):
        """START from the console: start the stack's control under the planner (the robot stands in the band), then
        wait, up to console_start_timeout_s, until it is quiet on its feet (the operator lowers the gantry meanwhile;
        a console STOP pressed during the wait is honoured within 2 s), then hand over to the streamed hold the way
        the first armed motion does (_ensure_control). The robot stays under the planner, balancing, if it never
        comes to rest: START again once it stands."""
        o = self.options
        snap = self.state.latest()
        if snap is None or self.clock() - snap.received > o['state_max_age_s']:
            started = self.clock()
            deadline = started + o['start_timeout_s']
            snap = None
            while snap is None and self.clock() < deadline:
                self.streamer.command(start=True, planner=True)
                snap = self.state.wait_after(started, 0.5)
            if snap is None:
                raise RuntimeError(f"no state from the SONIC deploy stack {o['start_timeout_s']:g} s after start: is it running "
                                   'with --input-type zmq_manager --output-type zmq and a planner, past INIT?')
            self._planner_warm = False
            if o.get('console_start_mode', 'planner') == 'stand':
                # out of the planner at once: in streamed mode with nothing streamed yet the stack tracks its
                # pre-loaded reference at frame 0, paused (a standing clip with --motion-data reference/standing/):
                # the policy balances on a static standing target instead of the planner's settling gait
                self.streamer.command(start=True, planner=False)
                self._event('the stack is in control, holding its standing reference (no planner steps): lower the robot '
                            'to the floor; the hold takes over once it is quiet')
            else:
                self._event('the stack is in control under the planner: lower the robot to the floor; the hold takes over once it is quiet')
            if o['planner_settle_s'] > 0:
                time.sleep(o['planner_settle_s'])
        deadline = self.clock() + o['console_start_timeout_s']
        while True:
            self._write_console_status()
            quiet = self._wait_until_quiet(timeout=2.0, raise_on_timeout=False, strict=True)
            if quiet is not None:
                break
            if self._console_stop_pending():
                self._event('console start abandoned: STOP pressed')
                return
            if self.clock() > deadline:
                raise RuntimeError(f"the robot did not come to rest within {o['console_start_timeout_s']:g} s; "
                                   'it stays under the planner (balancing). START again once it stands still')
        self._last_index = quiet.index
        self.streamer.prime(self._within_limits(quiet.q), quiet.base_quat)
        self.streamer.command(start=True, planner=False)
        self.control_mode = 'streamed'
        self._event('streamed motion on; the hold stream starts from the measured pose')
        if o['stand_ramp_s'] > 0:
            self._ease_to_default_stance(quiet.q)

    # ----- operator console --------------------------------------------------------------------
    def poll_operator(self):
        """The operator's stack commands (scripts/g1_console.py writes operator/stack.json: start or stop) and the
        status it reads back (operator/stack_status.json). Called from the frontend's loop, on the tool thread, so a
        command never races a tool call. `start` stands the robot (the stack's control starts the way the first
        armed motion would) without arming: the agent still waits for the operator's arming. `stop` is the stack's
        stop: every joint damped, the session disarmed, control forgotten (the deploy is then relaunched by hand)."""
        root = Path(self.config.root)/'operator'
        try:
            command = json.loads((root/'stack.json').read_text())
        except (OSError, ValueError):
            command = None
        if isinstance(command, dict) and command.get('nonce') and command.get('nonce') != self._operator_nonce:
            self._operator_nonce = command.get('nonce')
            what = command.get('cmd')
            if self.lock.acquire(blocking=False):
                try:
                    if what == 'start':
                        if self.streamer.primed:
                            self._event('console start: the stack is already in control; nothing to do')
                        else:
                            self._event('console start: standing the robot (operator command)')
                            self._check_stack()
                            self._console_start()
                            if self.streamer.primed:
                                self._event('console start done: the robot holds its standing pose')
                    elif what == 'stop':
                        self._event('console STOP: telling the stack to stop (every joint damped)')
                        self.streamer.hold()
                        for _ in range(3):
                            self.streamer.command(stop=True)
                            time.sleep(0.05)
                        self._disarm('console stop: the stack damped every joint')
                        self._reset_control('console stop; relaunch the deploy before the next start')
                    else:
                        self._event(f'console: unknown command {what!r}')
                except Exception as exc:
                    self._event(f'console {what} failed: {str(exc)[:200]}')
                finally:
                    self.lock.release()
        self._write_console_status()

    def _write_console_status(self):
        """operator/stack_status.json for the console, at most 4 times a second."""
        root = Path(self.config.root)/'operator'
        now = self.wall()
        if now - self._operator_status_at >= 0.25:
            self._operator_status_at = now
            snap = self.state.latest()
            state = None
            if snap is not None:
                state = {'tilt_deg': round(snap.tilt_deg(), 1), 'age_s': round(self.clock() - snap.received, 2),
                         'index': int(snap.index)}
            status = {'at': round(now, 2), 'control_mode': self.control_mode, 'primed': self.streamer.primed,
                      'executing': self.streamer.executing, 'armed': arming.read(self.config.root),
                      'fault': self.fault, 'state': state, 'events': self.events[-6:], 'camera_error': self.camera_error}
            try:
                root.mkdir(mode=0o700, parents=True, exist_ok=True)
                tmp = root/'stack_status.tmp'
                tmp.write_text(json.dumps(status, default=str))
                os.replace(tmp, root/'stack_status.json')
            except OSError:
                pass

    def _hold(self):
        dropped = self.streamer.hold()
        report = {'held': True, 'dropped_rows': dropped}
        self._check_stack()
        if self.control_mode in ('idle', 'planner') and arming.read(self.config.root)['armed']:
            try:
                # an armed hold starts the stack's control (the robot stands on its measured pose), or takes a robot
                # the walking planner still balances over, within walk_stuck_recover_s
                self._require_armed()
            except PermissionError as exc:
                report['start_error'] = str(exc)[:300]
        return {**report, **self._state_report()}, []

    def _state_report(self):
        snap = self.state.latest()
        report = {'streaming': {'primed': self.streamer.primed, 'executing': self.streamer.executing,
                                'frames_sent': self.streamer.sent, 'frame_index': self.streamer.frame_index},
                  'control': {'idle': 'no control yet', 'streamed': 'streamed hold',
                              'planner': "SONIC's walking planner (stepping; an armed call hands over)"}.get(
                                  self.control_mode, self.control_mode),
                  'step_id': self.step_id}
        if snap is None:
            report['state'] = 'no state from the deploy stack'
            return report
        report.update({'state_age_s': round(self.clock() - snap.received, 3), 'tilt_deg': round(snap.tilt_deg(), 2),
                       'root_orientation_world_wxyz': np.round(snap.base_quat, 5).tolist(),
                       'joint_positions_rad': np.round(snap.q, 4).tolist(),
                       'pelvis_height_est_m': round(self.pelvis_height_estimate(snap.q, snap.base_quat), 3)})
        return report

    # ----- planning -------------------------------------------------------------------------
    def _held_frame(self, snap):
        """The frame a plan holds: the last row on the wire, or one built from the measured state."""
        if self.last_row is not None:
            return self.last_row.copy()
        frame = np.zeros(R.ACTION_DIM)
        frame[R.ACTION_SLICES['root_height_m']] = 0.75
        frame[R.ROT6D] = self.converter.relative_rot6d(snap.base_quat)
        frame[R.JOINTS] = self._within_limits(snap.q)
        if self.converter.last is not None:
            for index, side in ((38, 'left'), (39, 'right')):  # the closing fraction the held hand target encodes
                frame[index] = self.converter.hand_fraction(side, getattr(self.converter.last, f'{side}_hand'))
        return frame

    def _free_space_move(self, arm, position, quaternion_wxyz=None, frame='pelvis', end_effector='palm',
                         use_waist=False, time_scale=1.0, preview_only=False):
        snap = self._snapshot()
        try:
            from .g1 import api as g1api
        except ImportError as exc:
            raise RuntimeError(f'the arm planner is not importable on this host: {exc}') from None
        measured = dict(zip(R.JOINT_NAMES, snap.q.tolist()))
        hands = {**dict(zip(R.HAND_JOINT_NAMES['left'], snap.left_hand_q.tolist())),
                 **dict(zip(R.HAND_JOINT_NAMES['right'], snap.right_hand_q.tolist()))}
        try:
            result = g1api.plan_arm_motion(arm, position, quaternion_wxyz, current_joints=measured, frame=frame,
                                           use_waist=use_waist, end_effector=end_effector,
                                           control_dt_s=1.0 / self.options['control_hz'], time_scale=time_scale,
                                           hand_joints=hands)
        except Exception as exc:
            raise RuntimeError(f'the arm planner failed or is unavailable on this host (cuRobo on CUDA): {exc}') from None
        summary = {key: result.get(key) for key in (
            'success', 'status', 'joint_names', 'frame', 'end_effector', 'position_only', 'planning_time_s',
            'duration_s', 'position_error_m', 'rotation_error_rad', 'achieved_pose', 'max_abs_velocity_rad_s',
            'limit_violations')}
        if not result.get('success') or not result.get('trajectory_canonical29'):
            return {'planned': False, 'plan': summary, 'executed_rows': 0, **self._state_report()}, []
        base = self._held_frame(snap)
        solved = [R.JOINT_NAMES.index(name) for name in result['joint_names']]
        trajectory = np.asarray(result['trajectory_canonical29'], dtype=float)
        rows = np.repeat(base[None], len(trajectory), axis=0)
        rows[:, R.JOINTS.start + np.asarray(solved)] = trajectory[:, solved]
        tail = self.options['rest_tail_rows']
        rows = np.vstack([rows, np.repeat(rows[-1][None], tail, axis=0)])  # settle: the plan ends in a pose to hold
        plan_id = uuid.uuid4().hex[:16]
        self.plans = {plan_id: {'step_id': self.step_id, 'rows': rows, 'summary': summary}}
        summary.update(motion_plan_id=plan_id, rows=int(len(rows)), seconds=round(len(rows) / self.options['control_hz'], 3),
                       held='other joints, hands and root rotation at the last commanded frame')
        if preview_only:
            return {'planned': True, 'plan': summary, 'executed_rows': 0, **self._state_report()}, []
        self._check_rows(rows, snap)
        return self._step(rows, 'free_space_move', extra={'plan': summary})

    def _execute_plan(self, motion_plan_id):
        plan = self.plans.get(motion_plan_id)
        if plan is None or plan['step_id'] != self.step_id:
            raise RejectedBeforeAction('unknown or stale motion plan; plan again from the current state')
        rows = plan['rows']
        self._check_rows(rows, self._require_armed())
        self.plans = {}
        return self._step(rows, 'free_space_move', extra={'plan': plan['summary']})

    # ----- observation ----------------------------------------------------------------------
    def _camera_pose(self, q):
        override = self.options['camera_pose_override']
        if override:
            return {'frame': 'pelvis', 'position_m': [float(v) for v in override['position_m']],
                    'quaternion_wxyz': [float(v) for v in override['quaternion_wxyz']],
                    'axes': 'camera X forward (optical axis), Y left, Z up'}
        transform = self.model.frame_transform('camera', dict(zip(R.JOINT_NAMES, q.tolist())))
        return {'frame': 'pelvis', 'position_m': np.round(transform[:3, 3], 5).tolist(),
                'quaternion_wxyz': np.round(quat_from_matrix(transform[:3, :3]), 6).tolist(),
                'axes': 'camera X forward (optical axis), Y left, Z up'}

    def _scaled(self, rgb, depth, intrinsics):
        import cv2
        width = self.options['agent_image_width']
        scale = width / rgb.shape[1]
        if scale >= 1.0:
            return rgb, depth, np.asarray(intrinsics, dtype=float), 1.0
        size = (int(width), int(round(rgb.shape[0] * scale)))
        small = cv2.resize(rgb, size, interpolation=cv2.INTER_AREA)
        small_depth = None
        if depth is not None:
            filled = np.where(np.isfinite(depth), depth, 0.0).astype(np.float32)
            small_depth = cv2.resize(filled, size, interpolation=cv2.INTER_NEAREST)
        K = np.asarray(intrinsics, dtype=float).copy()
        K[0, :] *= size[0] / rgb.shape[1]
        K[1, :] *= size[1] / rgb.shape[0]
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
        millimetres = np.zeros(depth.shape, dtype=np.uint16)
        millimetres[valid] = np.rint(np.clip(depth[valid] * 1000.0, 0, 65535)).astype(np.uint16)
        stream = io.BytesIO()
        Image.fromarray(millimetres).save(stream, format='PNG')
        values = depth[valid]
        stats = {'shape': [int(depth.shape[1]), int(depth.shape[0])], 'encoding': 'uint16_png', 'unit': 'millimeters',
                 'invalid': 0, 'valid_fraction': round(float(valid.mean()), 4),
                 'min_m': round(float(values.min()), 3) if values.size else None,
                 'max_m': round(float(values.max()), 3) if values.size else None}
        return stream.getvalue(), stats

    def _observation_packet(self, transition=None):
        snap = self._snapshot()
        content, attachments, depth_info = [], [], {}
        camera = {}
        geometry = None
        frame = None
        if self.camera is not None:
            try:
                frame = self.camera.read()
                # an image from before the last motion ended shows the robot where it was, not where the joints
                # (and the camera pose computed from them) are now: wait briefly for one captured afterwards
                ended = getattr(self, '_motion_ended', None)
                deadline = self.clock() + self.options['camera_max_age_s'] + 0.5
                while ended is not None and frame is not None and frame[0] < ended and self.clock() < deadline:
                    time.sleep(0.05)
                    frame = self.camera.read()
            except Exception as exc:
                camera['error'] = str(exc)[:300]
        else:
            camera['error'] = f'no camera in this session ({self.camera_error or "disabled"})'
        if frame is not None:
            stamp, rgb, depth, intrinsics = frame
            age = self.wall() - stamp
            if age > self.options['camera_max_age_s']:
                camera['error'] = f'the newest camera frame is {age:.1f} s old' + (
                    f' ({self.camera.error})' if getattr(self.camera, 'error', None) else '')
            else:
                small, small_depth, K, scale = self._scaled(rgb, depth if self.depth else None, intrinsics)
                frame_id = time.strftime('%Y%m%d%H%M%S') + uuid.uuid4().hex[:6]
                directory = self.frames_root / frame_id
                directory.mkdir(parents=True, exist_ok=True)
                (directory / 'cam_front.jpg').write_bytes(self._jpeg(rgb, 95))
                content.append({'type': 'image', 'mimeType': 'image/jpeg', 'data': base64.b64encode(self._jpeg(small)).decode()})
                attachments.append({'kind': 'rgb', 'camera': 'cam_front', 'content_index': len(content)})
                pose = self._camera_pose(snap.q)
                meta = {'frame_id': frame_id, 'captured_at': stamp, 'preview_scale': round(scale, 4),
                        'intrinsics_full': np.asarray(intrinsics, dtype=float).tolist(), 'camera_pose_pelvis': pose,
                        'joints': dict(zip(R.JOINT_NAMES, np.round(snap.q, 5).tolist()))}
                if self.depth and small_depth is not None:
                    png, stats = self._depth_png(small_depth)
                    (directory / 'cam_front_depth.png').write_bytes(png)
                    content.append({'type': 'image', 'mimeType': 'image/png', 'data': base64.b64encode(png).decode()})
                    depth_info['cam_front'] = {**stats, 'content_index': len(content), 'file': 'cam_front_depth.png'}
                    attachments.append({'kind': 'depth', 'camera': 'cam_front', 'content_index': len(content)})
                    meta['depth'] = 'cam_front_depth.png (uint16 mm, 0 invalid, at the preview scale)'
                (directory / 'meta.json').write_text(json.dumps(meta, indent=1))
                height, width = small.shape[:2]
                geometry = ({'cam_front': {'matrix': K.tolist(), 'width': int(width), 'height': int(height),
                                           'depth': 'distance along the optical axis (m); 0 = invalid'}},
                            {'cam_front': pose})
                self._raw = (self.step_id, small_depth, geometry[0]['cam_front'], pose, snap.base_quat.tolist())
                camera = {'frame_id': frame_id, 'directory': f'runtime/frames/{frame_id}', 'age_s': round(age, 3),
                          'preview_scale': round(scale, 4), 'width': int(width), 'height': int(height)}
        packet = {
            'step_id': self.step_id, 'robot': R.ROBOT, 'task': self.config.task,
            'proprio': {
                'root_orientation_world_wxyz': np.round(snap.base_quat, 6).tolist(),
                'joint_names_canonical29': list(R.JOINT_NAMES),
                'joint_positions_rad': np.round(snap.q, 5).tolist(),
                'joint_velocities_rad_s': np.round(snap.dq, 4).tolist(),
                'hand_state': {'joint_names': [*R.HAND_JOINT_NAMES['left'], *R.HAND_JOINT_NAMES['right']],
                               'positions_rad': np.round(np.concatenate([snap.left_hand_q, snap.right_hand_q]), 4).tolist(),
                               'velocities_rad_s': None, 'applied_torque_Nm': None}},
            'tilt_deg': round(snap.tilt_deg(), 2),
            'images': [a['camera'] for a in attachments], 'attachments': attachments, 'camera': camera,
            'motion': arming.read(self.config.root),
            'stack': {'state_age_s': round(self.clock() - snap.received, 3), 'index': snap.index},
        }
        if self.depth and depth_info:
            packet['depth'] = depth_info
        if geometry is not None:
            packet['camera_intrinsics'], packet['camera_poses'] = geometry
        if transition is not None:
            packet['transition'] = transition
        return packet, content

    def _pixel_to_position(self, arguments):
        step, depth, intrinsics, pose, root = self._raw
        if depth is None:
            raise ValueError('the current observation has no depth')
        results = []
        for item in arguments['selections']:
            point = T.pixel_to_pelvis(depth, intrinsics, pose, float(item['u']), float(item['v']),
                                      int(arguments.get('depth_window_radius', 1)))
            if point['valid']:
                point['position_level_m'] = T.level_from_pelvis(point['position_pelvis_m'], root).tolist()
            results.append({'u': item['u'], 'v': item['v'], **point})
        return {'observation_step_id': step, 'camera': 'cam_front',
                'frames': {'pelvis': 'robot pelvis', 'level': 'origin at the pelvis, z up, x along the measured heading'},
                'selections': results}

    # ----- status ---------------------------------------------------------------------------
    def _command_report(self):
        conv = self.converter
        report = {'mode': self.control_mode, 'primed': bool(self.streamer.primed),
                  'heading_anchor_wxyz': None if conv.anchor is None else np.round(conv.anchor, 6).tolist(),
                  'last_row': None if self.last_row is None else np.round(self.last_row, 5).tolist(),
                  'hands_fraction': None}
        if conv.last is not None:
            report['hands_fraction'] = [round(float(conv.hand_fraction(side, getattr(conv.last, f'{side}_hand'))), 4)
                                        for side in ('left', 'right')]
        return report

    def status(self):
        o = self.options
        self._check_stack()
        snap = self.state.latest()
        value = {
            'robot': R.ROBOT, 'motion': arming.read(self.config.root),
            'control': "semantic40 reference frames streamed over ZMQ to NVIDIA's SONIC deploy stack "
                       '(frozen encoder and decoder, its own PD loop)',
            'stack': {'state_endpoint': f"tcp://{o['state_host']}:{o['state_port']}", 'pose_endpoint': f"tcp://{o['zmq_host']}:"
                      f"{self.publisher.port if self.publisher else o['pose_port']}",
                      'state_age_s': None if snap is None else round(self.clock() - snap.received, 3),
                      'state_messages': self.state.received, 'frames_sent': self.streamer.sent,
                      'frames_held': self.streamer.held, 'streaming': self.streamer.primed, 'mode': self.control_mode,
                      'executing': self.streamer.executing, 'heading_anchored': self.converter.anchored},
            'action_layout': R.action_layout(), 'max_action_rows': R.MAX_ACTION_ROWS,
            'joint_names_canonical29': list(R.JOINT_NAMES),
            'joint_limits_rad': {'lower': np.round(self.lower, 4).tolist(), 'upper': np.round(self.upper, 4).tolist()},
            'limits': {key: o[key] for key in ('joint_limit_margin_rad', 'max_joint_step_rad', 'max_first_step_rad',
                                               'max_root_delta_deg', 'rest_tail_rows', 'fall_tilt_deg', 'state_max_age_s')},
            'step_id': self.step_id, 'last_feedback': self.last_feedback, 'fault': self.fault, 'events': self.events[-5:],
            'cameras': list(self.cameras), 'depth': self.depth, 'sensors': self._sensor_status(),
            'hand_shapes': self._hand_shapes_report(),
            # what the next streamed call starts from (a controller tracking its own last frame drifts from it after
            # walks and stops): the heading anchor the root rotation is relative to, the last row on the wire, and
            # the closing fraction of each held hand target
            'command': self._command_report(),
        }
        if snap is not None:
            value['proprio'] = {'root_orientation_world_wxyz': np.round(snap.base_quat, 6).tolist(),
                                'joint_positions_rad': np.round(snap.q, 5).tolist(),
                                'hand_positions_rad': {'left': np.round(snap.left_hand_q, 4).tolist(),
                                                       'right': np.round(snap.right_hand_q, 4).tolist()}}
            value['tilt_deg'] = round(snap.tilt_deg(), 2)
            value['proprio']['pelvis_height_est_m'] = round(self.pelvis_height_estimate(snap.q, snap.base_quat), 3)
            if snap.motor_temperature is not None and snap.motor_temperature.size:
                value['motor_temperature_max_c'] = float(snap.motor_temperature.max())
        return value

    def _sensor_status(self):
        if self.camera is None:
            return {'cam_front': f'unavailable ({self.camera_error or "disabled"})'}
        frame = getattr(self.camera, 'frame', None)
        if frame is None:
            return {'cam_front': 'no frames yet' + (f' ({self.camera.error})' if getattr(self.camera, 'error', None) else '')}
        age = self.wall() - frame[0]
        return {'cam_front': 'ok' if age <= self.options['camera_max_age_s'] else f'no recent images ({age:.1f} s)'}

    def close(self):
        self._watching = False
        try:
            if self.control_mode == 'planner' and self.fault is None:
                # without planner messages the deploy times its planner out after 1 s and jumps to its reference
                # motion's frame 0: the robot lunged and fell when a session ended here (sim)
                try:
                    self._recover_from_stepping()
                except Exception as exc:
                    self._event(f'the robot stays with the planner at exit: {str(exc)[:160]}')
                    print('WARNING: the robot service exits while SONIC\'s planner still balances a stepping robot; the deploy '
                          'times its planner out 1 s from now and jumps to its reference motion (the robot may lunge): band on, '
                          'or STOP the stack', file=sys.stderr, flush=True)
            self.streamer.hold()
        finally:
            self.streamer.stop()
            if self.publisher is not None:
                self.publisher.close()
            self.state.close()
            if self.camera is not None:
                try:
                    self.camera.close()
                except Exception:
                    pass
