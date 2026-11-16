"""MCP backend for a team of robots under one agent: it holds one backend per robot and adds what a team needs.

Each robot keeps its own backend, tools, limits and guards unchanged (`g1_*` for the G1, `r1_*` for the R1), so a
call for one robot can never be checked against the other robot's body. On top of them this backend gives:

  - one command thread per robot. A robot runs one call at a time, as before; different robots run at the same
    time. `team_parallel` starts one call per robot together and waits; `team_start` returns at once and
    `team_wait` collects, so the agent can work with one robot while another walks.
  - validation of a whole group before anything moves: if one call of a group is refused, none starts.
  - one arming for the session. The operator arms and disarms every robot together, and a fault that disarms one
    robot (a fall, a runaway) stops the others with it.
  - `team_stop`, and the same stop when a call of a `stop_all_on_failure` group fails: every running motion stops
    within one tick and holds (services.arming holds), without disarming.
  - the team frame (services/team/frame.py): where each robot stands, kept by dead reckoning from the base motions
    the robots report, and a separation check that refuses a walk whose path comes too close to another robot.

What it does not do: it does not close a control loop across robots. Two calls of a group start together and then
run on their own clocks; nothing here makes one robot follow the other's motion or the force on a shared object. A
physically coupled skill (a shared carry) needs that loop in one process at control rate, built on this backend's
threads and stop; until it exists, coupled groups are quasi-static steps that start together.
"""
from __future__ import annotations

import dataclasses
import queue
import threading
import time
import types
import uuid

from services import arming
from services.controller.errors import public_failure

from . import tools as T
from .frame import TeamFrame
from .members import KINDS
from .tools import Refused

TEAM_DEFAULTS = {
    'start_poses': {},            # robot -> [x_m, y_m, yaw_deg] in the team frame, measured by the operator
    'start_uncertainty_m': 0.05,
    'radius_m': {},               # robot -> body radius in the plane (default: the robot kind's)
    'min_gap_m': 0.3,             # a base motion that leaves less body-to-body gap than this is refused
    'drift_per_m': 0.1,           # uncertainty a robot's entry gains per metre it walks
    'separation': True,           # False turns the separation check off (the team frame is still kept)
    'require_poses': False,       # True: a base motion is refused while a robot's pose is unknown
    'observation_profiles': {},   # robot -> 'rgbd' | 'rgb-only' (default: the session's)
    'stop_wait_s': 8.0,           # team_stop waits this long for the running calls to end
}
HOLD_KEY = 'team-stop'
KEPT_JOBS = 16


class Job:
    """One call on one robot, on that robot's command thread."""

    def __init__(self, robot, tool, arguments, run, *, group=None, motion=None, visible=True):
        self.id = uuid.uuid4().hex[:8]
        self.robot, self.tool, self.arguments, self.run = robot, tool, arguments, run
        self.group, self.motion, self.visible = group, motion, visible
        self.started = self.ended = None
        self.value, self.images, self.exception = None, [], None
        self.stopped = None           # why the call ended early without raising, when it did
        self.delivered = False
        self.done = threading.Event()

    @property
    def failed(self):
        return self.exception is not None or self.stopped is not None


class Robot:
    def __init__(self, name, kind, backend):
        self.name, self.kind, self.backend = name, kind, backend
        self.tools = {tool['name']: tool for tool in backend.tools()}
        self.current = None           # the job queued or running on this robot
        self.queue = queue.Queue()
        self.thread = threading.Thread(target=self._loop, name=f'team-{name}', daemon=True)
        self.thread.start()

    def _loop(self):
        while True:
            job = self.queue.get()
            if job is None:
                return
            job.started = time.monotonic()
            try:
                job.value, job.images = job.run()
            except BaseException as exc:  # reported whole to the caller; the thread serves the next call
                job.exception = exc
            job.ended = time.monotonic()
            try:
                job.finish(job)
            finally:
                job.done.set()


def _early_stop(value):
    """Why a call that returned normally still ended early (a robot's own stop inside a motion), or None."""
    if not isinstance(value, dict):
        return None
    if value.get('stopped'):
        return str(value['stopped'])[:300]
    feedback = (value.get('transition') or {}).get('feedback') if isinstance(value.get('transition'), dict) else None
    if value.get('fallen') is True or (isinstance(feedback, dict) and feedback.get('fallen') is True):
        return 'the robot fell'
    return None


class TeamBackend:
    """A team of robots. Subclasses fix MEMBERS (robot name -> services.backends key) and NAME."""
    NAME = 'team'
    MEMBERS = {}

    def __init__(self, config, *, members=None, clock=time.monotonic, wall=time.time):
        self.config, self.clock, self.wall = config, clock, wall
        names = list(members) if members is not None else list(self.MEMBERS)
        if len(names) < 2:
            raise ValueError('a team needs at least two robots')
        unknown = set(config.backend_options) - {*names, 'team'}
        if unknown:
            raise ValueError(f"unknown {self.NAME} backend options: {sorted(unknown)} (one object of options per robot "
                             f"{names}, and 'team')")
        team = config.backend_options.get('team', {})
        unknown = set(team) - set(TEAM_DEFAULTS)
        if unknown:
            raise ValueError(f'unknown team options: {sorted(unknown)}')
        self.options = o = {**TEAM_DEFAULTS, **team}
        for key in ('start_poses', 'radius_m', 'observation_profiles'):
            extra = set(o[key]) - set(names)
            if extra:
                raise ValueError(f'team option {key} names robots the team does not have: {sorted(extra)}')
        self.robots = {}
        self._lock = threading.RLock()
        self._ended = threading.Condition(self._lock)
        self._jobs = {}               # id -> Job, the visible ones (running and the last KEPT_JOBS ended)
        self._stop_reason = None      # set while the team holds every motion (team_stop, a failed group)
        self.events = []
        try:
            for name in names:
                if members is not None:
                    kind, backend = members[name]
                else:
                    from services.backends import backend_class
                    key = self.MEMBERS[name]
                    kind = KINDS[key]
                    backend = backend_class(key)(self._member_config(key, config.backend_options.get(name, {}),
                                                                     o['observation_profiles'].get(name)))
                self.robots[name] = Robot(name, kind, backend)
        except BaseException:
            self.close()
            raise
        owners = {}
        for robot in self.robots.values():
            for tool in robot.tools:
                if tool in owners or tool in T.TEAM_TOOLS:
                    raise ValueError(f'tool name {tool!r} is used twice in the team; robot tools need their own prefix')
                owners[tool] = robot
        self._owner = owners
        self.mutating = frozenset().union(T.MUTATING, *(getattr(r.backend, 'mutating', ()) for r in self.robots.values()))
        self.frame = TeamFrame({name: float(o['radius_m'].get(name, robot.kind.radius_m))
                                for name, robot in self.robots.items()},
                               min_gap_m=o['min_gap_m'], drift_per_m=o['drift_per_m'], wall=wall)
        for name, pose in o['start_poses'].items():
            if not (isinstance(pose, (list, tuple)) and len(pose) == 3):
                raise ValueError(f'start_poses[{name!r}] must be [x_m, y_m, yaw_deg]')
            self.frame.set(name, *pose, uncertainty_m=o['start_uncertainty_m'], source='operator (start pose)')

    def _member_config(self, key, options, profile):
        changes = {'robot': key, 'backend_options': dict(options),
                   'observation_profile': profile or self.config.observation_profile}
        if dataclasses.is_dataclass(self.config):
            return dataclasses.replace(self.config, **changes)
        return types.SimpleNamespace(**{**vars(self.config), **changes})

    # ----- tools ----------------------------------------------------------------------------------------------------
    def tools(self):
        names = list(self.robots)
        example = tuple(robot.kind.observe for robot in list(self.robots.values())[:2])
        return [*T.tool_definitions(names, example), *(tool for robot in self.robots.values() for tool in robot.tools.values())]

    def validate(self, name, arguments):
        if name in T.TEAM_TOOLS:
            T.validate_request(name, arguments, list(self.robots))
            if name in (T.START, T.PARALLEL):
                self._plan(arguments)
            return
        robot = self._owner.get(name)
        if robot is None:
            raise Refused(f'Unknown tool: {name}')
        self._require_idle(robot)
        self._validate_member(robot, name, arguments)
        self._check_separation({robot.name: robot.kind.base_motion(name, arguments)}, ())

    def all_lanes(self):
        return tuple(self.robots)

    def lanes(self, name):
        """The robots a tool call occupies (the service runs one call per robot at a time): a member's tool its robot,
        every team tool all of them."""
        robot = self._owner.get(name)
        return (robot.name,) if robot is not None else tuple(self.robots)

    def call(self, name, arguments):
        if name == T.STATUS:
            return self.status(), []
        if name == T.OBSERVE:
            return self._observe()
        if name == T.SET_POSE:
            return self._set_pose(arguments), []
        if name == T.STOP:
            return self._stop(arguments.get('reason') or 'team_stop'), []
        if name == T.WAIT:
            return self._wait(arguments.get('jobs') or None, float(arguments.get('timeout_s', 60)))
        if name in (T.START, T.PARALLEL):
            jobs = self._start(arguments)
            if name == T.START:
                with self._lock:
                    return {'started': [self._brief(job) for job in jobs], 'team_frame': self.frame.report()}, []
            return self._wait([job.id for job in jobs], None)
        robot = self._owner.get(name)
        if robot is None:
            raise ValueError(f'unknown tool {name}')
        motion = robot.kind.base_motion(name, arguments)
        job = self._submit(robot, name, arguments, motion=motion, visible=False)
        job.done.wait()
        if job.exception is not None:
            raise job.exception
        value = job.value
        if motion is not None and isinstance(value, dict):
            with self._lock:
                value = {**value, 'team_frame': self.frame.report()}
        return value, job.images

    # ----- one robot ------------------------------------------------------------------------------------------------
    def _require_idle(self, robot):
        with self._lock:
            job = robot.current
            if job is not None:
                raise Refused(f'{robot.name} is running {job.tool} (job {job.id}): wait for it (team_wait) or stop it '
                              '(team_stop) before the next call for this robot')

    def _on_thread(self, robot, run):
        """Run something on a robot's command thread and wait for it (the robot must be idle)."""
        job = self._submit(robot, 'internal', {}, run=run, visible=False)
        job.done.wait()
        if job.exception is not None:
            raise job.exception
        return job.value

    def _validate_member(self, robot, tool, arguments):
        validate = getattr(robot.backend, 'validate', None)
        if validate is not None:
            self._on_thread(robot, lambda: (validate(tool, arguments), []))

    def _submit(self, robot, tool, arguments, *, run=None, group=None, motion=None, visible=True, barrier=None):
        backend = robot.backend
        if run is None:
            def run():
                if barrier is not None:
                    barrier.wait(timeout=10.0)   # the calls of a group start together
                return backend.call(tool, arguments)
        job = Job(robot, tool, arguments, run, group=group, motion=motion, visible=visible)
        job.finish = self._finish
        with self._lock:
            if robot.current is not None:
                raise Refused(f'{robot.name} is running {robot.current.tool}')
            robot.current = job
            if visible:
                self._jobs[job.id] = job
        robot.queue.put(job)
        return job

    def _finish(self, job):
        """On the robot's thread, as its call ends: the team frame, a failed group's stop, the end of a team hold."""
        robot = job.robot
        if job.exception is None and job.tool != 'internal':
            job.stopped = _early_stop(job.value)
        with self._lock:
            if job.motion is not None:
                moved = None
                if job.exception is None:
                    try:
                        moved = robot.kind.base_result(job.tool, job.arguments, job.value)
                    except Exception:
                        moved = None
                if moved is not None:
                    forward, left, turn, measured, extra = moved
                    self.frame.advance(robot.name, forward, left, turn, measured=measured, extra_uncertainty_m=extra)
                else:   # the motion ended somewhere inside what it commanded
                    self.frame.blur(robot.name, abs(job.motion[0]) + abs(job.motion[1]))
            group = job.group
            if job.failed and group is not None and group['stop_all_on_failure'] and self._stop_reason is None:
                others = [j for j in group['jobs'] if j is not job and j.ended is None]
                if others:
                    why = str(job.exception)[:200] if job.exception is not None else job.stopped
                    self._hold(f"{robot.name}'s {job.tool} failed in a group that stops together ({why})")
            robot.current = None
            self._release_if_idle()
            if job.visible:
                ended = [j for j in self._jobs.values() if j.ended is not None]
                for old in sorted(ended, key=lambda j: j.ended)[:-KEPT_JOBS]:
                    self._jobs.pop(old.id, None)
            self._ended.notify_all()

    # ----- stopping -------------------------------------------------------------------------------------------------
    def _event(self, what):
        self.events = (self.events + [{'at': round(self.wall(), 2), 'what': what}])[-20:]

    def _hold(self, reason):
        self._stop_reason = reason
        arming.hold(self.config.root, HOLD_KEY, reason)
        self._event('team stop: ' + reason)

    def _release_if_idle(self):
        """End a team hold once nothing runs, and pass on a fault a robot raised while the hold hid its disarm."""
        if self._stop_reason is None or any(robot.current is not None for robot in self.robots.values()):
            return
        self._stop_reason = None
        arming.release(self.config.root, HOLD_KEY)
        self._event('team stop ended: every robot holds')
        state = arming.read(self.config.root, holds=False)
        for robot in self.robots.values():
            fault, at = getattr(robot.backend, 'fault', None), getattr(robot.backend, 'fault_disarmed_at', None)
            if fault and at is not None and state['armed'] and (state.get('since') or 0) <= at:
                reason = f"{robot.name}: {fault.get('what', 'fault') if isinstance(fault, dict) else fault}"
                arming.write(self.config.root, False, by=f'{self.NAME}-backend', reason=reason)
                self._event('disarmed: ' + reason)
                return

    def _stop(self, reason):
        with self._lock:
            running = [robot.current for robot in self.robots.values() if robot.current is not None]
            if not running:
                return {'stopped': [], 'note': 'nothing was running', 'team_frame': self.frame.report()}
            if self._stop_reason is None:
                self._hold(reason)
            deadline = self.clock() + self.options['stop_wait_s']
            while any(job.ended is None for job in running) and self.clock() < deadline:
                self._ended.wait(timeout=0.1)
            still = [job for job in running if job.ended is None]
            report = {'stopped': [self._brief(job) for job in running if job.ended is not None],
                      'still_running': [self._brief(job) for job in still], 'team_frame': self.frame.report()}
            if still:
                report['note'] = ('a call that runs inside its robot\'s own stack does not stop until it ends; no robot '
                                  'starts a motion until it has (team_wait)')
            return report

    # ----- groups ---------------------------------------------------------------------------------------------------
    def _plan(self, arguments):
        """Check a group's calls and return [(robot, tool, arguments, base motion)]. Nothing moves."""
        planned, seen = [], set()
        for call in arguments['calls']:
            tool, args = call['tool'], call.get('arguments', {})
            robot = self._owner.get(tool)
            if robot is None:
                raise Refused(f"{tool!r} is not a robot tool of this team; a group takes each robot's own tools "
                              f"({', '.join(sorted(self.robots))})")
            if robot.name in seen:
                raise Refused(f'two calls for {robot.name} in one group: a robot runs one call at a time')
            seen.add(robot.name)
            self._require_idle(robot)
            planned.append((robot, tool, args, robot.kind.base_motion(tool, args)))
        for robot, tool, args, _ in planned:
            try:
                self._validate_member(robot, tool, args)
            except (ValueError, PermissionError) as exc:
                raise type(exc)(f'{tool}: {exc} (no call of the group was started)') from None
        coupled = bool(arguments.get('coupled', False))
        names = [robot.name for robot, *_ in planned]
        close = {frozenset((a, b)) for a in names for b in names if a != b} if coupled else set()
        self._check_separation({robot.name: motion for robot, _, _, motion in planned}, close)
        return planned, coupled, bool(arguments.get('stop_all_on_failure', coupled)), close

    def _start(self, arguments):
        planned, coupled, stop_all, close = self._plan(arguments)
        group = {'id': uuid.uuid4().hex[:6], 'coupled': coupled, 'stop_all_on_failure': stop_all, 'close': close, 'jobs': []}
        barrier = threading.Barrier(len(planned)) if len(planned) > 1 else None
        with self._lock:
            for robot, tool, args, motion in planned:
                group['jobs'].append(self._submit(robot, tool, args, group=group, motion=motion, barrier=barrier))
        self._event(f"group {group['id']} started: " + ', '.join(job.tool for job in group['jobs'])
                    + (' (coupled)' if coupled else ''))
        return group['jobs']

    def _check_separation(self, motions, close):
        """Refuse base motions whose paths bring two robots' bodies too close, from the team frame."""
        motions = {name: motion for name, motion in motions.items() if motion is not None}
        if not motions or not self.options['separation']:
            return
        with self._lock:
            for robot in self.robots.values():   # robots already walking keep their paths; coupled groups stay allowed
                job = robot.current
                if job is not None and job.visible:
                    if job.motion is not None:
                        motions.setdefault(robot.name, job.motion)
                    if job.group is not None:
                        close = set(close) | job.group['close']
            conflicts, unknown = self.frame.conflicts(motions, close)
        if conflicts:
            raise Refused('this base motion would bring robots too close: '
                          + '; '.join(f"{c['robots'][0]} and {c['robots'][1]} would be {c['gap_m']:g} m apart (bodies, after "
                                      f"their uncertainties; the limit is {c['min_gap_m']:g} m)" for c in conflicts)
                          + '. If the team frame is wrong, correct it from a measurement (team_set_pose); if the robots '
                            'are meant to work within reach of each other, run the step as a coupled group')
        if unknown and self.options['require_poses']:
            raise Refused('the team frame has no pose for ' + ', '.join(sorted({n for pair in unknown for n in pair
                                                                               if n not in self.frame.poses}))
                          + ': give it (team_set_pose) before a base motion')

    # ----- waiting and reports --------------------------------------------------------------------------------------
    def _brief(self, job):
        state = 'running' if job.ended is None else ('failed' if job.failed else 'done')
        seconds = None if job.started is None else round((job.ended or time.monotonic()) - job.started, 2)
        return {'job': job.id, 'robot': job.robot.name, 'tool': job.tool, 'state': state, 'seconds': seconds,
                **({'group': job.group['id']} if job.group else {})}

    def _wait(self, ids, timeout_s):
        with self._lock:
            if ids is None:
                jobs = [job for job in self._jobs.values() if job.ended is None or not job.delivered]
            else:
                missing = [i for i in ids if i not in self._jobs]
                if missing:
                    raise Refused(f'unknown or expired job ids: {missing}')
                jobs = [self._jobs[i] for i in ids]
            deadline = None if timeout_s is None else self.clock() + timeout_s
            while any(job.ended is None for job in jobs) and (deadline is None or self.clock() < deadline):
                self._ended.wait(timeout=0.1)
            reports, images = [], []
            for job in jobs:
                report = self._brief(job)
                if job.ended is not None and not job.delivered:
                    job.delivered = True
                    if job.exception is not None:
                        handled = isinstance(job.exception, (ValueError, PermissionError)) or getattr(
                            job.exception, 'handled_stop', False)
                        report['error'] = public_failure(job.exception, operation=job.tool, stage='robot_execution',
                                                         traceback_tail=not handled)
                    else:
                        report['result'] = job.value
                        if job.stopped is not None:
                            report['ended_early'] = job.stopped
                        if job.images:
                            report['images'] = len(job.images)
                            images.extend(job.images)
                reports.append(report)
            value = {'done': all(job.ended is not None for job in jobs), 'jobs': reports,
                     'team_frame': self.frame.report()}
            if images:
                value['image_order'] = [f"{r['robot']}:{r['tool']}" for r in reports for _ in range(r.get('images', 0))]
            if self._stop_reason is not None:
                value['team_stop'] = self._stop_reason
            return value, images

    def _observe(self):
        members, images, order = {}, [], []
        for name, robot in self.robots.items():
            with self._lock:
                job = robot.current
            if job is not None:
                members[name] = {'busy': self._brief(job)}
                continue
            try:
                value, shots = self._on_thread(robot, lambda robot=robot: ((robot.backend.call(robot.kind.observe, {})), []))
            except Exception as exc:
                members[name] = {'error': public_failure(exc, operation=robot.kind.observe, stage='robot_execution')}
                continue
            members[name] = value
            images.extend(shots)
            order.extend([name] * len(shots))
        with self._lock:
            return {'robots': members, 'image_order': order, 'team_frame': self.frame.report()}, images

    def _set_pose(self, arguments):
        with self._lock:
            self.frame.set(arguments['robot'], arguments['x_m'], arguments['y_m'], arguments['yaw_deg'],
                           uncertainty_m=arguments.get('uncertainty_m', 0.05),
                           source='agent: ' + (arguments.get('source') or 'measured'))
            self._event(f"team frame: {arguments['robot']} set to ({arguments['x_m']:g}, {arguments['y_m']:g}) m, "
                        f"{arguments['yaw_deg']:g} deg")
            return {'team_frame': self.frame.report()}

    def status(self):
        with self._lock:
            robots = {}
            for name, robot in self.robots.items():
                job = robot.current
                robots[name] = {'kind': robot.kind.backend, 'status_tool': robot.kind.status,
                                'running': None if job is None or not job.visible else self._brief(job),
                                'fault': getattr(robot.backend, 'fault', None),
                                'events': list(getattr(robot.backend, 'events', []))[-3:]}
            return {'robot': self.NAME, 'motion': arming.read(self.config.root), 'robots': robots,
                    'team_frame': self.frame.report(),
                    'jobs': [self._brief(job) for job in self._jobs.values() if job.ended is None or not job.delivered],
                    'team_stop': self._stop_reason, 'events': self.events[-5:]}

    # ----- operator and shutdown ------------------------------------------------------------------------------------
    def poll_operator(self):
        """Each robot's own operator console (the G1's start and stop), on that robot's thread, while it is idle; a
        robot that is running a call takes its console command when the call has ended, as a single robot does."""
        for robot in self.robots.values():
            poll = getattr(robot.backend, 'poll_operator', None)
            if poll is None:
                continue
            with self._lock:
                if robot.current is not None:
                    continue
            self._on_thread(robot, lambda poll=poll: (poll(), []))

    def close(self):
        robots = list(getattr(self, 'robots', {}).values())
        with self._lock:
            if any(robot.current is not None for robot in robots) and self._stop_reason is None:
                self._hold('the robot service is closing')
            deadline = self.clock() + self.options['stop_wait_s']
            while any(robot.current is not None for robot in robots) and self.clock() < deadline:
                self._ended.wait(timeout=0.1)
        arming.release(self.config.root, HOLD_KEY)
        for robot in robots:
            robot.queue.put(None)
            robot.thread.join(timeout=2.0)
        for robot in robots:
            try:
                robot.backend.close()
            except Exception as exc:
                print(f'closing {robot.name} failed: {exc}', flush=True)


class G1R1Backend(TeamBackend):
    """The Unitree G1 (services/g1sonic) and the Unitree R1 (services/r1sonic) as one team."""
    NAME = 'g1r1'
    MEMBERS = {'g1': 'g1sonic', 'r1': 'r1sonic'}
