"""The team backend with two hardware-free robots: routing, calls on both at once, the shared stop, the team frame."""
import json
import threading
import time

import pytest

from conftest import configuration
from services import arming
from services.team.backend import G1R1Backend, TeamBackend
from services.team.frame import TeamFrame
from services.team.members import KINDS, Kind
from services.team.tools import Refused


class Stopped(RuntimeError):
    handled_stop = True


class FakeRobot:
    """A robot whose motions poll arming every tick, the way the real backends do."""

    def __init__(self, prefix, root):
        self.p, self.root = prefix, root
        self.calls, self.fault, self.fault_disarmed_at, self.events, self.closed = [], None, None, [], False
        self.mutating = frozenset({f'{prefix}_move', f'{prefix}_walk'})

    def tools(self):
        return [{'name': f'{self.p}_{name}', 'description': name,
                 'inputSchema': {'type': 'object', 'properties': {}, 'additionalProperties': True}}
                for name in ('status', 'observe', 'hold', 'move', 'walk', 'fail', 'soft', 'block')]

    def validate(self, name, arguments):
        if name.endswith(('_move', '_walk', '_fail', '_soft', '_block')) and not arming.read(self.root)['armed']:
            raise PermissionError('motion is not armed')
        if abs(arguments.get('forward_m', 0.0)) > 3.0:
            raise ValueError('forward_m must be in [-3, 3]')

    def call(self, name, arguments):
        self.calls.append((name, threading.current_thread().name, time.monotonic()))
        kind = name.split('_', 1)[1]
        if kind == 'observe':
            return {'robot': self.p}, [{'type': 'image', 'mimeType': 'image/jpeg', 'data': self.p}]
        if kind == 'move':
            deadline = time.monotonic() + arguments.get('secs', 0.2)
            while time.monotonic() < deadline:
                if not arming.read(self.root)['armed']:
                    if arguments.get('fault_on_stop'):
                        self.fault, self.fault_disarmed_at = {'what': 'fell while stopping'}, time.time()
                    raise Stopped('disarmed during the motion: held where it was')
                time.sleep(0.01)
            return {'moved': True}, []
        if kind == 'walk':
            return {'walked': arguments.get('forward_m', 0.0), 'turned': arguments.get('turn_deg', 0.0)}, []
        if kind == 'fail':
            time.sleep(arguments.get('after', 0.05))
            raise RuntimeError('the stack refused the motion')
        if kind == 'soft':
            time.sleep(arguments.get('after', 0.05))
            return {'stopped': 'the robot took 3 steps during the motion: held'}, []
        if kind == 'block':
            time.sleep(arguments.get('secs', 0.4))   # inside the robot's own stack: it does not look at arming
            return {'blocked': True}, []
        return {'robot': self.p, 'ok': True}, []

    def status(self):
        return {'robot': self.p}

    def close(self):
        self.closed = True


def kind(prefix):
    def motion(tool, arguments):
        return (arguments.get('forward_m', 0.0), 0.0, arguments.get('turn_deg', 0.0)) if tool == f'{prefix}_walk' else None

    def result(tool, arguments, value):
        return (value['walked'], 0.0, value['turned'], True, 0.0) if tool == f'{prefix}_walk' else None
    return Kind('fake', f'{prefix}_status', f'{prefix}_observe', f'{prefix}_hold', 0.3, motion, result)


@pytest.fixture
def team(tmp_path):
    made = []

    def make(**options):
        config = configuration(tmp_path, backend_options={'team': options} if options else {})
        arming.write(config.root, True, by='test')
        a, b = FakeRobot('a', config.root), FakeRobot('b', config.root)
        backend = TeamBackend(config, members={'a': (kind('a'), a), 'b': (kind('b'), b)})
        made.append(backend)
        return backend, a, b
    yield make
    for backend in made:
        backend.close()


def run(backend, name, arguments=None):
    arguments = arguments or {}
    backend.validate(name, arguments)
    return backend.call(name, arguments)


def test_the_team_lists_its_own_tools_and_every_robots_tools(team):
    backend, a, b = team()
    names = [tool['name'] for tool in backend.tools()]
    assert names[:7] == ['team_status', 'team_observe', 'team_parallel', 'team_start', 'team_wait', 'team_stop',
                         'team_set_pose']
    assert {'a_move', 'b_move', 'a_status', 'b_walk'} <= set(names)
    assert {'a_move', 'b_walk', 'team_parallel', 'team_start'} <= backend.mutating


def test_two_robots_may_not_share_a_tool_name(tmp_path):
    config = configuration(tmp_path)
    with pytest.raises(ValueError, match='used twice'):
        TeamBackend(config, members={'a': (kind('a'), FakeRobot('a', config.root)),
                                     'b': (kind('a'), FakeRobot('a', config.root))})


def test_a_robots_own_tool_runs_on_that_robots_thread(team):
    backend, a, b = team()
    value, _ = run(backend, 'a_move', {'secs': 0.02})
    assert value == {'moved': True}
    assert [(name, thread) for name, thread, _ in a.calls] == [('a_move', 'team-a')] and b.calls == []
    run(backend, 'b_status')
    assert b.calls[0][:2] == ('b_status', 'team-b')


def test_parallel_calls_start_together_and_run_at_the_same_time(team):
    backend, a, b = team()
    started = time.monotonic()
    value, images = run(backend, 'team_parallel', {'calls': [{'tool': 'a_move', 'arguments': {'secs': 0.3}},
                                                               {'tool': 'b_move', 'arguments': {'secs': 0.3}}]})
    assert time.monotonic() - started < 0.55                       # not 0.6: they overlapped
    assert abs(a.calls[0][2] - b.calls[0][2]) < 0.05               # and started together
    assert value['done'] and [(j['robot'], j['state']) for j in value['jobs']] == [('a', 'done'), ('b', 'done')]
    assert all(job['result'] == {'moved': True} for job in value['jobs'])


def test_a_group_is_refused_whole_before_anything_moves(team):
    backend, a, b = team()
    with pytest.raises(ValueError, match='b_walk: forward_m must be in .*no call of the group was started'):
        run(backend, 'team_parallel', {'calls': [{'tool': 'a_move', 'arguments': {}},
                                                 {'tool': 'b_walk', 'arguments': {'forward_m': 9.0}}]})
    assert a.calls == [] and b.calls == []
    with pytest.raises(Refused, match='two calls for a'):
        run(backend, 'team_parallel', {'calls': [{'tool': 'a_move'}, {'tool': 'a_hold'}]})
    with pytest.raises(Refused, match='not a robot tool'):
        run(backend, 'team_parallel', {'calls': [{'tool': 'team_stop'}]})


def test_a_started_call_keeps_its_robot_busy_and_leaves_the_other_free(team):
    backend, a, b = team()
    value, _ = run(backend, 'team_start', {'calls': [{'tool': 'a_move', 'arguments': {'secs': 0.4}}]})
    job = value['started'][0]
    assert job['robot'] == 'a' and job['state'] == 'running'
    with pytest.raises(Refused, match='a is running a_move'):
        run(backend, 'a_status')
    assert run(backend, 'b_move', {'secs': 0.02})[0] == {'moved': True}      # the other robot works meanwhile
    assert backend.status()['robots']['a']['running']['job'] == job['job']
    observed, images = run(backend, 'team_observe')
    assert 'busy' in observed['robots']['a'] and observed['robots']['b'] == {'robot': 'b'}
    assert observed['image_order'] == ['b'] and [i['data'] for i in images] == ['b']
    waited, _ = run(backend, 'team_wait', {'jobs': [job['job']], 'timeout_s': 0.05})
    assert not waited['done'] and waited['jobs'][0]['state'] == 'running'
    waited, _ = run(backend, 'team_wait', {'timeout_s': 5})
    assert waited['done'] and waited['jobs'][0]['result'] == {'moved': True}
    again, _ = run(backend, 'team_wait', {'jobs': [job['job']]})
    assert 'result' not in again['jobs'][0] and again['jobs'][0]['state'] == 'done'   # a result is returned once
    assert run(backend, 'a_status')[0]['ok']


def test_team_stop_stops_running_motions_and_leaves_the_session_armed(team):
    backend, a, b = team()
    run(backend, 'team_start', {'calls': [{'tool': 'a_move', 'arguments': {'secs': 5}},
                                          {'tool': 'b_move', 'arguments': {'secs': 5}}]})
    time.sleep(0.05)
    started = time.monotonic()
    value, _ = run(backend, 'team_stop', {'reason': 'the box slipped'})
    assert time.monotonic() - started < 1.0
    assert sorted(j['robot'] for j in value['stopped']) == ['a', 'b'] and value['still_running'] == []
    waited, _ = run(backend, 'team_wait')
    assert all(j['state'] == 'failed' and 'held where it was' in j['error']['reason'] for j in waited['jobs'])
    assert arming.read(backend.config.root)['armed'] and arming.held(backend.config.root) == {}
    assert run(backend, 'a_move', {'secs': 0.02})[0] == {'moved': True}      # and motion works again


def test_a_call_inside_a_robots_own_stack_ends_first(team):
    backend, a, b = team(stop_wait_s=0.1)
    run(backend, 'team_start', {'calls': [{'tool': 'a_block', 'arguments': {'secs': 0.5}}]})
    time.sleep(0.05)
    value, _ = run(backend, 'team_stop')
    assert [j['tool'] for j in value['still_running']] == ['a_block'] and 'does not stop until it ends' in value['note']
    with pytest.raises(PermissionError):
        run(backend, 'b_move')                     # nothing starts while the stop still waits for it
    waited, _ = run(backend, 'team_wait', {'timeout_s': 5})
    assert waited['jobs'][0]['state'] == 'done' and arming.read(backend.config.root)['armed']
    assert run(backend, 'b_move', {'secs': 0.02})[0] == {'moved': True}


@pytest.mark.parametrize('failing', ['a_fail', 'a_soft'])
def test_a_failed_call_stops_its_group_when_asked(team, failing):
    backend, a, b = team()
    calls = [{'tool': failing, 'arguments': {}}, {'tool': 'b_move', 'arguments': {'secs': 5}}]
    started = time.monotonic()
    value, _ = run(backend, 'team_parallel', {'calls': calls, 'coupled': True})
    assert time.monotonic() - started < 1.5
    by_robot = {job['robot']: job for job in value['jobs']}
    assert by_robot['b']['state'] == 'failed' and 'held where it was' in by_robot['b']['error']['reason']
    assert by_robot['a']['state'] == 'failed'
    if failing == 'a_soft':
        assert 'took 3 steps' in by_robot['a']['ended_early']
    assert arming.read(backend.config.root)['armed']
    value, _ = run(backend, 'team_parallel', {'calls': [{'tool': failing}, {'tool': 'b_move', 'arguments': {'secs': 0.3}}]})
    assert {job['robot']: job['state'] for job in value['jobs']} == {'a': 'failed', 'b': 'done'}   # not asked: b finishes


def test_a_fault_raised_during_a_team_stop_disarms_the_session(team):
    backend, a, b = team()
    run(backend, 'team_start', {'calls': [{'tool': 'a_move', 'arguments': {'secs': 5, 'fault_on_stop': True}}]})
    time.sleep(0.05)
    run(backend, 'team_stop')
    state = arming.read(backend.config.root)
    assert not state['armed'] and 'a: fell while stopping' in state['reason']


def test_walks_move_the_team_frame_and_a_walk_into_the_other_robot_is_refused(team):
    backend, a, b = team(start_poses={'a': [0.0, 0.0, 0.0], 'b': [2.0, 0.0, 180.0]}, min_gap_m=0.3)
    value, _ = run(backend, 'a_walk', {'forward_m': 0.5})
    frame = value['team_frame']
    assert frame['poses']['a']['x_m'] == 0.5 and frame['poses']['a']['uncertainty_m'] > 0.05
    assert frame['gap_m']['a-b'] < 1.5 - 0.6
    with pytest.raises(Refused, match='a and b would be .* m apart'):
        run(backend, 'a_walk', {'forward_m': 1.0})
    with pytest.raises(Refused, match='too close'):       # both walking toward each other in one group
        run(backend, 'team_parallel', {'calls': [{'tool': 'a_walk', 'arguments': {'forward_m': 0.4}},
                                                 {'tool': 'b_walk', 'arguments': {'forward_m': 0.4}}]})
    assert [name for name, *_ in a.calls] == ['a_walk'] and b.calls == []
    value, _ = run(backend, 'team_parallel', {'coupled': True, 'calls': [
        {'tool': 'a_walk', 'arguments': {'forward_m': 0.4}}, {'tool': 'b_walk', 'arguments': {'forward_m': 0.4}}]})
    assert value['done'] and value['team_frame']['poses']['b']['x_m'] == pytest.approx(1.6, abs=1e-3)
    value, _ = run(backend, 'team_set_pose', {'robot': 'a', 'x_m': 0.0, 'y_m': 1.5, 'yaw_deg': 90, 'uncertainty_m': 0.02,
                                              'source': 'table corner in both cameras'})
    assert value['team_frame']['poses']['a']['source'].startswith('agent: table corner')
    run(backend, 'a_walk', {'forward_m': 0.5})            # along +y now, away from b


def test_a_turn_walks_along_the_new_heading_or_the_old_one():
    frame = TeamFrame({'a': 0.3, 'b': 0.3}, min_gap_m=0.2)
    frame.set('a', 0, 0, 0)
    frame.set('b', 0, 1.5, 0)
    assert frame.conflicts({'a': (1.0, 0.0, 0.0)})[0] == []                 # straight ahead: clear of b at the side
    conflict, _ = frame.conflicts({'a': (1.0, 0.0, 90.0)})                  # turning left first walks into b
    assert conflict and conflict[0]['robots'] == ['a', 'b']
    frame.advance('a', 1.0, 0.0, 90.0, measured=True)
    assert (round(frame.poses['a'].x, 3), round(frame.poses['a'].y, 3)) == (1.0, 0.0)


def test_without_poses_the_separation_check_is_off_unless_required(team):
    backend, a, b = team()
    assert run(backend, 'a_walk', {'forward_m': 1.0})[0]['team_frame']['poses'] == {'a': None, 'b': None}
    strict, a2, b2 = team(require_poses=True)
    with pytest.raises(Refused, match='no pose for a, b'):
        run(strict, 'a_walk', {'forward_m': 1.0})


def test_options_are_one_object_per_robot_and_team(tmp_path):
    with pytest.raises(ValueError, match="unknown g1r1 backend options: \\['zmq_host'\\]"):
        G1R1Backend(configuration(tmp_path, backend_options={'zmq_host': '10.0.0.1'}))
    with pytest.raises(ValueError, match='unknown team options'):
        G1R1Backend(configuration(tmp_path, backend_options={'team': {'gap': 1}}))
    from services.backends import BACKENDS, backend_class
    assert backend_class('g1r1') is G1R1Backend and set(G1R1Backend.MEMBERS.values()) <= set(BACKENDS)
    assert {KINDS[key].backend for key in G1R1Backend.MEMBERS.values()} == {'g1sonic', 'r1sonic'}


def test_a_hold_reads_as_disarmed_without_touching_the_operators_file(tmp_path):
    root = tmp_path/'session'
    arming.write(root, True, by='operator')
    arming.hold(root, 'k', 'team stop: test')
    state = arming.read(root)
    assert not state['armed'] and state['held'] and 'team stop: test' in state['reason']
    assert arming.read(root, holds=False)['armed']
    arming.release(root, 'k')
    assert arming.read(root)['armed']
    arming.write(root, False, reason='operator')
    arming.hold(root, 'k', 'x')
    assert arming.read(root)['reason'] == 'operator'       # a hold never hides the operator's own disarm
    arming.release(root, 'k')


def test_the_team_serves_through_the_robot_service(tmp_path):
    from services.controller.frontend import Frontend
    from services.controller.session import Session
    config = configuration(tmp_path)
    arming.write(config.root, True, by='test')
    backend = TeamBackend(config, members={'a': (kind('a'), FakeRobot('a', config.root)),
                                           'b': (kind('b'), FakeRobot('b', config.root))})
    frontend = Frontend(Session(config), backend)
    try:
        assert {'status', 'team_parallel', 'a_move', 'b_move'} <= {tool['name'] for tool in frontend.tools()}
        result = frontend.call('team_parallel', {'calls': [{'tool': 'a_move', 'arguments': {'secs': 0.05}},
                                                            {'tool': 'b_observe'}]})
        value = json.loads(result['content'][0]['text'])
        assert value['done'] and value['image_order'] == ['b:b_observe'] and len(result['content']) == 2
        refused = frontend.call('team_parallel', {'calls': []})
        assert refused['isError'] and 'at most one per robot' in refused['content'][0]['text']
        status = json.loads(frontend.call('status', {})['content'][0]['text'])
        assert set(status['robot_status']['robots']) == {'a', 'b'}
        frontend.backend.poll_operator()
    finally:
        frontend.close()
    assert all(robot.backend.closed for robot in backend.robots.values())


def test_the_real_g1_and_r1_backends_run_as_a_team_and_stop_together(tmp_path):
    """The G1 backend against the fake SONIC stack, the R1 backend with no stack behind it: tool names stay apart,
    the G1 runs on its own thread, and team_stop halts its streamed motion without disarming."""
    pytest.importorskip('zmq')
    pytest.importorskip('msgpack')
    import socket
    from services.g1sonic.fake_stack import FakeSonicStack
    from services.g1sonic import tools as G1
    from services.r1sonic import tools as R1
    from services.team import tools as T
    from test_g1sonic_backend import STANDING, ramp
    stack = FakeSonicStack(joints=STANDING).start()
    with socket.socket() as probe:           # a port nothing listens on: the R1 has no stack in this test
        probe.bind(('127.0.0.1', 0))
        free = probe.getsockname()[1]
    config = configuration(tmp_path, robot='g1r1', backend_options={
        'g1': {'zmq_host': '127.0.0.1', 'pose_port': 0, 'state_port': stack.state_port, 'camera': None,
               'state_max_age_s': 0.5, 'planner_settle_s': 0.0, 'stand_ramp_s': 0.0, 'walk_warmup': False, 'hands_walk': True, 'planner_stream': True},
        'r1': {'stack_host': '127.0.0.1', 'stack_port': free, 'stack_timeout_s': 0.2},
        'team': {'start_poses': {'g1': [0, 0, 0], 'r1': [1.5, 0, 180]}}})
    (config.root/'agent-workspace').mkdir(parents=True)
    backend = G1R1Backend(config)
    try:
        g1 = backend.robots['g1'].backend
        stack.connect_pose('127.0.0.1', g1.publisher.port)
        names = [tool['name'] for tool in backend.tools()]
        assert len(names) == len(set(names)) == len(T.TEAM_TOOLS) + len(G1.ROBOT_TOOLS) + len(R1.ROBOT_TOOLS)
        arming.write(config.root, True)
        value, _ = run(backend, 'g1_hold')
        assert 'start_error' not in value and g1.control_mode == 'streamed'
        target = STANDING.copy()
        target[15] += 0.3                      # one arm joint, slowly: 3 s of rows
        rows = [row.tolist() for row in ramp(STANDING, target, 95)]
        started, _ = run(backend, 'team_start', {'calls': [{'tool': 'g1_step', 'arguments': {'actions': rows}}]})
        time.sleep(0.3)
        stopped, _ = run(backend, 'team_stop', {'reason': 'test'})
        assert [job['tool'] for job in stopped['stopped']] == ['g1_step']
        waited, _ = run(backend, 'team_wait')
        error = waited['jobs'][0]['error']
        assert error['type'] == 'MotionStopped' and 'held by the robot service' in error['reason']
        assert 0 < error['robot_state']['rows_sent'] < len(rows)
        assert arming.read(config.root)['armed']
        assert backend.status()['robots']['r1']['kind'] == 'r1sonic'
    finally:
        backend.close()
        stack.stop()
