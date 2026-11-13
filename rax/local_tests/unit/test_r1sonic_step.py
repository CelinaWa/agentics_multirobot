"""r1_step, the R1 backend's low-level reference tool, against a stack that records what it is sent."""
import types

import numpy as np
import pytest

from services import arming
from services.r1sonic import tools as T
from services.r1sonic.backend import DEFAULT_UPPER, R1SonicBackend
from services.r1sonic.tools import RejectedBeforeAction


class RecordingStack:
    """Stands still on the default pose and reports whatever was streamed as executed."""

    def __init__(self, names):
        self.names, self.streamed = names, []
        self.endpoint, self.requests, self.failures = 'recording', 0, 0
        self.q = {**{n: 0.0 for n in names}, **DEFAULT_UPPER, **T.DEFAULT_LEGS}

    def state(self):
        return {'joint_names': self.names, 'q': [self.q[n] for n in self.names], 'base_quat_wxyz': [1, 0, 0, 0],
                'tilt_deg': 0.5, 'fallen': False, 'queued': 0, 'resets': 0, 'gripper_width_m': {'left': 0.07, 'right': 0.07}}

    def stream(self, rows):
        self.streamed.append(np.asarray(rows, dtype=float))
        return {'queued': len(rows)}

    def hold(self):
        return {'held': True}

    def close(self):
        pass


@pytest.fixture
def backend(tmp_path):
    root = tmp_path/'session'
    (root/'agent-workspace').mkdir(parents=True)
    arming.write(root, True, by='test')
    config = types.SimpleNamespace(root=root, task='t', observation_profile='rgb-only', backend_options={})
    value = R1SonicBackend(config, stack=RecordingStack(list(T.LEGS) + list(T.UPPER) + ['head_pitch_joint', 'head_yaw_joint']),
                           clock=iter(np.arange(0, 1e6, 0.5)).__next__)
    value._wait = lambda duration, allow_steps=False: {'tilt_max_deg': 0.5, 'steps': 0, 'foot_lift_max_m': 0.0,
                                                       'state': value._state()}
    yield value
    value.close()


def crouch(depth, rows=60):
    names = [f'{s}_{j}_joint' for s in T.ARMS for j in ('hip_pitch', 'knee', 'ankle_pitch')]
    start = np.array([T.DEFAULT_LEGS[n] for n in names])
    offset = np.array([-depth / 2, depth, -depth / 2] * 2)
    return names, [(start + offset * (k + 1) / rows).tolist() for k in range(rows)]


def test_the_tool_is_listed_and_other_tools_send_plain_rows(backend):
    assert T.STEP in {tool['name'] for tool in backend.tools()} and T.STEP in backend.mutating
    backend.validate(T.MOVE_JOINTS, {'joints': {'waist_yaw_joint': 0.2}, 'secs': 1.0})
    backend.call(T.MOVE_JOINTS, {'joints': {'waist_yaw_joint': 0.2}, 'secs': 1.0})
    assert backend.stack.streamed[-1].shape[1] == 24                       # no leg reference: the stack's default stance
    assert backend.status()['command']['legs_reference'] == 'the default stance'


def test_leg_rows_ride_behind_the_points_and_stay_for_later_motions(backend):
    names, rows = crouch(0.6)
    arguments = {'joints': names, 'rows': rows}
    backend.validate(T.STEP, arguments)
    value, _ = backend.call(T.STEP, arguments)
    sent = backend.stack.streamed[-1]
    assert sent.shape == (60, 36)
    knee = 24 + T.LEGS.index('left_knee_joint')
    assert sent[0, knee] == pytest.approx(0.31) and sent[-1, knee] == pytest.approx(0.9)
    assert np.allclose(sent[:, :21], sent[0, :21])                          # the arms did not move: the three points stay
    assert value['legs_reference']['right_knee_joint'] == 0.9 and value['rows'] == 60
    assert backend.status()['command']['legs']['left_hip_pitch_joint'] == pytest.approx(-0.4)
    backend.call(T.MOVE_JOINTS, {'joints': {'waist_yaw_joint': 0.2}, 'secs': 1.0})
    later = backend.stack.streamed[-1]
    assert later.shape[1] == 36 and np.allclose(later[:, knee], 0.9)        # an arm motion keeps the crouch


def test_arm_joints_move_the_points(backend):
    rows = [[DEFAULT_UPPER['right_elbow_joint'] - 0.01 * (k + 1)] for k in range(20)]
    backend.call(T.STEP, {'joints': ['right_elbow_joint'], 'rows': rows})
    sent = backend.stack.streamed[-1]
    assert sent.shape == (20, 24) and not np.allclose(sent[0, 7:10], sent[-1, 7:10])   # the right wrist point moved
    assert backend.cmd['right_elbow_joint'] == pytest.approx(DEFAULT_UPPER['right_elbow_joint'] - 0.2)


def test_limits_and_jumps_are_refused_before_anything_is_sent(backend):
    names, rows = crouch(0.6)
    with pytest.raises(RejectedBeforeAction, match='changes by .* rad between rows'):
        backend.validate(T.STEP, {'joints': names, 'rows': rows[::10]})              # six rows for the same crouch
    with pytest.raises(RejectedBeforeAction, match='from its current command'):
        backend.validate(T.STEP, {'joints': ['left_knee_joint'], 'rows': [[1.5]]})
    names, rows = crouch(2.6, rows=250)
    with pytest.raises(RejectedBeforeAction, match='outside its limits'):
        backend.validate(T.STEP, {'joints': names, 'rows': rows})
    with pytest.raises(RejectedBeforeAction, match='one angle per named joint'):
        backend.validate(T.STEP, {'joints': ['left_knee_joint'], 'rows': [[0.3, 0.3]]})
    with pytest.raises(RejectedBeforeAction, match='distinct names'):
        backend.validate(T.STEP, {'joints': ['head_pitch_joint'], 'rows': [[0.0]]})
    assert backend.stack.streamed == []
    arming.write(backend.config.root, False)
    with pytest.raises(PermissionError):
        backend.validate(T.STEP, {'joints': ['left_knee_joint'], 'rows': [[0.31]]})


def test_the_operator_can_cap_how_far_the_legs_leave_the_stance(backend):
    backend.options['max_leg_offset_rad'] = 0.2
    names, rows = crouch(0.6)
    with pytest.raises(RejectedBeforeAction, match='the operator allows 0.2 rad'):
        backend.validate(T.STEP, {'joints': names, 'rows': rows})
    names, rows = crouch(0.2)
    backend.validate(T.STEP, {'joints': names, 'rows': rows})
    assert backend.status()['limits']['max_leg_offset_rad'] == 0.2


def test_a_crouch_survives_arms_that_drifted_from_their_command(backend):
    names, rows = crouch(0.6)
    backend.call(T.STEP, {'joints': names, 'rows': rows})
    backend.stack.q['right_elbow_joint'] += 0.7            # the controller holds the arm elsewhere while crouched
    backend.call(T.MOVE_JOINTS, {'joints': {'waist_yaw_joint': 0.1}, 'secs': 1.0})
    sent = backend.stack.streamed[-1]
    assert sent.shape[1] == 36 and np.allclose(sent[:, 24 + T.LEGS.index('left_knee_joint')], 0.9)
    assert any('re-synced' in e['what'] for e in backend.events)        # the arms did re-sync; the legs stayed


def test_a_side_step_is_a_walk_argument_on_its_own():
    from services.r1sonic import tools as T
    T.validate_request(T.WALK, {'left_m': 0.1})
    T.validate_request(T.WALK, {'forward_m': 0.2, 'turn_deg': 10})
    for bad in ({'left_m': 0.5}, {'left_m': 0.1, 'forward_m': 0.2}, {'left_m': -0.1, 'turn_deg': 5}):
        with pytest.raises(ValueError):
            T.validate_request(T.WALK, bad)


def test_the_team_frame_takes_a_side_steps_measured_travel():
    from services.team import members as M
    assert M._r1_motion('r1_walk', {'left_m': -0.1}) == (0.0, -0.1, 0.0)
    out = M._r1_result('r1_walk', {'left_m': -0.1}, {'walked_m': 0.02, 'lateral_m': -0.08, 'turned_deg': 1.0})
    assert out[:4] == (0.02, -0.08, 1.0, True)
    out = M._r1_result('r1_walk', {'forward_m': 0.3}, {'walked_m': 0.28, 'lateral_m': 0.05, 'turned_deg': 2.0})
    assert out[:2] == (0.28, 0.0) and out[4] == 0.05
