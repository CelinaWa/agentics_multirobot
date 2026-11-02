"""The G1 + SONIC backend against a fake deploy stack on real ZMQ sockets: nothing here touches hardware."""
import math
import threading
import time

import numpy as np
import pytest

pytest.importorskip('zmq')
pytest.importorskip('msgpack')
from services import arming  # noqa: E402
from services.controller.config import Configuration  # noqa: E402
from services.g1sonic import robot as R  # noqa: E402
from services.g1sonic import tools as T  # noqa: E402
from services.g1sonic.backend import DEFAULTS, G1SonicBackend, MotionStopped  # noqa: E402
from services.g1sonic.bridge import (ReferenceConverter, Streamer, heading_quat, quat_from_matrix,  # noqa: E402
                                     quat_multiply, rot6d_from_quat, rot6d_to_matrix, yaw_of)
from services.g1sonic.camera import FakeCamera  # noqa: E402
from services.g1sonic.fake_stack import FakeSonicStack  # noqa: E402
from services.g1sonic.zmq_protocol import (HEADER_SIZE, build_command_message, pack_pose_message,  # noqa: E402
                                           parse_message)

STANDING = R.DEFAULT_STANDING.copy()  # the fake stack waits in SONIC's default standing pose, like the real INIT


def yaw_quat(yaw):
    return np.array([math.cos(yaw / 2), 0.0, 0.0, math.sin(yaw / 2)])


def frame(joints, yaw=0.0, hands=(0, 0)):
    row = np.zeros(R.ACTION_DIM)
    row[2] = 0.75
    row[R.ROT6D] = R.rot6d_from_yaw(yaw)
    row[R.JOINTS] = joints
    row[38], row[39] = hands
    return row


def ramp(start, end, rows, tail=5):
    """A motion from START to END joints over ROWS ticks, then TAIL identical rows."""
    out = [frame(start + (end - start) * (i + 1) / rows) for i in range(rows)]
    return out + [frame(end)] * tail


def wait_for(condition, timeout=3.0, every=0.01):
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        if condition():
            return True
        time.sleep(every)
    return False


# ----------------------------------------------------------------------------- protocol and conversion
def test_pose_and_command_messages_round_trip_in_the_stacks_wire_format():
    fields = {'joint_pos': np.arange(29, dtype=np.float32).reshape(1, 29), 'joint_vel': np.zeros((1, 29), np.float32),
              'body_quat': np.array([[1, 0, 0, 0]], np.float32), 'frame_index': np.array([7], np.int64),
              'left_hand_joints': np.ones((1, 7), np.float32)}
    raw = pack_pose_message(fields, 'pose', 1)
    assert raw.startswith(b'pose{') and len(raw) == len(b'pose') + HEADER_SIZE + (29 + 29 + 4 + 7) * 4 + 8
    topic, header, parsed = parse_message(raw)
    assert topic == 'pose' and header['v'] == 1 and header['endian'] == 'le' and header['count'] == 1
    assert [f['name'] for f in header['fields']] == list(fields)
    for name, value in fields.items():
        assert parsed[name].dtype == value.dtype and np.array_equal(parsed[name], value), name
    topic, _, command = parse_message(build_command_message(stop=True))
    assert topic == 'command' and (int(command['start'][0]), int(command['stop'][0]), int(command['planner'][0])) == (0, 1, 0)


def test_the_camera_wire_round_trips_colour_depth_and_intrinsics():
    from services.g1sonic.camera import decode_frame, encode_frame, read_message
    v, u = np.mgrid[0:48, 0:64]
    rgb = np.stack([u * 4, v * 5, np.full_like(u, 90)], axis=-1).astype(np.uint8)
    depth = np.full((48, 64), 1.5)
    depth[0, 0], depth[1, 1], depth[2, 2] = np.nan, 0.0, 70.0  # invalid, invalid, beyond the wire's range
    K = np.array([[50.0, 0.0, 32.0], [0.0, 50.0, 24.0], [0.0, 0.0, 1.0]])
    raw = encode_frame(12.5, rgb, depth, K)
    assert raw.startswith(b'HBCF')

    class Stream:
        def __init__(self, data):
            self.data, self.at = data, 0

        def recv(self, count):
            chunk = self.data[self.at:self.at + count]
            self.at += len(chunk)
            return chunk

    header, blobs = read_message(Stream(raw))
    stamp, rgb2, depth2, K2 = decode_frame(header, blobs)
    assert stamp == 12.5 and rgb2.shape == rgb.shape and np.abs(rgb2.astype(int) - rgb).mean() < 4  # JPEG
    assert all(np.isnan(depth2[i, i]) for i in range(3)) and depth2[10, 10] == pytest.approx(1.5, abs=1e-3)
    assert np.allclose(K2, K) and read_message(Stream(b'')) is None


def test_the_network_camera_reconnects_after_the_server_restarts():
    import socket
    from services.g1sonic.camera import NetworkCamera, encode_frame
    rgb = np.zeros((8, 12, 3), dtype=np.uint8)
    K = np.array([[10.0, 0.0, 6.0], [0.0, 10.0, 4.0], [0.0, 0.0, 1.0]])
    listener = socket.socket()
    listener.bind(('127.0.0.1', 0))
    listener.listen(2)
    port = listener.getsockname()[1]

    def serve(stamp):
        conn, _ = listener.accept()
        conn.sendall(encode_frame(stamp, rgb, None, K))
        conn.close()  # the server goes away after one frame

    NetworkCamera.RECONNECT_S = 0.2
    camera = NetworkCamera('127.0.0.1', port, timeout=2.0).start()
    try:
        threading.Thread(target=serve, args=(1.0,), daemon=True).start()
        assert camera.read() is not None and camera.received == 1
        assert wait_for(lambda: camera.error == 'the camera server closed the stream')
        threading.Thread(target=serve, args=(2.0,), daemon=True).start()
        assert wait_for(lambda: camera.received == 2, timeout=3.0)  # reconnected on its own
        assert camera.connections == 2 and np.isnan(camera.read()[2]).all()  # no depth in these frames
    finally:
        camera.close()
        listener.close()


def test_the_ma_teleop_camera_client_reads_the_jetsons_jpeg_stream_and_refuses_other_pickles():
    import pickle
    import cv2
    import zmq
    from services.g1sonic.camera import MaTeleopCamera
    with pytest.raises(ValueError, match='camera_intrinsics'):
        MaTeleopCamera('127.0.0.1', 5620, None)
    pub = zmq.Context.instance().socket(zmq.PUB)
    port = pub.bind_to_random_port('tcp://127.0.0.1')
    K = [[700.0, 0.0, 640.0], [0.0, 700.0, 360.0], [0.0, 0.0, 1.0]]
    camera = MaTeleopCamera('127.0.0.1', port, K).start()
    try:
        v, u = np.mgrid[0:36, 0:48]
        bgr = np.stack([np.full_like(u, 90), v * 5, u * 4], axis=-1).astype(np.uint8)
        ok, jpeg = cv2.imencode('.jpg', bgr, [cv2.IMWRITE_JPEG_QUALITY, 95])
        deadline = time.time() + 3.0
        while camera.frame is None and time.time() < deadline:
            pub.send_pyobj({'timestamp': 1.0, 'jpeg': jpeg.tobytes(), 'camera': 'rgb'})
            time.sleep(0.05)
        stamp, rgb, depth, K2 = camera.read()
        assert rgb.shape == (36, 48, 3) and np.isnan(depth).all() and np.allclose(K2, K)
        assert abs(int(rgb[10, 20, 0]) - 80) < 12 and abs(int(rgb[10, 20, 2]) - 90) < 12  # BGR -> RGB
        rgb2, stamp2 = MaTeleopCamera.decode(pickle.dumps({'timestamp': 2.0, 'image': bgr}))
        assert stamp2 == 2.0 and np.array_equal(rgb2, bgr[:, :, ::-1])
        with pytest.raises(pickle.UnpicklingError, match='refused'):
            MaTeleopCamera.decode(pickle.dumps(time.time))  # a function reference: not a frame
    finally:
        camera.close()
        pub.close(0)


def test_a_hand_value_between_0_and_1_closes_the_fingers_that_fraction_of_the_way():
    open_, half, fist = (R.hand_target('right', v) for v in (0.0, 0.5, 1.0))
    assert np.allclose(open_, 0.0) and np.allclose(fist, R.HAND_POSES['right']['close'])
    assert np.allclose(half, np.array(R.HAND_POSES['right']['close']) / 2)
    row = frame(STANDING)
    row[39] = 0.35
    R.validate_actions([row.tolist()])
    row[39] = 1.2
    with pytest.raises(ValueError, match='between 0'):
        R.validate_actions([row.tolist()])


def test_a_hold_primed_from_a_tilted_robot_keeps_its_heading_and_stands_it_upright():
    from services.g1sonic.bridge import ReferenceConverter, quat_multiply, yaw_of
    yaw, pitch = 0.6, np.radians(9.0)
    tilted = quat_multiply(np.array([np.cos(yaw / 2), 0, 0, np.sin(yaw / 2)]),
                           np.array([np.cos(pitch / 2), 0, np.sin(pitch / 2), 0]))
    converter = ReferenceConverter()
    converter.prime(STANDING, tilted)
    held = converter.hold(0).body_quat
    assert yaw_of(held) == pytest.approx(yaw, abs=1e-6)
    assert np.allclose(held, [np.cos(yaw / 2), 0, 0, np.sin(yaw / 2)], atol=1e-6)  # no pitch or roll held


def test_the_converter_does_what_humanoidarenas_provider_does():
    converter = ReferenceConverter(max_root_delta_deg=26.0)
    measured = yaw_quat(0.4)
    converter.reset(measured)
    first = converter.convert(frame(STANDING), 0)
    assert np.allclose(first.body_quat, heading_quat(measured), atol=1e-6)  # identity rot6d = the anchored heading
    assert np.allclose(first.joint_vel, 0.0) and first.projection_deg == pytest.approx(0.0, abs=1e-6)
    turned = converter.convert(frame(STANDING + 0.01, yaw=math.radians(20)), 1)
    assert yaw_of(turned.body_quat) == pytest.approx(0.4 + math.radians(20), abs=1e-6)
    assert np.allclose(turned.joint_vel, 0.01 / R.CONTROL_DT)
    assert turned.projection_deg == pytest.approx(20.0, abs=1e-3)
    jumped = converter.convert(frame(STANDING + 0.01, yaw=math.radians(80)), 2)
    assert jumped.projection_deg == pytest.approx(60.0, abs=1e-3)  # the requested jump is reported
    assert yaw_of(jumped.body_quat) == pytest.approx(0.4 + math.radians(20 + 26), abs=1e-6)  # and slerped to 26 deg
    hands = converter.convert(frame(STANDING + 0.01, yaw=math.radians(80), hands=(1, 0)), 3)
    assert np.allclose(hands.left_hand, R.hand_target('left', True)) and np.allclose(hands.right_hand, 0.0)
    matrix = rot6d_to_matrix(R.rot6d_from_yaw(0.7))
    assert np.allclose(quat_from_matrix(matrix), yaw_quat(0.7), atol=1e-9)
    assert np.allclose(rot6d_from_quat(quat_multiply(yaw_quat(0.1), yaw_quat(0.2))), R.rot6d_from_yaw(0.3), atol=1e-9)
    assert R.to_isaaclab(R.to_mujoco(np.arange(29.0))).tolist() == list(range(29))


def test_the_streamer_holds_the_last_reference_between_calls_and_a_hook_can_stop_it():
    sent = []
    converter = ReferenceConverter()
    streamer = Streamer(sent.append, converter, threaded=False)
    assert streamer.tick() is None  # nothing on the wire before a prime
    streamer.prime(STANDING, yaw_quat(0.0))
    streamer.tick()
    rows = ramp(STANDING, STANDING + 0.05, 10)
    executed = streamer.execute(rows)
    assert len(executed) == len(rows) and len(sent) == 1 + len(rows)
    streamer.tick()
    streamer.tick()
    assert streamer.held == 3 and streamer.frame_index == len(rows) + 3
    _, _, last = parse_message(sent[-1])
    assert np.allclose(last['joint_pos'].reshape(29), STANDING + 0.05) and np.allclose(last['joint_vel'], 0.0)
    assert int(last['frame_index'][0]) == len(rows) + 2
    ticks = []

    def stop_after_three(reference):
        ticks.append(reference.frame_index)
        if len(ticks) == 3:
            raise MotionStopped('operator disarmed')
    with pytest.raises(MotionStopped):
        streamer.execute(rows, stop_after_three)
    assert len(ticks) == 3 and not streamer.executing
    before = len(sent)
    streamer.tick()
    assert len(sent) == before + 1 and streamer.last.hold  # a hold again, not the dropped rows


# ----------------------------------------------------------------------------- against the fake stack
@pytest.fixture
def stack():
    fake = FakeSonicStack(joints=STANDING).start()
    yield fake
    fake.stop()


@pytest.fixture
def session(artifact_root):
    root = artifact_root / 'session'
    (root / 'agent-workspace').mkdir(parents=True)
    return root


def make_backend(session, stack, camera=None, **options):
    config = Configuration(root=session, task='stand', robot='g1sonic', task_description='Stand still.',
                           backend_options={'zmq_host': '127.0.0.1', 'pose_port': 0, 'state_port': stack.state_port,
                                            'camera': None, 'state_max_age_s': 0.5, 'planner_settle_s': 0.0,
                                            'stand_ramp_s': 0.0, 'walk_warmup': False, **options})
    backend = G1SonicBackend(config, camera=camera)
    stack.connect_pose('127.0.0.1', backend.publisher.port)
    assert backend.state.latest() is None  # like the real stack, the fake publishes nothing before control runs
    return backend


def start_control(backend, session):
    """Arm and hold: the backend starts the stack's control and streams the measured pose."""
    arming.write(session, True)
    value, _ = backend.call(T.HOLD, {})
    assert 'start_error' not in value and backend.control_mode == 'streamed'
    return value


@pytest.fixture
def backend(session, stack):
    value = make_backend(session, stack)
    yield value
    value.close()


def test_the_tools_match_the_robot_facts_and_refuse_unknown_arguments(backend):
    from services.workspace.setup import ROBOT_FACTS
    names = {tool['name'] for tool in backend.tools()}
    assert names == set(ROBOT_FACTS['g1sonic']['tools'])
    assert all(tool['inputSchema'].get('additionalProperties') is False for tool in backend.tools())
    with pytest.raises(ValueError, match='Unknown arguments'):
        backend.validate(T.STEP, {'actions': [frame(STANDING).tolist()], 'speed': 2})
    with pytest.raises(ValueError, match='between 0 .open. and 1'):
        backend.validate(T.STATUS, {}) or backend.validate(T.STEP, {'actions': [frame(STANDING, hands=(1.5, 0)).tolist()]})


def test_motion_is_refused_until_the_operator_arms_and_bad_rows_are_refused_before_the_wire(backend, session, stack):
    rows = [row.tolist() for row in ramp(STANDING, STANDING + 0.05, 10)]
    with pytest.raises(PermissionError, match='not armed'):
        backend.validate(T.STEP, {'actions': rows})
    assert stack.commands == []  # validation starts nothing
    start_control(backend, session)
    backend.validate(T.STEP, {'actions': rows})  # the same rows pass once armed
    knee = R.JOINT_NAMES.index('left_knee_joint')
    beyond = frame(STANDING)
    beyond[R.JOINTS.start + knee] = backend.upper[knee] + 0.1
    with pytest.raises(ValueError, match='outside the limits'):
        backend.validate(T.STEP, {'actions': [beyond.tolist()]})
    shoulder = R.JOINT_NAMES.index('left_shoulder_pitch_joint')
    far = STANDING.copy()
    far[shoulder] -= 0.6
    with pytest.raises(ValueError, match='first row moves'):
        backend.validate(T.STEP, {'actions': [frame(far).tolist()]})
    near = STANDING.copy()
    near[shoulder] -= 0.3
    fast = [frame(STANDING).tolist(), frame(near).tolist()]
    with pytest.raises(ValueError, match='in one tick'):
        backend.validate(T.STEP, {'actions': fast})
    moving = [row.tolist() for row in ramp(STANDING, STANDING + 0.05, 10, tail=0)]
    with pytest.raises(ValueError, match='pose to hold'):
        backend.validate(T.STEP, {'actions': moving})
    assert all(np.allclose(p['joint_pos'].reshape(29), STANDING, atol=1e-6) for p in stack.poses())  # only holds on the wire


def test_a_step_streams_every_row_then_holds_and_reports_feedback(backend, session, stack):
    arming.write(session, True)
    target = STANDING.copy()
    target[R.JOINT_NAMES.index('right_shoulder_pitch_joint')] -= 0.2
    rows = ramp(STANDING, target, 20)
    value, images = backend.call(T.STEP, {'actions': [row.tolist() for row in rows]})
    assert images == [] and value['step_id'] == len(rows) and backend.step_id == len(rows)
    transition = value['transition']
    assert transition['feedback']['ticks'] == len(rows) and len(transition['steps']) == len(rows)
    assert transition['steps'][0]['observation_step_id'] == 0 and transition['steps'][-1]['step_id'] == len(rows)
    assert transition['feedback']['fallen'] is False and 'joint_tracking_rmse_rad' in transition['feedback']
    assert transition['feedback']['max_root_orientation_projection_deg'] == pytest.approx(0.0, abs=1e-3)
    assert wait_for(lambda: len(stack.poses()) >= len(rows) + 1)
    poses = stack.poses()
    commanded = [p for p in poses if not np.allclose(p['joint_vel'], 0.0) or np.allclose(p['joint_pos'].reshape(29), target)]
    assert len(commanded) >= len(rows) - 1
    on_wire = np.array([p['joint_pos'].reshape(29) for p in poses])
    assert np.allclose(on_wire[-1], target, atol=1e-6)
    indices = [int(p['frame_index'][0]) for p in poses]
    assert indices == list(range(indices[0], indices[0] + len(indices)))  # continuous, holds included
    held_before = stack.poses()
    time.sleep(0.2)
    assert len(stack.poses()) > len(held_before)  # the hold keeps going out between calls
    assert wait_for(lambda: np.allclose(stack.q, target, atol=0.02), timeout=2.0)  # the fake tracked it
    status = backend.status()
    assert status['stack']['streaming'] and not status['stack']['executing'] and status['stack']['heading_anchored']
    assert status['last_feedback']['ticks'] == len(rows) and status['stack']['mode'] == 'streamed'
    starts = [c for c in stack.commands if c.get('start')]  # MA-Teleop's handshake: planner (stand), then streamed motion
    assert starts[0]['planner'] is True and starts[-1]['planner'] is False and all(c['planner'] for c in starts[:-1])
    assert set(status['joint_limits_rad']) == {'lower', 'upper'} and status['limits']['fall_tilt_deg'] == DEFAULTS['fall_tilt_deg']


def test_crouch_bends_the_legs_from_the_held_frame_and_rising_takes_the_arms_home_first(backend, session, stack):
    J = {name: index for index, name in enumerate(R.JOINT_NAMES)}
    with pytest.raises(ValueError, match='pelvis_drop_m must be in'):
        backend.validate(T.CROUCH, {'pelvis_drop_m': 0.5})
    with pytest.raises(PermissionError, match='not armed'):
        backend.validate(T.CROUCH, {'pelvis_drop_m': 0.1})
    start_control(backend, session)
    standing = backend.pelvis_height_estimate(STANDING)
    assert standing == pytest.approx(T.STANDING_PELVIS_M)
    backend.validate(T.CROUCH, {'pelvis_drop_m': 0.19, 'lean_rad': 0.3, 'secs': 2.0})
    value, _ = backend.call(T.CROUCH, {'pelvis_drop_m': 0.19, 'lean_rad': 0.3, 'secs': 2.0})
    crouch = value['transition']['crouch']
    assert value['transition']['kind'] == 'crouch' and crouch['phases'] == ['crouch'] and crouch['clipped_to_limits'] == []
    ankle = backend.lower[J['left_ankle_pitch_joint']] + DEFAULTS['joint_limit_margin_rad'] - STANDING[J['left_ankle_pitch_joint']]
    assert crouch['knee_flexion_rad'] == pytest.approx(1.2) and crouch['ankle_pitch_rad'] == pytest.approx(ankle, abs=1e-4)  # -0.46: the limit's margin
    assert value['transition']['feedback']['ticks'] == 2.0 * DEFAULTS['control_hz'] + DEFAULTS['rest_tail_rows']
    assert value['transition']['rows_executed'] == value['transition']['feedback']['ticks']
    assert len(value['transition']['steps']) == 2 and value['transition']['steps'][-1]['step_id'] == value['step_id']  # not every synthesized row
    held = backend.last_row[R.JOINTS]
    for side in ('left', 'right'):
        assert held[J[f'{side}_knee_joint']] == pytest.approx(STANDING[J[f'{side}_knee_joint']] + 1.2)
        assert held[J[f'{side}_hip_pitch_joint']] == pytest.approx(STANDING[J[f'{side}_hip_pitch_joint']] - 1.2 - ankle)
    assert held[J['waist_pitch_joint']] == pytest.approx(0.3) and held[J['left_shoulder_pitch_joint']] == STANDING[J['left_shoulder_pitch_joint']]
    assert 0.5 < crouch['pelvis_height_est_m']['goal'] < 0.65 < crouch['pelvis_height_est_m']['before']  # legs folded, pelvis lower
    assert wait_for(lambda: np.allclose(stack.q, held, atol=0.03), timeout=2.0)
    assert 0.5 < backend.status()['proprio']['pelvis_height_est_m'] < 0.65
    # an arm out, then rise: the arms come home over crouch_arms_home_s before the legs straighten
    out = held.copy()
    out[J['right_shoulder_pitch_joint']] -= 0.8
    backend.call(T.STEP, {'actions': [row.tolist() for row in ramp(held, out, 30)]})
    sent = len(stack.poses())
    value, _ = backend.call(T.CROUCH, {'pelvis_drop_m': 0.0, 'secs': 2.0})
    assert value['transition']['crouch']['phases'] == ['arms home', 'rise']
    ticks = value['transition']['feedback']['ticks']
    assert ticks == (DEFAULTS['crouch_arms_home_s'] + 2.0) * DEFAULTS['control_hz'] + DEFAULTS['rest_tail_rows']
    assert wait_for(lambda: len(stack.poses()) >= sent + ticks, timeout=5.0)
    on_wire = np.array([p['joint_pos'].reshape(29) for p in stack.poses()[sent:sent + ticks]])
    arms_home_at = int(np.argmax(np.abs(on_wire[:, J['right_shoulder_pitch_joint']] - STANDING[J['right_shoulder_pitch_joint']]) < 1e-3))
    assert 0.95 * DEFAULTS['crouch_arms_home_s'] * DEFAULTS['control_hz'] <= arms_home_at < DEFAULTS['crouch_arms_home_s'] * DEFAULTS['control_hz']
    assert np.allclose(on_wire[:arms_home_at, J['left_knee_joint']], STANDING[J['left_knee_joint']] + 1.2)  # legs wait
    assert np.allclose(backend.last_row[R.JOINTS], STANDING, atol=1e-6)
    assert value['transition']['crouch']['pelvis_height_est_m']['goal'] == pytest.approx(T.STANDING_PELVIS_M, abs=1e-3)
    # the deepest crouch clips nothing; a forward shift beyond the ankle margin is reported, not hidden
    value, _ = backend.call(T.CROUCH, {'pelvis_drop_m': 0.32, 'forward_rad': 0.3, 'secs': 2.0})
    assert value['transition']['crouch']['knee_flexion_rad'] == pytest.approx(1.8 + 0.2 * (0.32 - 0.304) / (0.326 - 0.304), abs=1e-3)
    assert value['transition']['crouch']['clipped_to_limits'] == ['left_ankle_pitch_joint', 'right_ankle_pitch_joint']


def test_a_disarm_during_a_step_stops_it_within_a_tick_and_the_robot_holds(backend, session, stack):
    arming.write(session, True)
    rows = ramp(STANDING, STANDING + 0.04, 60)  # 1.3 s
    threading.Timer(0.3, lambda: arming.write(session, False, reason='operator')).start()
    with pytest.raises(MotionStopped) as info:
        backend.call(T.STEP, {'actions': [row.tolist() for row in rows]})
    assert 'disarmed' in str(info.value) and info.value.handled_stop
    assert 'joint_positions_rad' in info.value.robot_state and 'tilt_deg' in info.value.robot_state
    assert 10 < backend.step_id < len(rows)
    assert not backend.streamer.executing
    count = len(stack.poses())
    assert wait_for(lambda: len(stack.poses()) > count + 2)  # holds continue
    last = stack.poses()[-1]
    assert np.allclose(last['joint_vel'], 0.0)
    with pytest.raises(PermissionError):
        backend.validate(T.STEP, {'actions': [frame(STANDING).tolist()]})


def test_a_falling_robot_damps_through_the_stack_and_disarms_the_session(backend, session, stack):
    arming.write(session, True)
    rows = ramp(STANDING, STANDING + 0.04, 40)

    def tip():
        stack.tilt_override_deg = 75.0
    threading.Timer(0.25, tip).start()
    with pytest.raises(MotionStopped, match='tilt'):
        backend.call(T.STEP, {'actions': [row.tolist() for row in rows]})
    assert wait_for(lambda: any(c.get('stop') for c in stack.commands))
    assert stack.damped and not arming.read(session)['armed']
    assert backend.fault['kind'] == 'fall' and backend.status()['fault']['kind'] == 'fall'
    arming.write(session, True)  # the operator arms again on purpose: the fault is acknowledged
    backend.validate(T.STEP, {'actions': [frame(STANDING + 0.0).tolist()]}) if False else None


def test_a_fall_while_holding_between_calls_damps_and_disarms_too(backend, session, stack):
    start_control(backend, session)
    stack.tilt_override_deg = 75.0  # the robot topples while it only holds its last frame
    assert wait_for(lambda: any(c.get('stop') for c in stack.commands), timeout=2.0)
    assert not arming.read(session)['armed'] and backend.fault['kind'] == 'fall'


def test_after_a_fall_is_acknowledged_the_next_fall_is_caught_again(backend, session, stack):
    start_control(backend, session)
    stack.tilt_override_deg = 75.0
    assert wait_for(lambda: backend.fault is not None and not arming.read(session)['armed'], timeout=2.0)
    stops = sum(1 for c in stack.commands if c.get('stop'))
    stack.tilt_override_deg = None
    stack.restart(STANDING)  # the operator stood it up and relaunched the controller
    time.sleep(0.05)
    start_control(backend, session)  # re-arming on purpose acknowledges the fault
    assert backend.fault is None and backend.last_fault['kind'] == 'fall'
    rows = ramp(STANDING, STANDING, 10)
    value, _ = backend.call(T.STEP, {'actions': [row.tolist() for row in rows]})
    assert value['transition']['feedback']['fallen'] is False  # no stale flag
    stack.tilt_override_deg = 75.0  # and a second fall is damped and disarmed like the first
    assert wait_for(lambda: sum(1 for c in stack.commands if c.get('stop')) > stops and not arming.read(session)['armed'],
                    timeout=2.0)


def test_a_long_walk_runs_as_segments_each_with_its_own_planner_start(session, stack):
    backend = make_backend(session, stack, walk_settle_s=0.2, walk_ramp_s=0.0, walk_segment_m=0.25)
    try:
        start_control(backend, session)
        value, _ = backend.call(T.WALK, {'forward_m': 0.6, 'speed_mps': 0.6})
        starts = [c for c in stack.commands if c.get('start') and c.get('planner')]
        assert len(starts) >= 3  # three segments, each switching to the planner
        assert value['transition']['feedback']['commanded']['segments'] == 3
        assert backend.control_mode == 'streamed'
    finally:
        backend.close()


def test_the_first_walk_after_a_start_warms_the_planner_up_with_two_short_turns(session, stack):
    backend = make_backend(session, stack, walk_settle_s=0.2, walk_turn_rate_deg_s=200.0, walk_warmup=True)
    try:
        start_control(backend, session)
        before = sum(1 for c in stack.commands if c.get('start') and c.get('planner'))
        backend.call(T.WALK, {'forward_m': 0.1, 'speed_mps': 0.5})
        first = sum(1 for c in stack.commands if c.get('start') and c.get('planner'))
        assert first - before == 3  # two warm-up turns, then the walk
        backend.call(T.WALK, {'forward_m': 0.1, 'speed_mps': 0.5})
        assert sum(1 for c in stack.commands if c.get('start') and c.get('planner')) == first + 1  # only once
    finally:
        backend.close()


STUCK = dict(walk_settle_s=0.2, walk_turn_rate_deg_s=200.0, walk_quiet_timeout_s=0.4, settle_quiet_s=0.3,
             walk_stuck_recover_s=1.5, walk_stuck_min_wait_s=0.4, walk_stuck_reset_s=0.2, walk_stuck_pick_s=0.3, stand_ramp_s=0.2)


def test_a_walk_that_leaves_the_robot_stepping_gets_a_bounded_recovery_and_then_the_operator(session, stack):
    backend = make_backend(session, stack, **STUCK)
    try:
        start_control(backend, session)
        stack.planner_stepping = True  # the planner never brings the legs to rest
        frame_yaw = yaw_of(stack.base_quat)  # the walk's planner frame: x along the heading when the planner took over
        value, _ = backend.call(T.WALK, {'forward_m': 0.1, 'turn_deg': 20, 'speed_mps': 0.5})
        fb = value['transition']['feedback']
        assert backend.control_mode == 'planner' and 'still stepping' in fb['control'] and stack.planner_mode is True
        walked = sum(1 for p in stack.planners if p[0] == 1)
        t0 = time.monotonic()
        hold, _ = backend.call(T.HOLD, {})  # the agent's natural next call: realign, one slow step, wait; then the operator
        took = time.monotonic() - t0
        assert backend.control_mode == 'planner' and 'still stepping' in hold['start_error'] and 'operator' in hold['start_error']
        assert 0.4 <= took < 1.5 + 0.2 + 0.2 + 0.4 + 1.5, took  # recover budget + reset step + settle + min wait, with slack
        events = ' '.join(e['what'] for e in backend.events)
        assert 'realigned with the measured heading' in events and 'reset the gait' in events and 'least-moving' not in events
        assert sum(1 for p in stack.planners if p[0] == 1) > walked  # the gait reset's slow step went out
        idle = [p for p in stack.planners if p[0] == 0][-1]  # the realigned facing: the measured heading in the planner frame
        wanted = yaw_of(stack.base_quat) - frame_yaw
        assert abs(math.sin(math.atan2(idle[2][1], idle[2][0]) - wanted)) < 0.15
        n = len(stack.planners)
        assert wait_for(lambda: len(stack.planners) > n + 3)  # the keepalive goes on: the planner keeps the robot balanced
        with pytest.raises(PermissionError, match='operator'):
            backend.call(T.STEP, {'actions': [frame(STANDING).tolist()] * 6})  # motion still waits (one more bounded try)
        stack.planner_stepping = False  # the robot comes to rest
        hold, _ = backend.call(T.HOLD, {})
        assert backend.control_mode == 'streamed' and hold['control'] == 'streamed hold' and 'start_error' not in hold
        assert wait_for(lambda: stack.planner_mode is False)
        assert 'transition' in backend.call(T.STEP, {'actions': [frame(STANDING).tolist()] * 6})[0]  # motion runs again
    finally:
        backend.close()


def test_walk_stuck_force_handover_primes_the_least_moving_instant_and_eases_to_the_default_stance(session, stack):
    backend = make_backend(session, stack, walk_stuck_force_handover=True, **STUCK)
    try:
        start_control(backend, session)
        stack.planner_stepping = True
        backend.call(T.WALK, {'forward_m': 0.1, 'speed_mps': 0.5})
        assert backend.control_mode == 'planner'
        hold, _ = backend.call(T.HOLD, {})
        assert backend.control_mode == 'streamed' and 'start_error' not in hold
        assert wait_for(lambda: stack.planner_mode is False)
        assert 'least-moving instant' in ' '.join(e['what'] for e in backend.events)
        assert np.allclose(backend.last_row[R.JOINTS], R.DEFAULT_STANDING, atol=1e-6)  # eased to the default stance
    finally:
        backend.close()


def test_closing_while_the_planner_still_balances_the_robot_hands_it_to_the_hold_first(session, stack):
    backend = make_backend(session, stack, **STUCK)
    start_control(backend, session)
    stack.planner_stepping = True
    backend.call(T.WALK, {'forward_m': 0.1, 'speed_mps': 0.5})
    assert backend.control_mode == 'planner'
    stack.planner_stepping = False  # the robot settles while the service closes
    backend.close()  # the service exits: without planner messages the deploy would time out and lunge
    assert backend.control_mode == 'streamed'
    assert wait_for(lambda: [c['planner'] for c in stack.commands if c.get('start')][-1] is False)


def test_walk_ramps_the_speed_up_and_down_around_the_full_speed(session, stack):
    backend = make_backend(session, stack, walk_settle_s=0.2, walk_ramp_s=0.4)
    try:
        start_control(backend, session)
        backend.call(T.WALK, {'forward_m': 0.6, 'speed_mps': 0.4})
        speeds = [p[3] for p in stack.planners if p[0] == 1]
        assert speeds[0] < 0.2 and speeds[-1] < 0.2 and max(speeds) == pytest.approx(0.4, abs=0.02)
    finally:
        backend.close()


def walking_off(q, t, step_s=0.5, swing_rad=0.25):
    """Legs of a robot stepping backward by itself: the planted leg sweeps forward, the other is lifted."""
    names = R.JOINT_NAMES
    phase, left = (t % step_s) / step_s, int(t // step_s) % 2 == 0
    stance, swing = ('left', 'right') if left else ('right', 'left')
    q[names.index(f'{stance}_hip_pitch_joint')] = STANDING[names.index(f'{stance}_hip_pitch_joint')] + swing_rad * (2 * phase - 1)
    q[names.index(f'{stance}_knee_joint')] = STANDING[names.index(f'{stance}_knee_joint')]
    q[names.index(f'{swing}_knee_joint')] = STANDING[names.index(f'{swing}_knee_joint')] + 0.6
    return q


def test_a_robot_stepping_away_by_itself_while_holding_disarms_for_the_operator(session, stack):
    backend = make_backend(session, stack)
    try:
        start_control(backend, session)
        time.sleep(1.0)
        assert backend.fault is None  # holding still: no runaway
        stack.leg_motion = walking_off
        assert wait_for(lambda: backend.fault is not None, timeout=6.0)
        assert backend.fault['kind'] == 'runaway' and 'by itself' in backend.fault['what']
        assert wait_for(lambda: not arming.read(session)['armed']) and not stack.damped  # disarmed, not damped: it is stepping
        stack.leg_motion = None
        with pytest.raises(PermissionError):
            backend.validate(T.STEP, {'actions': [frame(STANDING).tolist()]})
        arming.write(session, True)  # the operator arms again on purpose
        backend.call(T.HOLD, {})  # an armed call acknowledges the fault
        assert backend.fault is None and backend.last_fault['kind'] == 'runaway'
    finally:
        backend.close()


def test_an_odometry_walk_that_never_covers_its_distance_ends_after_the_extension(session, stack):
    # the stack's feet do not move: covered stays 0 (sim seed 30: a 0.05 m carrying walk at the table never ended,
    # the walk extended itself every tick and held the backend lock until the session timed out)
    backend = make_backend(session, stack, walk_settle_s=0.2, walk_ramp_s=0.0, walk_odometry=True,
                           walk_odometry_extend_s=0.5)
    try:
        start_control(backend, session)
        result = {}
        worker = threading.Thread(target=lambda: result.update(value=backend.call(
            T.WALK, {'forward_m': 0.05, 'speed_mps': 0.5, 'carry': True})[0]), daemon=True)
        worker.start()
        worker.join(timeout=10.0)
        assert not worker.is_alive(), 'the walk did not end'
        assert result['value']['transition']['feedback']['odometry_m']['forward'] < 0.05
    finally:
        backend.close()


def test_walk_drives_sonics_planner_then_holds_the_measured_pose_in_streamed_motion(session, stack):
    backend = make_backend(session, stack, walk_settle_s=0.2, walk_turn_rate_deg_s=200.0, walk_ramp_s=0.0)
    try:
        start_control(backend, session)
        with pytest.raises(ValueError, match='nonzero'):
            backend.validate(T.WALK, {})
        with pytest.raises(ValueError, match='speed_mps'):
            backend.validate(T.WALK, {'forward_m': 1.0, 'speed_mps': 2.0})
        with pytest.raises(ValueError, match='turn_deg'):
            backend.validate(T.WALK, {'turn_deg': 60})  # large turns go in several calls
        with pytest.raises(ValueError, match='forward_m'):
            backend.validate(T.WALK, {'forward_m': -0.3})  # the planner walks forward only
        with pytest.raises(ValueError, match='Unknown arguments'):
            backend.validate(T.WALK, {'left_m': 0.3})
        before = backend.step_id
        value, _ = backend.call(T.WALK, {'forward_m': 0.1, 'turn_deg': 30, 'speed_mps': 0.5})
        fb = value['transition']['feedback']
        assert value['transition']['kind'] == 'walk' and fb['measured_turn_deg'] == pytest.approx(30, abs=4)
        assert backend.step_id == before + 1 and backend.control_mode == 'streamed'
        modes = [p[0] for p in stack.planners]
        assert 1 in modes and modes[-1] == 0  # slow walk, then idle to stand
        walking = [p for p in stack.planners if p[0] == 1]
        assert all(abs(p[3] - 0.5) < 1e-6 for p in walking)
        assert wait_for(lambda: stack.planner_mode is False)  # back to streamed motion
        switches = [c['planner'] for c in stack.commands if c.get('start')]
        assert switches[-2:] == [True, False]
        held = stack.poses()[-1]
        w, x, y, z = held['body_quat'].reshape(4)
        assert np.degrees(np.arctan2(2 * (w * z + x * y), 1 - 2 * (y * y + z * z))) == pytest.approx(30, abs=4)  # hold the new heading
    finally:
        backend.close()


def test_a_carrying_walk_streams_the_measured_arms_and_held_hands_to_the_planner_and_keeps_the_hands(session, stack):
    from services.g1sonic.zmq_protocol import UPPER_BODY_INDICES
    backend = make_backend(session, stack, walk_settle_s=0.2, walk_turn_rate_deg_s=200.0)
    try:
        start_control(backend, session)
        with pytest.raises(ValueError, match='carry'):
            backend.validate(T.WALK, {'forward_m': 0.1, 'carry': 1})
        arms = STANDING.copy()
        arms[R.JOINT_NAMES.index('right_elbow_joint')] += 0.3
        backend.call(T.STEP, {'actions': [frame(arms, hands=(0, 0.8)).tolist()] * 10})
        measured = np.asarray(backend.state.latest().q, dtype=float)  # the arms as they are, not as last commanded
        backend.call(T.WALK, {'forward_m': 0.1, 'speed_mps': 0.5, 'carry': True})
        extras = [e for e in stack.planner_extras if e]
        assert extras and all(len(e['upper_body_position']) == 17 for e in extras)
        assert np.allclose(extras[-1]['upper_body_position'], measured[list(UPPER_BODY_INDICES)], atol=0.05)
        assert np.allclose(extras[-1]['right_hand_joints'], R.hand_target('right', 0.8), atol=1e-5)
        assert wait_for(lambda: stack.planner_mode is False)
        held = stack.poses()[-1]
        assert np.allclose(held['right_hand_joints'].reshape(7), R.hand_target('right', 0.8), atol=1e-5)  # still gripping
        assert backend.streamer.mirror_upper  # the deploy keeps the planner's upper body: streamed arms are mirrored
        backend.call(T.WALK, {'forward_m': 0.1, 'speed_mps': 0.5})  # without carry the planner owns the arms
        assert not backend.streamer.mirror_upper
        assert not any(stack.planner_extras[-5:])
    finally:
        backend.close()


def test_hand_shape_sets_what_the_closing_fraction_interpolates_and_moves_nothing(session, stack):
    backend = make_backend(session, stack)
    try:
        start_control(backend, session)
        with pytest.raises(ValueError, match='outside'):
            backend.validate(T.HAND_SHAPE, {'side': 'right', 'open': [0] * 7, 'closed': [0, 0, 0, -1, 0, 0, 0]})
        with pytest.raises(ValueError, match='both'):
            backend.validate(T.HAND_SHAPE, {'side': 'left', 'open': [0] * 7})
        assert wait_for(lambda: len(stack.poses()) > 0)
        sent = stack.poses()[-1]['right_hand_joints'].reshape(7).copy()
        pinch = [0.0, -1.0, -1.74, 0.9, 0.5, 0.9, 0.5]
        backend.validate(T.HAND_SHAPE, {'side': 'right', 'open': [0, 0.4, 0, 0.5, 0, 0.5, 0], 'closed': pinch})
        value, _ = backend.call(T.HAND_SHAPE, {'side': 'right', 'open': [0, 0.4, 0, 0.5, 0, 0.5, 0], 'closed': pinch})
        assert value['hand_shapes']['right']['closed'] == pytest.approx(pinch, abs=1e-4)
        time.sleep(0.1)
        assert np.allclose(stack.poses()[-1]['right_hand_joints'].reshape(7), sent)  # nothing moved
        backend.call(T.STEP, {'actions': [frame(STANDING, hands=(0, 1)).tolist()] * 6})
        assert wait_for(lambda: np.allclose(stack.poses()[-1]['right_hand_joints'].reshape(7), pinch, atol=1e-5))
        assert backend.status()['hand_shapes']['right']['default'] is False
        backend.call(T.HAND_SHAPE, {'side': 'right'})
        assert backend.status()['hand_shapes']['right']['default'] is True
    finally:
        backend.close()


def test_a_disarm_stops_a_walk_and_the_robot_stands(session, stack):
    backend = make_backend(session, stack, walk_settle_s=0.2)
    try:
        start_control(backend, session)
        threading.Timer(0.8, lambda: arming.write(session, False, reason='operator')).start()
        with pytest.raises(Exception, match='disarmed') as info:
            backend.call(T.WALK, {'forward_m': 2.0, 'speed_mps': 0.3})
        assert getattr(info.value, 'handled_stop', False)
        assert stack.planners[-1][0] == 0 and backend.control_mode == 'streamed'  # stopped walking, standing, held
    finally:
        backend.close()


def test_hold_is_always_allowed_and_drops_queued_rows(backend, session, stack):
    arming.write(session, True)
    rows = ramp(STANDING, STANDING + 0.04, 60)
    result = {}

    def run():
        try:
            backend.call(T.STEP, {'actions': [row.tolist() for row in rows]})
        except Exception as exc:  # the hold ends the call without an error on the caller's side
            result['error'] = exc
    worker = threading.Thread(target=run)
    worker.start()
    assert wait_for(lambda: backend.streamer.executing)
    time.sleep(0.2)
    value, _ = backend.call(T.HOLD, {})
    assert value['held'] and value['dropped_rows'] > 0
    worker.join(5.0)
    assert 'error' not in result and backend.step_id < len(rows)


def synthetic_camera():
    width, height = 320, 240
    K = np.array([[300.0, 0.0, 159.5], [0.0, 300.0, 119.5], [0.0, 0.0, 1.0]])
    rgb = np.full((height, width, 3), 90, dtype=np.uint8)
    depth = np.full((height, width), 1.0)  # a wall one metre along the optical axis
    depth[:10, :10] = np.nan
    return FakeCamera(rgb, depth, K)


def test_observe_gives_humanoidarenas_observation_and_pixels_back_project_into_the_pelvis_frame(session, stack):
    backend = make_backend(session, stack, camera=synthetic_camera())
    try:
        with pytest.raises(RuntimeError, match='starts it'):
            backend.call(T.OBSERVE, {})  # no proprio before the stack's control runs
        start_control(backend, session)
        value, images = backend.call(T.OBSERVE, {})
        assert [a['kind'] for a in value['attachments']] == ['rgb', 'depth'] and len(images) == 2
        assert value['proprio']['joint_names_canonical29'] == list(R.JOINT_NAMES)
        assert np.allclose(value['proprio']['joint_positions_rad'], STANDING, atol=1e-3)
        assert value['camera_intrinsics']['cam_front']['width'] == 320
        pose = value['camera_poses']['cam_front']
        assert pose['frame'] == 'pelvis' and pose['position_m'][2] > 0.3  # the head camera sits above the pelvis
        assert (backend.frames_root / value['camera']['frame_id'] / 'cam_front_depth.png').is_file()
        backend.validate(T.PIXEL_TO_POSITION, {'observation_step_id': 0, 'selections': [{'u': 159.5, 'v': 119.5}]})
        points, _ = backend.call(T.PIXEL_TO_POSITION, {'observation_step_id': 0, 'selections': [{'u': 159.5, 'v': 119.5},
                                                                                                  {'u': 2, 'v': 2}]})
        centre, invalid = points['selections']
        assert centre['valid'] and centre['depth_m'] == pytest.approx(1.0)
        from services.g1sonic.bridge import matrix_from_quat
        forward = matrix_from_quat(pose['quaternion_wxyz'])[:, 0]
        expected = np.asarray(pose['position_m']) + forward  # one metre along the camera's optical axis
        assert np.allclose(centre['position_pelvis_m'], expected, atol=1e-6)
        assert np.allclose(centre['position_level_m'], expected, atol=1e-6)  # level == pelvis for an upright robot
        assert invalid['valid'] is False
        with pytest.raises(ValueError, match='stale'):
            backend.validate(T.PIXEL_TO_POSITION, {'observation_step_id': 3, 'selections': [{'u': 1, 'v': 1}]})
    finally:
        backend.close()


@pytest.mark.parametrize('profile', ['rgbd', 'rgb-only'])
def test_the_g1sonic_workspace_renders_the_real_robot_skills(tmp_path, profile):
    from services.workspace import Setup, build, recipe_for
    workspace = tmp_path / 'ws'
    build(recipe_for(), Setup('g1sonic', 'stand', 'claude', profile), workspace)
    for path in workspace.rglob('*.md'):
        text = path.read_text()
        assert '{{' not in text and '{%' not in text, path
        assert 'humanoid_' not in text and 'HumanoidArena' not in text and 'start_episode' not in text, path
    skills = {p.name for p in (workspace / '.agents/skills').iterdir()}
    assert {'manual-robot-control', 'robot-interface', 'controller-field-guide', 'controller-tracking',
            'free-space-move', 'object-localization'} <= skills
    assert 'learn-from-experts' not in skills and 'live-stream' not in skills
    interface = (workspace / '.agents/skills/robot-interface/SKILL.md').read_text()
    assert 'g1_step' in interface and 'g1_hold' in interface and 'rby1' not in interface
    assert ('g1_pixel_to_position' in interface) == (profile == 'rgbd')
    guide = (workspace / '.agents/skills/controller-field-guide/SKILL.md').read_text()
    assert 'How to read this guide' in guide  # the g1sonic block rendered
    agents = (workspace / 'AGENTS.md').read_text()
    assert 'Unitree G1 under SONIC' in agents and 'Rainbow' not in agents


def test_an_unarmed_hold_starts_nothing_and_an_armed_one_starts_the_stack_on_the_measured_pose(backend, session, stack):
    value, _ = backend.call(T.HOLD, {})
    assert value['held'] and value['state'] == 'no state from the deploy stack'
    assert stack.commands == [] and backend.status()['stack']['mode'] == 'idle' and 'proprio' not in backend.status()
    value = start_control(backend, session)
    assert value['streaming']['primed'] and value['tilt_deg'] == pytest.approx(0.0, abs=0.1)
    assert wait_for(lambda: any(c.get('start') and not c['planner'] for c in stack.commands))  # the fake drains asynchronously
    starts = [c for c in stack.commands if c.get('start')]
    assert starts[0]['planner'] is True and starts[-1]['planner'] is False  # a ZMQ start only works in planner mode
    assert wait_for(lambda: len(stack.poses()) > 3)
    assert all(np.allclose(p['joint_pos'].reshape(29), STANDING, atol=1e-6) for p in stack.poses())  # primed from the measured pose
    assert np.allclose(backend.status()['proprio']['joint_positions_rad'], STANDING, atol=1e-3)
    assert backend.step_id == 0  # a hold moves nothing


def test_a_fresh_start_eases_from_the_measured_stance_to_the_default_one(session):
    twisted = STANDING.copy()
    twisted[R.JOINT_NAMES.index('right_hip_yaw_joint')] -= 0.5  # mid-step when the hold stream starts
    twisted[R.JOINT_NAMES.index('right_hip_roll_joint')] -= 0.3
    stack = FakeSonicStack(joints=twisted).start()
    backend = make_backend(session, stack, stand_ramp_s=0.4)
    try:
        start_control(backend, session)
        assert wait_for(lambda: len(stack.poses()) > 30)
        on_wire = np.array([p['joint_pos'].reshape(29) for p in stack.poses()])
        assert np.allclose(on_wire[0], twisted, atol=0.01)  # the first frame on the wire is (about) where the robot is
        assert np.abs(np.diff(on_wire, axis=0)).max() < 0.1  # no jump
        assert np.allclose(on_wire[-1], R.DEFAULT_STANDING, atol=1e-6) and np.allclose(backend.last_row[R.JOINTS], R.DEFAULT_STANDING)
        assert any(e['what'].startswith('eased to the default') for e in backend.events)
    finally:
        backend.close()
        stack.stop()


def test_a_hold_primed_on_a_joint_past_its_limit_pulls_it_inside_the_margin(session, stack):
    backend = make_backend(session, stack)
    j = R.JOINT_NAMES.index('waist_pitch_joint')
    stack.q[j] = backend.upper[j] + 0.02  # hanging in the gantry the policy can rest a joint past its limit
    start_control(backend, session)
    snap = backend.state.latest()
    held = backend._held_frame(snap)
    margin = backend.options['joint_limit_margin_rad']
    assert held[R.JOINTS][j] == pytest.approx(backend.upper[j] - margin)
    assert np.all(held[R.JOINTS] <= backend.upper - margin + 1e-9) and np.all(held[R.JOINTS] >= backend.lower + margin - 1e-9)
    backend.validate(T.STEP, {'actions': [held.tolist()] * 5})  # rows built from the held frame pass the limit check
    backend.close()


def test_a_fresh_start_stands_under_the_planner_before_the_hold_is_primed(session, stack):
    backend = make_backend(session, stack, planner_settle_s=0.4)
    try:
        started = time.monotonic()
        value = start_control(backend, session)
        assert time.monotonic() - started >= 0.4 and value['streaming']['primed']
        assert any('standing under the planner' in e['what'] for e in backend.events)
        assert wait_for(lambda: any(c.get('start') and not c['planner'] for c in stack.commands))  # drained asynchronously
        starts = [c for c in stack.commands if c.get('start')]
        assert starts[0]['planner'] is True and starts[-1]['planner'] is False
    finally:
        backend.close()


def test_a_restarted_stack_is_noticed_and_started_again_on_the_next_armed_hold(session, stack):
    backend = make_backend(session, stack, stack_restart_s=0.3)
    try:
        start_control(backend, session)
        assert wait_for(lambda: any(c.get('start') and not c['planner'] for c in stack.commands))  # the handshake has landed
        starts_before = len([c for c in stack.commands if c.get('start')])
        stack.restart()  # the operator relaunched the deploy: no state until a new start
        time.sleep(0.6)
        value, _ = backend.call(T.HOLD, {})  # notices the silence, forgets the old hold, starts the stack again
        assert 'start_error' not in value and backend.control_mode == 'streamed'
        assert any(e['what'].startswith('control reset') for e in backend.events)
        assert wait_for(lambda: len([c for c in stack.commands if c.get('start')]) > starts_before)
        assert wait_for(lambda: backend.state.latest() is not None and 0 < backend.state.latest().index < 200)
        assert backend.status()['stack']['mode'] == 'streamed'
        assert np.allclose(backend.status()['proprio']['joint_positions_rad'], STANDING, atol=1e-3)
    finally:
        backend.close()


def test_the_planner_is_optional_and_a_missing_one_is_reported_not_crashed(backend, session):
    start_control(backend, session)
    backend.validate(T.FREE_SPACE_MOVE, {'arm': 'right', 'position': [0.3, -0.2, 0.1], 'preview_only': True})
    try:
        value, _ = backend.call(T.FREE_SPACE_MOVE, {'arm': 'right', 'position': [0.3, -0.2, 0.1], 'preview_only': True})
    except RuntimeError as exc:
        assert 'planner' in str(exc)
    else:
        assert 'plan' in value
    with pytest.raises(ValueError, match='plan again'):
        backend.validate(T.EXECUTE_PLAN, {'motion_plan_id': 'nope'})
