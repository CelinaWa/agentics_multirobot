"""The R1's controller for the two-robot office world: the SONIC_R1 policy and the r1 stack protocol, without physics.

office_sim.py owns the physics. Once per control period it sends this process the whole state; this process mirrors
it into its own copy of the model, runs the R1 policy tick (the same code as the single-robot R1 sim server,
r1/sim/r1_sim_server.py: reference frame, streamed rows, gait, head, grippers) and answers with the R1's joint
targets and gripper commands. Beside that it serves the r1 stack protocol (state, stream, hold, walk, camera, head)
that RAX's r1sonic backend speaks, so the backend needs no change.

  MUJOCO_GL=osmesa /opt/r1venv/bin/python r1_brain.py --scene <abs office_g1_r1.xml> [--tick-port 5695] [--port 5691]
"""
import argparse, json, sys, threading
from pathlib import Path
import numpy as np, mujoco, onnxruntime as ort, zmq
from scipy.spatial.transform import Rotation

R1 = Path('/home/junjieye/workspace/multirobot/r1')          # the SONIC_R1 bundle and the r1_port runtime (lab checkout)
sys.path.insert(0, str(Path(__file__).resolve().parents[1] / 'r1' / 'sim'))   # this repository's R1 sim server
sys.path.insert(0, str(R1 / 'gr00t-wbc-r1'))
import r1_sim_server as S  # noqa: E402
from r1_sim_server import rot, wxyz  # noqa: E402

PREFIX = 'r1_'


class _NoStep:
    """mujoco, except that mj_step does nothing: the policy tick of the single-robot server ends by stepping its own
    physics, and here the physics belongs to office_sim.py."""

    def __getattr__(self, name):
        return (lambda *a, **k: None) if name == 'mj_step' else getattr(mujoco, name)


class _Rooted:
    """An array in which the first `n` entries are the R1's floating base, wherever it sits in the real array: the
    single-robot code reads its root as qpos[0:7] / qvel[0:6]; joint entries are addressed absolutely and pass through."""

    def __init__(self, array, base, n):
        self.array, self.base, self.n = array, base, n

    def _key(self, key):
        if isinstance(key, (int, np.integer)):
            return int(key) + self.base if key < self.n else key
        if isinstance(key, slice) and key.stop is not None and key.stop <= self.n:
            return slice((key.start or 0) + self.base, key.stop + self.base, key.step)
        if isinstance(key, slice):
            raise IndexError('only the root part of the R1 state can be sliced')
        return key

    def __getitem__(self, key):
        return self.array[self._key(key)]

    def __setitem__(self, key, value):
        self.array[self._key(key)] = value


class _Data:
    """The mirror MjData as the single-robot code expects it: the R1's root first."""

    def __init__(self, data, root_q, root_v):
        object.__setattr__(self, '_d', data)
        object.__setattr__(self, 'qpos', _Rooted(data.qpos, root_q, 7))
        object.__setattr__(self, 'qvel', _Rooted(data.qvel, root_v, 6))

    def __getattr__(self, name):
        return getattr(self._d, name)


class Brain(S.Sim):
    def __init__(self, scene, bundle):
        c = self.contract = json.loads((bundle / 'validation/r1_deployment_contract.json').read_text())
        self.names = c['isaaclab_joint_order']
        params = c['joint_parameters_in_isaaclab_order']
        self.default = np.array([p['default_position_rad'] for p in params])
        self.scale = np.array([p['action_scale_rad'] for p in params])
        self.kp = np.array([p['kp'] for p in params]); self.kd = np.array([p['kd'] for p in params])
        self.effort = np.array([p['effort_limit_nm'] for p in params])
        self.armature = np.array([p['armature'] for p in params])
        m = self.m = mujoco.MjModel.from_xml_path(str(scene))
        self.real = mujoco.MjData(m)
        self.qadr = np.array([m.jnt_qposadr[m.joint(PREFIX + n).id] for n in self.names])
        self.vadr = np.array([m.jnt_dofadr[m.joint(PREFIX + n).id] for n in self.names])
        self.act = np.array([m.actuator(PREFIX + n.replace('_joint', '')).id for n in self.names])
        self.grip_act = {s: [m.actuator(f'{PREFIX}{s}_gripper_f7').id, m.actuator(f'{PREFIX}{s}_gripper_f8').id] for s in ('left', 'right')}
        self.grip_q = {s: [m.jnt_qposadr[m.joint(f'{PREFIX}{s}_gripper_slide{n}').id] for n in '78'] for s in ('left', 'right')}
        self.head = [self.names.index('head_pitch_joint'), self.names.index('head_yaw_joint')]
        self.head_yaw = self.head_yaw_goal = 0.0      # the head yaw command (op head), slewed in the tick
        self.session = ort.InferenceSession(str(bundle / 'exports/100000/model/exported/model_step_100000_teleop.onnx'),
                                            providers=['CPUExecutionProvider'])
        root = m.body(PREFIX + 'pelvis').jntadr[0]
        self.root_q, self.root_v = int(m.jnt_qposadr[root]), int(m.jnt_dofadr[root])
        self.d = _Data(self.real, self.root_q, self.root_v)
        self.pel_id = m.body(PREFIX + 'pelvis').id
        self.layout, self.lock, self.realtime = {}, threading.RLock(), 1.0
        self.gt_path, self.gt, self.renderer, self.pinned, self.manual = None, [], None, {}, False
        self._load_gait()
        self._start_pose()

    def _start_pose(self):
        """The R1 standing on its default stance where the scene put it; the mirror holds it until the first state."""
        m, d = self.m, self.real
        mujoco.mj_resetData(m, d)
        start = d.qpos[self.root_q:self.root_q + 7].copy()
        yaw = rot(start[3:7]).as_euler('ZYX')[0]
        d.qpos[self.qadr] = self.default
        d.qpos[self.root_q + 2] = 0.80
        mujoco.mj_forward(m, d)
        feet = [m.body(PREFIX + n).id for n in ('left_ankle_roll_link', 'right_ankle_roll_link')]
        low = min(d.geom_xpos[i][2] - 0.03 for i in range(m.ngeom) if m.geom_bodyid[i] in feet)
        d.qpos[self.root_q + 2] -= max(0.0, low) - 0.002
        for side in ('left', 'right'):
            self._set_gripper(side, 0.07)
            for qa, sign in zip(self.grip_q[side], (1, -1)):
                d.qpos[qa] = sign * S.STROKE
        mujoco.mj_forward(m, d)
        self.root_quat = wxyz(Rotation.from_euler('z', yaw))
        self.root_pos = d.qpos[self.root_q:self.root_q + 3].copy()
        self.hist = S.History(); self.prev = np.zeros(26); self.queue = []
        self.row = self.measured_row(); self.row[21] = self.row[22] = 0.07; self.row[23] = 0.0
        self.ticks, self.resets, self.fallen = 0, 0, False
        self.walking = 0; self.walk_dir = np.zeros(3); self.walk_turn = 0.0; self.walk_advance = 0.0; self.gait_i = 0; self.walk_sign = 1.0
        self.table_shift = np.zeros(2)

    def hello(self):
        d = self.real
        init = [(int(a), float(d.qpos[a])) for a in (*range(self.root_q, self.root_q + 7), *self.qadr,
                                                    *(q for s in ('left', 'right') for q in self.grip_q[s]))]
        grip = [k for s in ('left', 'right') for k in self.grip_act[s]]
        return {'qadr': self.qadr.tolist(), 'vadr': self.vadr.tolist(), 'act': self.act.tolist(), 'kp': self.kp.tolist(),
                'kd': self.kd.tolist(), 'effort': self.effort.tolist(), 'armature': self.armature.tolist(),
                'default': self.default.tolist(), 'grip_act': grip, 'init_grip': [float(d.ctrl[k]) for k in grip],
                'init_qpos': init, 'substeps': S.SUBSTEPS}

    def exchange(self, raw):
        """One control period: mirror the world's state, run the policy tick, return joint targets + gripper commands."""
        m, d = self.m, self.real
        state = np.frombuffer(raw, dtype=np.float64)
        with self.lock:
            d.time = state[0]; d.qpos[:] = state[1:1 + m.nq]; d.qvel[:] = state[1 + m.nq:1 + m.nq + m.nv]
            mujoco.mj_forward(m, d)
            self.tick()
            target = self.default + self.scale * self.prev
            grip = [d.ctrl[k] for s in ('left', 'right') for k in self.grip_act[s]]
        return np.concatenate((target, grip)).astype(np.float64).tobytes()

    # ----- the parts of the single-robot server that name things in the model -------------------------------------
    def points_world(self):
        m, d = self.m, self.real
        out = []
        for body, offset in S.POINTS:
            i = m.body(PREFIX + body).id
            R = d.xmat[i].reshape(3, 3)
            out.append((d.xpos[i] + R @ np.asarray(offset), wxyz(Rotation.from_matrix(R))))
        return out

    def camera(self):
        """The head camera, rendered from a private copy of the state with the tick lock released: a render takes
        0.1-0.3 s on the CPU, and holding the lock for it stalls the whole world (the G1 deploy then loses its robot)."""
        m = self.m
        if self.renderer is None:
            self.renderer = mujoco.Renderer(m, S.CAM_H, S.CAM_W)
            self.cam_data = mujoco.MjData(m)
        d = self.cam_data
        d.qpos[:] = self.real.qpos; d.time = self.real.time
        q = self.real.qpos[self.qadr].tolist()
        self.lock.release()                    # the caller (the stack server) holds it once
        try:
            mujoco.mj_forward(m, d)
            cam = m.camera(PREFIX + 'head_cam').id
            self.renderer.update_scene(d, camera=cam)
            rgb = self.renderer.render().copy()
            self.renderer.enable_depth_rendering()
            self.renderer.update_scene(d, camera=cam)
            depth = self.renderer.render().astype(np.float32)
            self.renderer.disable_depth_rendering()
        finally:
            self.lock.acquire()
        depth[depth > 6.0] = 0.0
        f = S.CAM_H / 2 / np.tan(np.radians(m.cam_fovy[cam]) / 2)
        Rp = d.xmat[self.pel_id].reshape(3, 3); Rc = d.cam_xmat[cam].reshape(3, 3)
        axes = np.column_stack([-Rc[:, 2], -Rc[:, 0], Rc[:, 1]])
        meta = {'t': float(d.time), 'intrinsics': [[f, 0, S.CAM_W / 2], [0, f, S.CAM_H / 2], [0, 0, 1]], 'width': S.CAM_W, 'height': S.CAM_H,
                'pose_pelvis': {'position_m': (Rp.T @ (d.cam_xpos[cam] - d.xpos[self.pel_id])).tolist(),
                                'quaternion_wxyz': wxyz(Rotation.from_matrix(Rp.T @ axes)).tolist()},
                'q': q}
        return meta, rgb, depth

    def _reset_robot(self, pose=None):
        """The stack protocol's `reset`, here: the operator moved the standing R1 by hand (office_sim's place key). Its
        reference frame starts again from where it now stands, on the default stance, arms where they are. With POSE
        (x, y, yaw_deg) the reference goes there at once, BEFORE the robot is placed: left at the old heading while
        the pelvis was turned, the policy turned the robot back within 0.5 s (a 120 deg placement ended 25 deg from
        the old heading)."""
        d = self.real
        self.root_pos = d.qpos[self.root_q:self.root_q + 3].copy()
        self.root_quat = wxyz(Rotation.from_euler('z', rot(d.qpos[self.root_q + 3:self.root_q + 7]).as_euler('ZYX')[0]))
        grip, head = self.row[21:23].copy(), self.row[23]
        self.queue = []
        self.row = self.measured_row()[:S.ROW]
        self.row[21:23], self.row[23] = grip, head
        if pose is not None:     # after the row: its points are relative to the robot, and stay so at the new pose
            self.root_pos[:2] = [float(pose[0]), float(pose[1])]
            self.root_quat = wxyz(Rotation.from_euler('z', np.radians(float(pose[2]))))
        self.walking = 0
        self.resets += 1
        if d.qpos[self.root_q + 2] > 0.5 and self.tilt_deg() < 20:
            self.fallen = False       # a person stood it up again (S40: the flag stayed set and the session stayed down)

    def save_gt(self):
        pass


def main():
    ap = argparse.ArgumentParser(description=__doc__.split('\n\n')[0])
    ap.add_argument('--scene', type=Path, required=True)
    ap.add_argument('--bundle', type=Path, default=R1 / 'SONIC_R1_100k')
    ap.add_argument('--tick-port', type=int, default=5695)
    ap.add_argument('--port', type=int, default=5691, help='the r1 stack protocol port (RAX r1sonic backend: stack_port)')
    a = ap.parse_args()
    S.mujoco = _NoStep()
    brain = Brain(a.scene, a.bundle)
    with brain.lock:
        brain.camera()                         # creates the renderer now, not during the first observation
    stop = threading.Event()
    threading.Thread(target=S.serve, args=(brain, a.port, stop), daemon=True).start()
    ticks = zmq.Context.instance().socket(zmq.REP)
    ticks.bind(f'tcp://127.0.0.1:{a.tick_port}')
    print(json.dumps({'ready': True, 'tick_port': a.tick_port, 'stack_port': a.port}), flush=True)
    while not stop.is_set():
        if not ticks.poll(200):
            continue
        raw = ticks.recv()
        ticks.send(json.dumps(brain.hello()).encode() if raw == b'hello' else brain.exchange(raw))


if __name__ == '__main__':
    main()
