"""R1 sim stack: MuJoCo R1 + PiPER grippers + head RGB-D camera under SONIC_R1 100K in teleop mode (encoder mode 1),
served over ZMQ REP as the "r1 stack" protocol the RAX r1sonic backend speaks (a real-robot bridge serves the same).

  r1_sim_server.py --scene <scene_r1_dog_box.xml> --layout <layout.json> [--port 5591] [--gt out.npz]

Control (what the policy is given every 50 Hz tick): the default standing lower body, and three point targets (left
wrist, right wrist, torso) in the reference frame = the robot's standing frame at start (origin at the pelvis, x
forward, z up); the head is held at the commanded pitch (yaw 0) and the executed head action is what the history sees.
Grippers are position servos outside the policy, as on the robot (PiPER over CAN).

Requests (JSON; replies JSON, `camera` replies [json, rgb uint8 HxWx3, depth float32 HxW]):
  state                       joints (Isaac order), base quat, gyro, gripper widths, commanded targets, queue, fallen
  stream rows=[[24]...]       rows at 50 Hz: left wrist pos3 quat4(wxyz), right wrist pos3 quat4, torso pos3 quat4,
                              left gripper width m, right gripper width m, head pitch rad; replaces any queued rows
  hold                        drop the queued rows, keep the last commanded row
  camera                      head camera RGB + depth (distance along the optical axis, m) + intrinsics + camera pose in
                              the pelvis frame
  gt / place / slide_table    operator only (ground truth, object placement, sliding the table with what is on it)
"""
import argparse, json, threading, time, sys
from pathlib import Path
import numpy as np, mujoco, onnxruntime as ort, zmq
from scipy.spatial.transform import Rotation

R1 = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(R1 / "gr00t-wbc-r1"))
sys.path.insert(0, str(Path(__file__).resolve().parent))
from r1_port.runtime.observations import teleop_input, History  # noqa: E402
from r1kin import Kinematics  # noqa: E402

POINTS = (("left_wrist_roll_link", (0.12, 0, 0)), ("right_wrist_roll_link", (0.12, 0, 0)), ("waist_yaw_link", (0, 0, 0.37)))
ROW = 24
HZ, SUBSTEPS = 50, 4
STROKE = 0.035
GRIPPER_PAD = 0.004            # finger inner faces at slide 0 are 2 mm off the centre each: width = 2 * slide + 0.004
CAM_W, CAM_H = 640, 480


def rot(q):
    return Rotation.from_quat(np.asarray(q)[..., [1, 2, 3, 0]])


def wxyz(r):
    return r.as_quat()[..., [3, 0, 1, 2]]


class Sim:
    def __init__(self, scene, layout, bundle, gt_path=None, realtime=1.0, spawn_back=None):
        self.contract = json.loads((bundle / "validation/r1_deployment_contract.json").read_text())
        c = self.contract
        self.names = c["isaaclab_joint_order"]
        params = c["joint_parameters_in_isaaclab_order"]
        self.default = np.array([p["default_position_rad"] for p in params])
        self.scale = np.array([p["action_scale_rad"] for p in params])
        kp = np.array([p["kp"] for p in params]); kd = np.array([p["kd"] for p in params])
        effort = np.array([p["effort_limit_nm"] for p in params])
        self.m = mujoco.MjModel.from_xml_path(str(scene))
        m = self.m
        self.qadr = np.array([m.jnt_qposadr[m.joint(n).id] for n in self.names])
        self.vadr = np.array([m.jnt_dofadr[m.joint(n).id] for n in self.names])
        self.act = np.array([m.actuator(n.replace("_joint", "")).id for n in self.names])
        for n, p in zip(self.names, params):
            m.dof_armature[m.jnt_dofadr[m.joint(n).id]] = p["armature"]
        self.kp, self.kd, self.effort = kp, kd, effort
        self.grip_act = {s: [m.actuator(f"{s}_gripper_f7").id, m.actuator(f"{s}_gripper_f8").id] for s in ("left", "right")}
        self.grip_q = {s: [m.jnt_qposadr[m.joint(f"{s}_gripper_slide{n}").id] for n in "78"] for s in ("left", "right")}
        self.head = [self.names.index("head_pitch_joint"), self.names.index("head_yaw_joint")]
        self.session = ort.InferenceSession(str(bundle / "exports/100000/model/exported/model_step_100000_teleop.onnx"),
                                            providers=["CPUExecutionProvider"])
        self.d = mujoco.MjData(m)
        self.layout = json.loads(Path(layout).read_text()) if layout else {}
        self.spawn_back = spawn_back      # start this far behind the table edge (arms can be raised before the table comes)
        self.lock = threading.RLock()
        self.realtime = realtime
        self.gt_path, self.gt = gt_path, []
        self.renderer = None
        self.pel_id = m.body("pelvis").id
        self.pinned = {}   # body_name -> qpos7: heavy/operator-held bodies frozen during a manipulation primitive
        self.manual = False  # when set, a manipulation primitive steps the sim itself and the run thread pauses
        kinxml = Path(scene).parent / "r1_piper_cam.xml"
        if not kinxml.exists():
            cands = sorted(Path(scene).parent.glob("r1_piper*cam*.xml")) or sorted(Path(scene).parent.glob("r1_piper*.xml"))
            kinxml = cands[0] if cands else Path(scene)
        self.kin = Kinematics(kinxml)
        self._load_gait()
        self._reset_robot()

    def _load_gait(self):
        """Parametric symmetric forward-walk gait (legs only): walks straight in the body frame, starts/ends at the
        standing stance (ramped, no lurch), loopable. Amplitudes seeded from the human-walk reference. The arms keep
        tracking the streamed 3 points; the reference root advances in the robot's heading."""
        self.G_PERIOD = 52                        # ticks per full gait cycle (two steps)
        self.G_RAMP = 16                          # ticks to ramp the gait amplitude in/out (short, so brief walks still reach full stride)
        self.G_DPHASE = 2 * np.pi / self.G_PERIOD
        self.A_HIP, self.A_KNEE, self.A_ANK = 0.40, 0.55, 0.20
        self.G_SPEED = 0.009                      # m advanced per tick at full amplitude (tuned so the robot keeps up)
        n = self.names
        self.LHP, self.LKN, self.LAN = n.index("left_hip_pitch_joint"), n.index("left_knee_joint"), n.index("left_ankle_pitch_joint")
        self.RHP, self.RKN, self.RAN = n.index("right_hip_pitch_joint"), n.index("right_knee_joint"), n.index("right_ankle_pitch_joint")
        self.g_phase = 0.0; self.g_ramp = 0.0; self.g_hipscale = 1.0

    def _gait_pose(self, phase):
        """One gait pose (26-dim joint target): legs oscillate forward-walk, both legs half a cycle apart."""
        q = self.default.copy()
        for ph, hp, kn, an in ((phase, self.LHP, self.LKN, self.LAN), (phase + np.pi, self.RHP, self.RKN, self.RAN)):
            q[hp] = self.default[hp] - self.A_HIP * self.g_hipscale * np.cos(ph) * self.walk_sign   # hip-swing sign sets propulsion direction (+1 fwd, -1 back); scaled down during pure turns
            q[kn] = self.default[kn] + self.A_KNEE * max(0.0, -np.sin(ph))  # knee flexes through the swing
            q[an] = self.default[an] - self.A_ANK * np.sin(ph)
        return q

    # ----- robot --------------------------------------------------------------------------------------------------
    def _reset_robot(self):
        m, d = self.m, self.d
        mujoco.mj_resetData(m, d)
        self._reset_dog_joints()
        sx, sy, syaw = [float(v) for v in self.layout.get("spawn", "0,0,0").split(",")]
        if self.spawn_back is not None and "edge" in self.layout:
            sx = self.layout["edge"] - self.spawn_back
        self.table_shift = np.zeros(2)
        d.qpos[self.qadr] = self.default
        d.qpos[0:2] = sx, sy
        d.qpos[3:7] = wxyz(Rotation.from_euler("z", syaw, degrees=True))
        d.qpos[2] = 0.80
        mujoco.mj_forward(m, d)
        feet = [m.body(n).id for n in ("left_ankle_roll_link", "right_ankle_roll_link")]
        low = min(d.geom_xpos[i][2] - 0.03 for i in range(m.ngeom) if m.geom_bodyid[i] in feet)
        # drop until the lowest foot geom touches: settle with the policy holding the default pose
        d.qpos[2] -= max(0.0, low) - 0.002
        mujoco.mj_forward(m, d)
        for side in ("left", "right"):
            self._set_gripper(side, 0.07)
            for qa, sign in zip(self.grip_q[side], (1, -1)):
                d.qpos[qa] = sign * STROKE
        self.root_quat = wxyz(Rotation.from_euler("z", syaw, degrees=True))
        self.root_pos = d.qpos[:3].copy()
        self.hist = History()
        self.prev = np.zeros(26)
        self.queue = []
        self.row = self.measured_row()
        self.row[21] = self.row[22] = 0.07
        self.row[23] = 0.0
        self.ticks = 0
        self.fallen = False
        self.warm = 100        # ticks with the policy only, robot pinned upright by nothing: just let it settle
        self.walking = 0; self.walk_dir = np.zeros(3); self.walk_turn = 0.0; self.walk_advance = 0.0; self.gait_i = 0; self.walk_sign = 1.0

    def points_world(self):
        m, d = self.m, self.d
        out = []
        for b, off in POINTS:
            i = m.body(b).id
            R = d.xmat[i].reshape(3, 3)
            out.append((d.xpos[i] + R @ np.asarray(off), wxyz(Rotation.from_matrix(R))))
        return out

    def measured_row(self):
        """The three points now, in the reference frame, as a stream row (grippers/head from the current command)."""
        R0 = rot(self.root_quat)
        row = np.zeros(ROW)
        for k, (p, q) in enumerate(self.points_world()):
            row[7 * k:7 * k + 3] = R0.inv().apply(p - self.root_pos)
            row[7 * k + 3:7 * k + 7] = wxyz(R0.inv() * rot(q))
        if hasattr(self, "row"):
            row[21:] = self.row[21:]
        return row

    def _set_gripper(self, side, width):
        s = float(np.clip((width - GRIPPER_PAD) / 2, 0, STROKE))
        self.d.ctrl[self.grip_act[side][0]] = s
        self.d.ctrl[self.grip_act[side][1]] = -s

    def gripper_width(self, side):
        a, b = self.grip_q[side]
        return float(self.d.qpos[a] - self.d.qpos[b] + GRIPPER_PAD)

    def tick(self):
        m, d = self.m, self.d
        if self.queue:
            self.row = self.queue.pop(0)
        row = self.row
        if self.walking > 0:                                      # locomotion: symmetric gait steps straight in the body frame; root advances in the walk heading
            r = self.g_ramp
            if self.walk_turn:                                    # steer: rotate the reference heading (and the advance direction) with the turn
                self.root_quat = wxyz(rot(self.root_quat) * Rotation.from_euler("z", self.walk_turn * r))
                c, s2 = np.cos(self.walk_turn * r), np.sin(self.walk_turn * r)
                self.walk_dir[:2] = [c * self.walk_dir[0] - s2 * self.walk_dir[1], s2 * self.walk_dir[0] + c * self.walk_dir[1]]
            self.root_pos[:2] += self.walk_dir[:2] * self.G_SPEED * r * self.walk_advance
            phs = self.g_phase + np.arange(10) * 5 * self.G_DPHASE
            qg = np.array([self.default + r * (self._gait_pose(p) - self.default) for p in phs])
            dqg = np.vstack([np.zeros((1, 26)), np.diff(qg, axis=0)]) * (HZ / 5)
            self.g_phase += self.G_DPHASE
            self.gait_i += 1; self.walking -= 1
            self.g_ramp = float(min(1.0, self.gait_i / self.G_RAMP, max(0.0, self.walking / self.G_RAMP)))
        else:
            qg, dqg = np.tile(self.default, (10, 1)), np.zeros((10, 26))
        R0 = rot(self.root_quat)
        pts = np.array([R0.apply(row[7 * k:7 * k + 3]) + self.root_pos for k in range(3)])
        quats = np.array([wxyz(R0 * rot(row[7 * k + 3:7 * k + 7])) for k in range(3)])
        quat = d.qpos[3:7].copy()
        feat = teleop_input(qg, dqg, self.root_pos, self.root_quat, pts, quats, quat)
        prop = self.hist.append(dict(base_ang_vel=d.qvel[3:6], joint_pos=d.qpos[self.qadr] - self.default,
                                     joint_vel=d.qvel[self.vadr], actions=self.prev,
                                     gravity_dir=rot(quat).inv().apply([0, 0, -1])))
        act = self.session.run(None, {"obs_dict": np.r_[feat, prop].astype(np.float32)[None]})[0].ravel()
        self.prev[:] = np.clip(act, -20, 20)
        self.prev[self.head[0]] = (row[23] - self.default[self.head[0]]) / self.scale[self.head[0]]
        self.prev[self.head[1]] = 0.0
        target = self.default + self.scale * self.prev
        self._set_gripper("left", row[21]); self._set_gripper("right", row[22])
        for _ in range(SUBSTEPS):
            d.ctrl[self.act] = np.clip(self.kp * (target - d.qpos[self.qadr]) - self.kd * d.qvel[self.vadr], -self.effort, self.effort)
            mujoco.mj_step(m, d)
        if self.pinned:   # keep heavy/operator-held bodies (e.g. the ~111 kg table) fixed under the robot's touch
            for b, q7 in self.pinned.items():
                j = m.body(b).jntadr[0]; a = m.jnt_qposadr[j]; v = m.jnt_dofadr[j]
                d.qpos[a:a + 7] = q7; d.qvel[v:v + 6] = 0
        self.ticks += 1
        if d.qpos[2] < 0.45 or self.tilt_deg() > 40:
            self.fallen = True
        if self.gt_path and self.ticks % 5 == 0:
            self.gt.append(np.r_[d.time, d.qpos])

    def tilt_deg(self):
        return float(np.degrees(np.arccos(np.clip(rot(self.d.qpos[3:7]).as_matrix()[2, 2], -1, 1))))

    def walk(self, forward_m, turn_deg=0.0):
        """Walk forward_m along the current heading (negative = backward) with an optional turn. Blocks until the
        walk finishes and the robot is still, then holds the measured pose. Closed loop: the scripted gait under the
        balancing policy covers 20-140 % of the commanded distance in one pass (measured), so up to two correction
        passes follow until the residual is < 3 cm and < 6 deg. Re-observe afterwards: the robot has moved."""
        with self.lock:
            p0 = self.d.qpos[:2].copy()
            yaw0 = Rotation.from_quat(np.asarray(self.d.qpos[3:7])[[1, 2, 3, 0]]).as_euler("ZYX")[0]
            head = np.array([np.cos(yaw0), np.sin(yaw0)])
        fwd_left, turn_left = float(forward_m), float(turn_deg)
        for i in range(3):
            out = self._walk_once(fwd_left, turn_left)
            with self.lock:
                walked = float(np.dot(self.d.qpos[:2] - p0, head))
                yaw = Rotation.from_quat(np.asarray(self.d.qpos[3:7])[[1, 2, 3, 0]]).as_euler("ZYX")[0]
            turned = float(np.degrees((yaw - yaw0 + np.pi) % (2 * np.pi) - np.pi))
            fwd_left, turn_left = float(forward_m) - walked, float(turn_deg) - turned
            if out["fallen"] or (abs(fwd_left) < 0.03 and abs(turn_left) < 6.0):
                break
            if abs(fwd_left) < 0.03: fwd_left = 0.0
            elif abs(fwd_left) < 0.12: fwd_left = float(np.sign(fwd_left)) * 0.12   # shorter passes net ~0 (balance recovery cancels the step)
            if abs(turn_left) < 6.0: turn_left = 0.0
        out.update({"walked_m": round(walked, 3), "turned_deg": round(turned, 1), "passes": i + 1})
        return out

    def _walk_once(self, forward_m, turn_deg=0.0):
        """One open-loop gait pass: advance the reference root at the gait speed while the leg gait steps and the
        arms hold their streamed targets; wait until the robot is still."""
        with self.lock:
            start2d = self.d.qpos[:2].copy()
            head_yaw = Rotation.from_quat(np.asarray(self.root_quat)[[1, 2, 3, 0]]).as_euler("ZYX")[0]  # walk along the reference heading (arms stay valid in this frame)
            sgn = 1.0 if forward_m >= 0 else -1.0
            self.walk_dir = np.array([np.cos(head_yaw), np.sin(head_yaw), 0.0]) * sgn
            self.walk_sign = sgn                                      # backward: run the gait cycle in reverse (time-reversed forward walk)
            self.walk_fwd = self.walk_dir[:2].copy()
            self.walk_advance = 1.0 if abs(float(forward_m)) > 0.02 else 0.0   # 0 => turn in place
            self.g_hipscale = 1.0 if self.walk_advance else 0.30              # smaller stride when only turning (step in place)
            dist_ticks = int(round(abs(float(forward_m)) / self.G_SPEED))
            turn_ticks = int(round(abs(float(turn_deg)) / 0.8))               # ~0.8 deg/tick (~40 deg/s)
            n = max(1, max(dist_ticks + 12, turn_ticks + 2 * self.G_RAMP))
            self.walk_turn = np.radians(float(turn_deg)) / n
            self.g_phase = 0.0; self.g_ramp = 0.0; self.gait_i = 0; self.walking = n
        t0 = time.monotonic()
        while self.walking > 0 and time.monotonic() - t0 < 25:   # wait out the gait (reference advances the commanded distance, no further)
            time.sleep(0.01)
        with self.lock:
            self.walking = 0
        # settle to rest before measuring / holding: the policy keeps shuffling for ~1 s after the gait ends (measured
        # 2-3 cm and 1-5 deg of drift after a fixed 0.7 s wait), so wait until the pelvis is still for 0.5 s (max 5 s)
        t1 = time.monotonic(); still_since = None
        while time.monotonic() - t1 < 5.0:
            time.sleep(0.05)
            with self.lock:
                v = float(np.linalg.norm(self.d.qvel[0:2])); w = float(abs(self.d.qvel[5]))
            if v < 0.02 and w < 0.05:
                still_since = still_since or time.monotonic()
                if time.monotonic() - still_since >= 0.5:
                    break
            else:
                still_since = None
        with self.lock:
            disp = float(np.dot(self.d.qpos[:2] - start2d, self.walk_fwd))
            self.root_pos = self.d.qpos[:3].copy()               # snap the reference frame onto the settled robot
            self.root_quat = wxyz(Rotation.from_euler("z", Rotation.from_quat(np.asarray(self.d.qpos[3:7])[[1, 2, 3, 0]]).as_euler("ZYX")[0]))
            self.queue = []
            self.row = self.measured_row()   # keeps the gripper + head commands (a walk must not drop a held puppy or reset the camera)
            return {"requested_m": round(float(forward_m), 3), "walked_m": round(disp, 3), "turn_deg": float(turn_deg),
                    "pelvis": self.d.qpos[:3].round(3).tolist(), "tilt_deg": round(self.tilt_deg(), 1), "fallen": bool(self.fallen)}

    # ----- manipulation primitives (closed-loop; the tick thread steps the policy while these block) ----------
    GRASP_A = np.array([1.0, 0.0, -0.05]) / np.linalg.norm([1.0, 0.0, -0.05])
    GRASP_F = np.array([0.0, 1.0, 0.0])

    def _to_pel(self, p):
        with self.lock:
            Rp = self.d.xmat[self.pel_id].reshape(3, 3); pel = self.d.xpos[self.pel_id].copy()
        return Rp.T @ (np.asarray(p, float) - pel)

    def _gfk(self, side):
        with self.lock:
            q = dict(zip(self.names, self.d.qpos[self.qadr].copy()))
        self.kin.set(q); return self.kin.site(f"{side}_grasp")[0]

    def _geom_world(self, gid):
        with self.lock:
            return self.d.geom_xpos[gid].copy()

    def _queue_ramp(self, target, qidxs, n, settle=12):
        """Push a smooth ramp from the current commanded row to target; the tick thread consumes it. Blocks on the
        TICK COUNT (not wall-clock): waits until the queue is drained and `settle` ticks pass, so the servo never
        reads mid-ramp even if ONNX inference makes the run loop tick slower than HZ."""
        with self.lock:
            st = self.row.copy(); rows = []
            for k in range(n):
                s = (k + 1) / n; s = s * s * (3 - 2 * s); r = st + s * (target - st)
                for j in qidxs:
                    qa, qb = st[j:j + 4], target[j:j + 4].copy()
                    if qa @ qb < 0: qb = -qb
                    qq = (1 - s) * qa + s * qb; nr = np.linalg.norm(qq)
                    r[j:j + 4] = qq / nr if nr > 1e-9 else qa
                rows.append(r)
            if self.manual:   # deterministic execution: step the sim ourselves (the run thread is paused), like offline
                self.queue = []
                for r in rows:
                    self.row = r; self.tick()
                    if self.tilt_deg() > 22.0:   # tipping: stop pushing immediately so we hold instead of fall
                        return
                for _ in range(settle):
                    self.tick()
                return
            self.queue = list(rows)
            start = self.ticks
        deadline = time.monotonic() + (n + settle) / HZ / self.realtime + 10.0   # safety cap
        while time.monotonic() < deadline:
            with self.lock:
                if not self.queue and self.ticks - start >= n + settle:
                    return
            time.sleep(0.003)

    def _arm_to(self, side, goalw, grip, secs, servo, off=(0, 0, 0), approach=None, fingers=None, wr=0.0):
        """Move one arm's grasp site to a world goal: IK+ramp once, then servo by pushing the COMMANDED wrist point
        past the goal by 0.85x the measured error, to cancel the policy's steady-state tracking lag (matching the
        proven offline recipe). Re-IKing to the goal alone leaves the gripper short, so the creep never advances."""
        A = self.GRASP_A if approach is None else np.asarray(approach, float)
        Fn = self.GRASP_F if fingers is None else np.asarray(fingers, float)
        off = np.asarray(off, float)
        base = 0 if side == "left" else 7; gidx = 21 if side == "left" else 22; qi = base + 3; idx = 0 if side == "left" else 1
        goal = lambda: ((self._to_pel(goalw()) + off) if callable(goalw) else (self._to_pel(goalw) + off))
        g = goal()
        with self.lock:
            q0 = dict(zip(self.names, self.d.qpos[self.qadr].copy()))
        q0[f"{side}_wrist_roll_joint"] = wr   # seed the IK wrist-roll branch (0 = level jaws)
        qsol, pe, _ = self.kin.ik(side, g, A, q0=q0, fingers=Fn, approach_weight=0.45)
        if pe > 0.12:   # IK did not solve (diverged/unreachable) -> commanding it throws a garbage pose and can
            return round(float(pe) * 100, 1)   # tip the robot over; hold the current pose and let the caller move on
        pts = self.kin.points({**q0, **qsol})
        with self.lock:
            t = self.row.copy()
        t[base:base + 3] = pts[idx][0]; t[base + 3:base + 7] = pts[idx][1]
        if grip is not None: t[gidx] = grip
        self._queue_ramp(t, [qi], int(secs * HZ))
        cmd = t.copy()
        for _ in range(servo):
            e = goal() - self._gfk(side)
            if np.linalg.norm(e) < 0.006:
                break
            cmd[base:base + 3] = cmd[base:base + 3] + 0.85 * e
            self._queue_ramp(cmd, [qi], 16)
        return round(float(pe) * 100, 1)

    def _set_grip_wait(self, side, width, ticks=70):
        gidx = 21 if side == "left" else 22
        with self.lock:
            self.queue = []; self.row = self.row.copy(); self.row[gidx] = width
            if self.manual:
                for _ in range(ticks):
                    self.tick()
                return
            start = self.ticks
        deadline = time.monotonic() + ticks / HZ / self.realtime + 5.0
        while time.monotonic() < deadline:
            with self.lock:
                if self.ticks - start >= ticks:
                    return
            time.sleep(0.003)

    def grasp_paw(self, cmdopen=0.05, wedge=0.06, liftz=0.20):
        """Grasp the nearest hind paw (ground-truth located) with the side-appropriate arm via a width-feedback
        wedge, then lift it partially. Blocks. Returns which paw/side/arm, the clamped width, lift, and held flag."""
        with self.lock:
            self.manual = True   # step the sim deterministically ourselves (settle + grasp), like offline
            self.queue = []
            # DAMP THE WALK TRANSIENT before grasping. The robot is a dynamic balancing policy: it never comes to
            # rest, it holds a low-amplitude limit cycle (~0.15 rad/s). After the continuous-time pre-walk/slide it
            # carries a LARGE transient on top of that cycle, and it takes ~500 ticks to damp down to the steady
            # cycle. Grasping before it damps catches a high-variance pose and misses (seed27 flipped pass/fail with
            # a 130-tick settle); settling to the steady cycle first makes the capture reliable across runs.
            for _ in range(600):
                self.tick()
            paw_ids = {n: self.m.geom(f"dog_paw_{n}").id for n in range(4)}
            pos = {n: self.d.geom_xpos[paw_ids[n]].copy() for n in range(4)}
            order = sorted(range(4), key=lambda n: pos[n][0])       # nearest (lowest x) first
            a = order[0]; nearest = a; pely = self.d.xpos[self.pel_id][1]
            # grip the near leg PAIR (wider, more robust to the robot's balancing variance than a single thin leg):
            # the nearest paw + its closest-in-y neighbour, if within the ~7 cm jaw span
            b = None   # single-leg grip (pair-grip tested worse live); form closure is the real robustness fix
            ga = paw_ids[a]; gb = paw_ids[b] if b is not None else None
            p0 = pos[a].copy(); dog0z = self.d.xpos[self.m.body("dog").id][2]
        side = "right" if p0[1] < pely else "left"
        paw_ahead = float(self._to_pel(p0)[0])
        if paw_ahead > 0.36:   # reach guard: leaning further to reach a far paw tipped the robot over (RAX session 1, tilt 81 deg)
            with self.lock:
                self.manual = False; self.queue = []
            return {"paw": int(nearest), "side": side, "wedged": False, "held": False,
                    "clamped_width_m": round(float(self.gripper_width(side)), 4), "lifted_m": 0.0,
                    "tilt_deg": round(self.tilt_deg(), 1), "fallen": bool(self.fallen),
                    "refused": f"nearest hind paw is {paw_ahead:.2f} m ahead of the pelvis, beyond the safe reach (0.35 m): walk closer before grasping"}
        if gb is not None:   # target between the two legs, at the x of the farther paw so both legs are present there
            def paww():
                with self.lock:
                    pa = self.d.geom_xpos[ga].copy(); pb = self.d.geom_xpos[gb].copy()
                return np.array([max(pa[0], pb[0]) + 0.005, (pa[1] + pb[1]) / 2, (pa[2] + pb[2]) / 2])
        else:
            paww = lambda: self._geom_world(ga)
        self._pin("table")   # the heavy table must not drift under the robot's touch while grasping
        # torso counter-lean: the reaching arm shifts the CoM forward + toward its side, which tips the robot in
        # ~1/2 of startup basins and the gripper lands off the leg. Pre-lean the torso point back + away so the
        # policy keeps balance through the reach (3-point teleop torso channel). Then settle in the leaned pose.
        with self.lock:
            self.row[14] -= 0.030                                   # torso back (−x): counter the reach
            self.row[15] += (0.025 if side == "right" else -0.025)  # torso away from the reaching arm
            for _ in range(80):
                self.tick()
        openw = cmdopen
        TILT_ABORT = 16.0   # deg: stop reaching before an awkward pose tips the robot over (safety)
        # ADAPTIVE lean: the right lean is a per-basin balance-vs-reach trade-off. Approach the leg; if the lean
        # left the gripper short of it (can't reach), ease the torso forward and retry until it can reach.
        for _ in range(4):
            self._arm_to(side, paww, openw, 1.3, 4, off=(-0.05, 0, 0))
            re = np.linalg.norm(self._to_pel(paww()) + np.array([-0.05, 0, 0]) - self._gfk(side))
            if re < 0.028 or self.tilt_deg() > TILT_ABORT:
                break
            with self.lock:                                         # gripper short -> ease torso forward, regain reach
                self.row[14] += 0.013
                for _ in range(45):
                    self.tick()
        # VARIANT RETRY: the grasp is deterministic given the state, so retry with different grasp geometry
        # (approach down-tilt az, how far the jaws creep onto the leg xmax, and a y-offset to cover the jaws being
        # shifted off the leg by the robot's balancing pose) until one holds through a test-lift.
        VARIANTS = [(-0.05, 0.065, 0.0), (-0.12, 0.065, 0.0), (-0.05, 0.090, 0.0), (0.02, 0.065, 0.0),
                    (-0.05, 0.065, 0.015), (-0.05, 0.065, -0.015), (-0.12, 0.090, 0.015), (-0.12, 0.090, -0.015),
                    (-0.20, 0.050, 0.0), (0.02, 0.090, 0.0)]
        grabbed = held = False; cw = 0.0; lifted = 0.0
        try:
            for vi, (az, xmax, yo) in enumerate(VARIANTS):
                if self.tilt_deg() > TILT_ABORT:
                    self._arm_to(side, paww, 0.074, 1.0, 1, off=(-0.12, yo, 0), approach=self.GRASP_A); break
                A = np.array([1.0, 0.0, az]); A = A / np.linalg.norm(A)
                self._set_grip_wait(side, openw, 20)   # open for this attempt
                self._arm_to(side, paww, openw, 1.6 if vi == 0 else 1.0, 5, off=(-0.055, yo, 0), approach=A)
                if self.tilt_deg() > TILT_ABORT:   # reaching this pose is tipping us; retreat and abort (no fall)
                    self._arm_to(side, paww, 0.074, 1.0, 2, off=(-0.12, yo, 0.05), approach=self.GRASP_A); break
                xoff = -0.055
                while xoff < xmax - 1e-6:
                    xoff = min(xoff + 0.012, xmax)
                    self._arm_to(side, paww, openw, 0.6, 2, off=(xoff, yo, 0), approach=A)
                    if self.gripper_width(side) >= wedge:
                        break
                self._set_grip_wait(side, 0.0, 70)
                cw = self.gripper_width(side)
                import os as _os
                if _os.environ.get("GDBG"):
                    try:
                        suf = "c2"
                        p7 = self._geom_world(self.m.geom(f"{side}_gripper_finger7_{suf}").id)
                        p8 = self._geom_world(self.m.geom(f"{side}_gripper_finger8_{suf}").id)
                        oax = self._to_pel(p7) - self._to_pel(p8); oax = oax/ (np.linalg.norm(oax)+1e-9)
                        re = np.linalg.norm(self._to_pel(paww()) - self._gfk(side))
                        with self.lock:
                            cs=[(self.m.geom(self.d.contact[i].geom1).name or "",self.m.geom(self.d.contact[i].geom2).name or "") for i in range(self.d.ncon)]
                        tc=sorted(set(x for pr in cs if (f"{side}_gripper" in pr[0]+pr[1]) for x in pr if x and x.startswith("dog")))
                        print(f"[gd] v{vi} az={az:+.2f} yo={yo:+.3f} reach={re*100:.1f}cm open_axis={oax.round(2).tolist()} (z-tilt={oax[2]:+.2f}) cw={cw:.3f} dogcontacts={tc[:3]}", flush=True)
                    except Exception as _e: print("[gd] err", repr(_e)[:80], flush=True)
                if cw <= 0.01:
                    continue   # closed on nothing; next variant
                grabbed = True
                heldpoint = paww(); held = True
                for dz in np.arange(0.04, liftz + 1e-6, 0.04):
                    self._arm_to(side, heldpoint, 0.0, 1.5, 2, off=(0, 0, float(dz)), approach=A)
                    if self.gripper_width(side) < 0.01:
                        held = False; break
                if held:
                    with self.lock:
                        lifted = float((self.d.xpos[self.m.body("dog").id][2] - dog0z))
                    break   # success: keep holding
                # slipped: lower back and release before the next variant
                self._arm_to(side, heldpoint, 0.074, 1.0, 1)
        finally:
            self._unpin("table")
            with self.lock:
                self.manual = False; self.queue = []   # keep self.row (the holding command) so the run thread holds the puppy
        return {"paw": int(nearest), "side": side, "wedged": bool(grabbed), "clamped_width_m": round(float(cw), 4),
                "held": bool(held), "lifted_m": round(lifted, 3), "tilt_deg": round(self.tilt_deg(), 1),
                "fallen": bool(self.fallen)}

    def edge_grasp(self, cmdopen=0.05, wedge=0.06, liftz=0.22, reach_ahead=0.28, do_overhang=True, lean=0):
        """EDGE-OVERHANG grasp via the SHARED oh_grasp_core -- byte-identical to the validated standalone full_pipe_oh,
        so the live server reproduces the standalone grasp (instead of a reimplementation the chaotic grasp diverges on)."""
        from oh_grasp_core import oh_grasp
        with self.lock:
            self.manual = True; self.queue = []
        try:
            res = oh_grasp(self, self.kin, float(cmdopen), float(wedge), float(liftz),
                           float(reach_ahead) if reach_ahead is not None else 0.28, bool(do_overhang), int(lean))
        finally:
            self._unpin("table", "box")
            with self.lock:
                self.manual = False; self.queue = []
        res["tilt_deg"] = round(self.tilt_deg(), 1); res["fallen"] = bool(self.fallen)
        return res

    def _pin(self, *bodies):
        with self.lock:
            for b in bodies:
                j = self.m.body(b).jntadr[0]; a = self.m.jnt_qposadr[j]
                self.pinned[b] = self.d.qpos[a:a + 7].copy()

    def _unpin(self, *bodies):
        with self.lock:
            for b in (bodies or list(self.pinned)):
                self.pinned.pop(b, None)

    def place_in_box(self, side="auto", release=True):
        """With a paw held, bring the hand over the box interior (ground-truth box pose; the operator should already
        have slid the box under the hand if it was far), lower, and release so the puppy drops in. Blocks.
        side='auto' uses whichever gripper is currently closed on something (the holding arm)."""
        if side == "auto":
            wl, wr = self.gripper_width("left"), self.gripper_width("right")
            side = "left" if wl < wr else "right"
        with self.lock:
            bx = self.d.xpos[self.m.body("box").id].copy()
        boxtop = float(bx[2] + 0.14)
        with self.lock:
            self.manual = True
        self._pin("table", "box")   # keep the operator-positioned box (and table) put while lowering in
        try:
            self._arm_to(side, np.r_[bx[:2], boxtop + 0.08], 0.0, 2.2, 3)
            self._arm_to(side, np.r_[bx[:2], boxtop + 0.02], 0.0, 2.0, 2)
            if release:
                self._set_grip_wait(side, 0.074, 90)
                self._arm_to(side, np.r_[bx[0] - 0.05, bx[1], boxtop + 0.16], 0.074, 1.6, 1)
        finally:
            self._unpin("table", "box")
            with self.lock:
                self.manual = False; self.queue = []
        with self.lock:
            dg = self.d.xpos[self.m.body("dog").id].copy()
        inside = bool(abs(dg[0] - bx[0]) < 0.134 and abs(dg[1] - bx[1]) < 0.19 and bx[2] - 0.02 < dg[2] < bx[2] + 0.16)
        return {"in_box": inside, "dog": dg.round(3).tolist(), "box": bx.round(3).tolist(),
                "tilt_deg": round(self.tilt_deg(), 1), "fallen": bool(self.fallen)}

    def run(self, stop):
        period = 1.0 / HZ / self.realtime
        nxt = time.monotonic()
        while not stop.is_set():
            with self.lock:
                if not self.manual:          # a manipulation primitive is stepping the sim itself; don't double-step
                    self.tick()
            nxt += period
            delay = nxt - time.monotonic()
            if delay > 0:
                time.sleep(delay)
            else:
                nxt = time.monotonic()      # behind (a render paused physics): sim time does not catch up

    # ----- requests -----------------------------------------------------------------------------------------------
    def state(self):
        d = self.d
        q = d.qpos[self.qadr]; dq = d.qvel[self.vadr]
        R0 = rot(self.root_quat)
        return {"t": round(float(d.time), 4), "tick": self.ticks, "joint_names": self.names,
                "q": q.tolist(), "dq": dq.tolist(),
                # the IMU: orientation relative to the reference frame (heading at start = 0), gyro in the pelvis frame
                "base_quat_wxyz": wxyz(R0.inv() * rot(d.qpos[3:7])).tolist(), "gyro": d.qvel[3:6].tolist(),
                "gripper_width_m": {s: round(self.gripper_width(s), 4) for s in ("left", "right")},
                "command_row": np.round(self.row, 5).tolist(), "queued": len(self.queue),
                "measured_points_row": np.round(self.measured_row()[:21], 5).tolist(),
                "fallen": self.fallen, "tilt_deg": round(self.tilt_deg(), 2), "mode": "teleop"}

    def stream(self, rows):
        rows = [np.asarray(r, dtype=float) for r in rows]
        if any(r.shape != (ROW,) or not np.all(np.isfinite(r)) for r in rows):
            raise ValueError(f"rows must be finite and {ROW} long")
        for r in rows:
            for k in range(3):
                r[7 * k + 3:7 * k + 7] /= np.linalg.norm(r[7 * k + 3:7 * k + 7])
        self.queue = rows
        return {"queued": len(rows)}

    def camera(self):
        m, d = self.m, self.d
        if self.renderer is None:
            self.renderer = mujoco.Renderer(m, CAM_H, CAM_W)
        cam = m.camera("head_cam").id
        opt = mujoco.MjvOption()
        opt.geomgroup[:] = [1, 1, 1, 0, 1, 0]
        self.renderer.update_scene(d, camera=cam, scene_option=opt)
        rgb = self.renderer.render().copy()
        opt.geomgroup[4] = 0                   # clear plastic walls: invisible to the depth sensor (the rim is not)
        self.renderer.enable_depth_rendering()
        self.renderer.update_scene(d, camera=cam, scene_option=opt)
        depth = self.renderer.render().astype(np.float32)   # distance along the optical axis
        self.renderer.disable_depth_rendering()
        depth[depth > 6.0] = 0.0
        fovy = np.radians(m.cam_fovy[cam]); f = CAM_H / 2 / np.tan(fovy / 2)
        K = [[f, 0, CAM_W / 2], [0, f, CAM_H / 2], [0, 0, 1]]
        pel = m.body("pelvis").id
        Rp = d.xmat[pel].reshape(3, 3); Rc = d.cam_xmat[cam].reshape(3, 3)
        pos = Rp.T @ (d.cam_xpos[cam] - d.xpos[pel])
        # camera frame X forward (optical axis), Y left, Z up  <- MuJoCo camera looks along -z with y up, x right
        axes = np.column_stack([-Rc[:, 2], -Rc[:, 0], Rc[:, 1]])
        meta = {"t": float(d.time), "intrinsics": K, "width": CAM_W, "height": CAM_H,
                "pose_pelvis": {"position_m": pos.tolist(), "quaternion_wxyz": wxyz(Rotation.from_matrix(Rp.T @ axes)).tolist()},
                "q": d.qpos[self.qadr].tolist()}
        return meta, rgb, depth

    def gt_report(self):
        m, d = self.m, self.d
        out = {}
        for b in ("pelvis", "dog", "box", "table", "right_gripper_base", "left_gripper_base"):
            i = m.body(b).id
            out[b] = {"pos": d.xpos[i].round(4).tolist(), "quat": d.xquat[i].round(4).tolist()}
        site = m.site("right_grasp").id
        out["right_grasp"] = d.site_xpos[site].round(4).tolist()
        out["dog_parts"] = [d.xpos[m.body(n).id].round(4).tolist() for n in ("dog",)]
        out["dog_paws"] = [d.geom_xpos[m.geom(f"dog_paw_{k}").id].round(4).tolist() for k in range(4)]
        out["dog_hips"] = [d.xpos[m.body(f"dog_limb_{k}").id].round(4).tolist() for k in range(4)]
        out["dog_limb_q"] = {f"{k}_{j}": round(float(d.qpos[m.jnt_qposadr[m.joint(f"dog_limb_{k}_{j}").id]]), 3)
                             for k in range(4) for j in ("swing", "splay")}
        out["dog_limb_spring"] = {f"{k}_{j}": round(float(m.qpos_spring[m.jnt_qposadr[m.joint(f"dog_limb_{k}_{j}").id]]), 3)
                                  for k in range(4) for j in ("swing", "splay")}
        _ci = m.camera("head_cam").id   # true camera pose + the right grasp site, for calibration checks
        out["head_cam_world"] = {"pos": d.cam_xpos[_ci].round(4).tolist(),
                                 "xmat": d.cam_xmat[_ci].round(4).tolist()}
        out["right_grasp_world"] = d.site_xpos[m.site("right_grasp").id].round(4).tolist()
        out["fallen"] = self.fallen
        out["tilt_deg"] = round(self.tilt_deg(), 2)
        out["layout"] = self.layout
        out["table_edge_ahead_m"], out["table_top_z"] = self.table_edge_ahead()
        out["table_shift"] = self.table_shift.round(4).tolist()
        box = m.body("box").id
        out["box_quat"] = d.xquat[box].round(4).tolist()
        return out

    def place(self, body, pos, quat=(1, 0, 0, 0)):
        m, d = self.m, self.d
        j = m.body(body).jntadr[0]
        a, v = m.jnt_qposadr[j], m.jnt_dofadr[j]
        d.qpos[a:a + 3] = pos; d.qpos[a + 3:a + 7] = quat; d.qvel[v:v + 6] = 0
        mujoco.mj_forward(m, d)

    def slide_table(self, dx, dy=0.0):
        """Slide the table and everything on it (the operator moves the table, not the robot)."""
        m, d = self.m, self.d
        for body in ("table", "box", "dog"):
            j = m.body(body).jntadr[0]
            a, v = m.jnt_qposadr[j], m.jnt_dofadr[j]
            d.qpos[a] += dx; d.qpos[a + 1] += dy; d.qvel[v:v + 6] = 0
        self.table_shift += (dx, dy)
        mujoco.mj_forward(m, d)

    def _reset_dog_joints(self):
        """A fresh plush: every puppy limb/neck/tail joint back to 0, and the spring rests the operator set (dog_joint)
        cleared, so an arranged leg cannot prop the re-placed puppy up like a kickstand on the next reset."""
        m, d = self.m, self.d
        dog = m.body("dog").id
        for j in range(m.njnt):
            b = m.jnt_bodyid[j]
            if m.jnt_type[j] == mujoco.mjtJoint.mjJNT_FREE or m.body_rootid[b] != dog or b == dog:
                continue
            a, v = m.jnt_qposadr[j], m.jnt_dofadr[j]
            d.qpos[a] = 0.0; d.qvel[v] = 0.0; m.qpos_spring[a] = 0.0

    def reset_scene(self):
        """Puppy and box back where the layout put them on the table (wherever the table now is)."""
        m, d = self.m, self.d
        self._reset_dog_joints()
        for body, key in (("dog", "place"), ("box", "place_box")):
            f = [float(v) for v in self.layout[key].split("=")[1].split(",")[1:]]
            self.place(body, [f[0] + self.table_shift[0], f[1] + self.table_shift[1], f[2]], f[3:7])

    def table_edge_ahead(self):
        """How far the table's front edge is ahead of the pelvis along the heading (operator side, ground truth)."""
        m, d = self.m, self.d
        top = m.geom("table_top").id
        edge_x = d.geom_xpos[top][0] - m.geom_size[top][0]
        pel = d.xpos[m.body("pelvis").id]
        R = d.xmat[m.body("pelvis").id].reshape(3, 3)
        heading = np.arctan2(R[1, 0], R[0, 0])
        return float((edge_x - pel[0]) / max(0.3, np.cos(heading))), float(d.geom_xpos[top][2] + m.geom_size[top][2])

    def save_gt(self):
        if self.gt_path and self.gt:
            np.savez_compressed(self.gt_path, frames=np.array(self.gt), layout=json.dumps(self.layout))


def serve(sim, port, stop):
    ctx = zmq.Context.instance()
    sock = ctx.socket(zmq.REP)
    sock.bind(f"tcp://127.0.0.1:{port}")
    poller = zmq.Poller(); poller.register(sock, zmq.POLLIN)
    last_save = time.monotonic()
    while not stop.is_set():
        if time.monotonic() - last_save > 30:
            with sim.lock:
                sim.save_gt()
            last_save = time.monotonic()
        if not poller.poll(200):
            continue
        req = json.loads(sock.recv())
        op = req.get("op")
        try:
            if op == "walk":                       # handled outside the lock: it blocks while the tick thread (which holds the lock) steps
                out = sim.walk(float(req["forward_m"]), float(req.get("turn_deg", 0.0)))
                sock.send(json.dumps({"ok": True, **out}).encode()); continue
            if op == "grasp_paw":                  # blocking manipulation primitives (outside the lock, like walk)
                out = sim.grasp_paw(float(req.get("cmdopen", 0.05)), float(req.get("wedge", 0.06)), float(req.get("liftz", 0.20)))
                sock.send(json.dumps({"ok": True, **out}).encode()); continue
            if op == "place_in_box":
                out = sim.place_in_box(req["side"], bool(req.get("release", True)))
                sock.send(json.dumps({"ok": True, **out}).encode()); continue
            if op == "edge_grasp":
                out = sim.edge_grasp(float(req.get("cmdopen", 0.05)), float(req.get("wedge", 0.06)),
                                     float(req.get("liftz", 0.22)), req.get("reach_ahead"), bool(req.get("do_overhang", False)), int(req.get("lean", 0)))
                sock.send(json.dumps({"ok": True, **out}).encode()); continue
            with sim.lock:
                if op == "camera":
                    meta, rgb, depth = sim.camera()
                    sock.send_multipart([json.dumps({"ok": True, **meta}).encode(), rgb.tobytes(), depth.tobytes()])
                    continue
                if op == "state":
                    out = sim.state()
                elif op == "stream":
                    out = sim.stream(req["rows"])
                elif op == "hold":
                    sim.queue = []; out = {"held": True}
                elif op == "gt":
                    out = sim.gt_report()
                elif op == "place":
                    sim.place(req["body"], req["pos"], req.get("quat", (1, 0, 0, 0))); out = {}
                elif op == "reset":
                    sim._reset_robot(); out = {}
                elif op == "reset_scene":
                    sim.reset_scene(); out = {}
                elif op == "slide_table":
                    sim.slide_table(float(req["dx"]), float(req.get("dy", 0.0))); out = {}
                elif op == "dog_joint":     # operator arranges a plush limb: set a dog joint AND its spring rest so it holds there
                    j = sim.m.joint(req["name"]).id; a = sim.m.jnt_qposadr[j]
                    sim.d.qpos[a] = float(req["value"]); sim.m.qpos_spring[a] = float(req["value"]); sim.d.qvel[sim.m.jnt_dofadr[j]] = 0.0
                    mujoco.mj_forward(sim.m, sim.d); out = {"name": req["name"], "value": float(req["value"])}
                elif op == "contacts":      # diagnostics: every contact on a gripper (geoms, normal force, penetration, friction)
                    out = {"contacts": []}
                    f6 = np.zeros(6)
                    for i in range(sim.d.ncon):
                        c = sim.d.contact[i]
                        n1, n2 = sim.m.geom(c.geom1).name or "", sim.m.geom(c.geom2).name or ""
                        if "gripper" not in n1 + n2: continue
                        mujoco.mj_contactForce(sim.m, sim.d, i, f6)
                        out["contacts"].append({"geoms": [n1, n2], "normal_N": round(float(f6[0]), 3),
                                                "tangent_N": round(float(np.hypot(f6[1], f6[2])), 3), "dist_m": round(float(c.dist), 5),
                                                "mu": round(float(c.friction[0]), 3), "dim": int(c.dim)})
                    for s_ in ("left", "right"):
                        out[f"{s_}_ctrl"] = [round(float(sim.d.ctrl[k]), 4) for k in sim.grip_act[s_]]
                        out[f"{s_}_act_force"] = [round(float(sim.d.actuator_force[k]), 3) for k in sim.grip_act[s_]]
                        out[f"{s_}_width"] = round(sim.gripper_width(s_), 4)
                elif op == "head":          # command head pitch (rad, + = look down); held like the real r1_head
                    sim.row[23] = float(np.clip(req["pitch"], -0.62, 0.62)); out = {"head_pitch": float(sim.row[23])}
                elif op == "shutdown":
                    stop.set(); out = {}
                else:
                    raise ValueError(f"unknown op {op!r}")
            sock.send(json.dumps({"ok": True, **out}).encode())
        except Exception as exc:  # noqa: BLE001  (the reply carries it; the stack keeps running)
            sock.send(json.dumps({"ok": False, "error": f"{type(exc).__name__}: {exc}"}).encode())
    with sim.lock:
        sim.save_gt()


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--scene", type=Path, required=True)
    ap.add_argument("--layout", type=Path)
    ap.add_argument("--bundle", type=Path, default=R1 / "SONIC_R1_100k")
    ap.add_argument("--port", type=int, default=5591)
    ap.add_argument("--gt", type=Path)
    ap.add_argument("--realtime", type=float, default=1.0, help="sim speed relative to wall time")
    ap.add_argument("--spawn-back", type=float, default=None, help="start this far behind the table edge")
    a = ap.parse_args()
    sim = Sim(a.scene, a.layout, a.bundle, a.gt, a.realtime, a.spawn_back)
    stop = threading.Event()
    t = threading.Thread(target=sim.run, args=(stop,), daemon=True); t.start()
    print(json.dumps({"ready": True, "port": a.port, "layout": sim.layout}), flush=True)
    try:
        serve(sim, a.port, stop)
    except KeyboardInterrupt:
        stop.set()
    t.join(2)


if __name__ == "__main__":
    main()
