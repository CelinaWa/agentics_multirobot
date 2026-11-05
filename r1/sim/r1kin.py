"""R1 + PiPER kinematics in the pelvis frame (fixed base at the origin): FK of the teleop points, the gripper grasp
point and the head camera, and a damped least-squares IK that turns a grasp-point goal (+ an approach direction) into
joint angles the 5-DOF arm can actually reach. The teleop policy then gets the FK of that solution, a consistent
wrist pose, instead of a pose no arm configuration has.

Used by the sim server's tests and by the RAX r1sonic backend (pure mujoco + numpy)."""
from pathlib import Path
import numpy as np
import mujoco
from scipy.spatial.transform import Rotation

POINTS = (("left_wrist_roll_link", (0.12, 0, 0)), ("right_wrist_roll_link", (0.12, 0, 0)), ("waist_yaw_link", (0, 0, 0.37)))
ARM = {side: [f"{side}_shoulder_pitch_joint", f"{side}_shoulder_roll_joint", f"{side}_shoulder_yaw_joint",
              f"{side}_elbow_joint", f"{side}_wrist_roll_joint"] for side in ("left", "right")}
WAIST = ["waist_roll_joint", "waist_yaw_joint"]


def wxyz(R):
    return Rotation.from_matrix(R).as_quat()[[3, 0, 1, 2]]


class Kinematics:
    def __init__(self, robot_xml):
        spec = mujoco.MjSpec.from_file(str(robot_xml))
        pelvis = spec.body("pelvis")
        for j in list(pelvis.joints):        # floating base -> fixed at the origin: everything is in the pelvis frame
            spec.delete(j)
        pelvis.pos = [0, 0, 0]
        self.m = spec.compile()
        self.d = mujoco.MjData(self.m)
        m = self.m
        self.jid = {m.joint(i).name: i for i in range(m.njnt)}
        self.lower = {n: m.jnt_range[i][0] for n, i in self.jid.items()}
        self.upper = {n: m.jnt_range[i][1] for n, i in self.jid.items()}
        self.set({})

    def set(self, q):
        """q: {joint name: rad} (missing joints keep their value)."""
        for n, v in q.items():
            if n in self.jid:
                self.d.qpos[self.m.jnt_qposadr[self.jid[n]]] = v
        mujoco.mj_kinematics(self.m, self.d); mujoco.mj_comPos(self.m, self.d); mujoco.mj_camlight(self.m, self.d)

    def body(self, name):
        i = self.m.body(name).id
        return self.d.xpos[i].copy(), self.d.xmat[i].reshape(3, 3).copy()

    def site(self, name):
        i = self.m.site(name).id
        return self.d.site_xpos[i].copy(), self.d.site_xmat[i].reshape(3, 3).copy()

    def points(self, q=None):
        """The three teleop points (pos, wxyz) in the pelvis frame."""
        if q is not None:
            self.set(q)
        out = []
        for b, off in POINTS:
            p, R = self.body(b)
            out.append((p + R @ np.asarray(off), wxyz(R)))
        return out

    def camera(self, q=None):
        """Head camera pose in the pelvis frame, axes X forward (optical), Y left, Z up."""
        if q is not None:
            self.set(q)
        i = self.m.camera("head_cam").id
        Rc = self.d.cam_xmat[i].reshape(3, 3)
        return self.d.cam_xpos[i].copy(), np.column_stack([-Rc[:, 2], -Rc[:, 0], Rc[:, 1]])

    def ik(self, side, goal, approach=None, q0=None, waist=False, approach_weight=0.15, iters=200, margin=0.05,
           fingers=None, restarts=6, seed=0):
        """Joint angles putting SIDE's grasp point at GOAL (pelvis frame) with the gripper axis along APPROACH and the
        finger opening axis along FINGERS (unit vectors, optional, soft; FINGERS sign-free). Several starts (q0 first);
        returns the best (q dict, position error m, approach error deg)."""
        start = dict(q0) if q0 is not None else self.all_q()
        rng = np.random.default_rng(seed)
        best = None
        for k in range(restarts):
            q = dict(start)
            if k:
                for n in ARM[side]:
                    q[n] = float(rng.uniform(self.lower[n], self.upper[n]))
            res = self._ik(side, goal, approach, q, waist, approach_weight, iters, margin, fingers)
            if best is None or res[1] + 0.002 * res[2] < best[1] + 0.002 * best[2]:
                best = res
            if res[1] < 0.003 and res[2] < 10:
                break
        self.set({**start, **best[0]})
        return best

    def _ik(self, side, goal, approach, q0, waist, approach_weight, iters, margin, fingers):
        m, d = self.m, self.d
        if q0 is not None:
            self.set(q0)
        names = ARM[side] + (WAIST if waist else [])
        dofs = [m.jnt_dofadr[self.jid[n]] for n in names]
        qa = [m.jnt_qposadr[self.jid[n]] for n in names]
        lo = np.array([self.lower[n] + margin for n in names]); hi = np.array([self.upper[n] - margin for n in names])
        site = m.site(f"{side}_grasp").id
        jp = np.zeros((3, m.nv)); jr = np.zeros((3, m.nv))
        goal = np.asarray(goal, float)
        a_goal = None if approach is None else np.asarray(approach, float) / np.linalg.norm(approach)
        f_goal = None if fingers is None else np.asarray(fingers, float) / np.linalg.norm(fingers)
        for _ in range(iters):
            mujoco.mj_kinematics(m, d); mujoco.mj_comPos(m, d)
            p = d.site_xpos[site]; R = d.site_xmat[site].reshape(3, 3)
            mujoco.mj_jacSite(m, d, jp, jr, site)
            err = [goal - p]; J = [jp[:, dofs]]
            if a_goal is not None:
                a = R[:, 2]                   # gripper approach = grasp site z (gripper base z)
                err.append(approach_weight * np.cross(a, a_goal)); J.append(approach_weight * jr[:, dofs])
            if f_goal is not None:
                y = R[:, 1]; g = f_goal if y @ f_goal >= 0 else -f_goal
                err.append(approach_weight * np.cross(y, g)); J.append(approach_weight * jr[:, dofs])
            e = np.concatenate(err); Jm = np.vstack(J)
            if np.linalg.norm(e) < 1e-4:
                break
            dq = Jm.T @ np.linalg.solve(Jm @ Jm.T + 1e-3 * np.eye(len(e)), e)
            q = np.clip(d.qpos[qa] + np.clip(dq, -0.2, 0.2), lo, hi)
            d.qpos[qa] = q
        mujoco.mj_kinematics(m, d)
        p = d.site_xpos[site]; R = d.site_xmat[site].reshape(3, 3)
        q = {n: float(d.qpos[a]) for n, a in zip(names, qa)}
        ang = 0.0 if a_goal is None else float(np.degrees(np.arccos(np.clip(R[:, 2] @ a_goal, -1, 1))))
        return q, float(np.linalg.norm(goal - p)), ang

    def all_q(self):
        return {n: float(self.d.qpos[self.m.jnt_qposadr[i]]) for n, i in self.jid.items() if self.m.jnt_type[i] != 0}
