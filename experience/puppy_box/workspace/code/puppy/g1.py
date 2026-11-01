"""Small helpers for puppy_box: observe, build semantic40 rows from the last frame, measure hands.

Usage from a script or `python -i`:
    from g1 import G1
    g = G1(); o = g.observe(); g.move({'left_shoulder_pitch_joint': -0.3}, secs=1.5)
"""
import json
import math
import os
import sys
import time

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.abspath(os.path.join(HERE, '..', '..'))
sys.path.insert(0, ROOT)
sys.path.insert(0, os.path.join(ROOT, '.agents/skills/robot-interface/scripts'))

from api.runtime import Context  # noqa: E402
import g1_model  # noqa: E402

NAMES = g1_model.MODEL.canonical29
IDX = {n: i for i, n in enumerate(NAMES)}
MAX_STEP = 0.10          # rad per tick, below the backend's 0.15
LO = [-2.5307, -2.5307, -2.618, -0.5236, -2.9671, -0.52, -2.7576, -2.7576, -0.52, -0.0873, -0.0873, -3.0892, -3.0892, -0.8727, -0.8727, -1.5882, -2.2515, -0.2618, -0.2618, -2.618, -2.618, -1.0472, -1.0472, -1.9722, -1.9722, -1.6144, -1.6144, -1.6144, -1.6144]
HI = [2.8798, 2.8798, 2.618, 2.9671, 0.5236, 0.52, 2.7576, 2.7576, 0.52, 2.8798, 2.8798, 2.6704, 2.6704, 0.5236, 0.5236, 2.2515, 1.5882, 0.2618, 0.2618, 2.618, 2.618, 2.0944, 2.0944, 1.9722, 1.9722, 1.6144, 1.6144, 1.6144, 1.6144]
LOG = os.path.join(ROOT, 'output', 'puppy_log.jsonl')


def short(name):
    return name.replace('_joint', '')


class G1:
    def __init__(self, ctx=None):
        self.ctx = ctx or Context()
        self.last = None   # last 40-row we sent (or held)
        self.obs = None

    def log(self, **kw):
        kw['t'] = time.time()
        with open(LOG, 'a') as f:
            f.write(json.dumps(kw) + '\n')

    # ---------------------------------------------------------------- state
    def observe(self):
        self.obs = self.ctx.value('g1_observe')
        os.makedirs(os.path.join(ROOT, 'output', 'obs'), exist_ok=True)
        with open(os.path.join(ROOT, 'output', 'obs', 'last.json'), 'w') as f:
            json.dump(self.obs, f)
        return self.obs

    def joints(self, obs=None):
        o = obs or self.obs
        return dict(zip(o['proprio']['joint_names_canonical29'], o['proprio']['joint_positions_rad']))

    def hand(self, side, obs=None):
        o = obs or self.obs
        hs = o['proprio']['hand_state']
        return [p for n, p in zip(hs['joint_names'], hs['positions_rad']) if n.startswith(side)]

    def pose(self, obs=None):
        return g1_model.pose_from_observation(obs or self.obs)

    def points(self, frame='pelvis', obs=None):
        p = self.pose(obs)
        out = {}
        for name in p.names():
            if any(k in name for k in ('palm', 'hand', 'thumb', 'index', 'middle', 'camera', 'wrist')):
                out[name] = [round(v, 3) for v in p.point(name, frame)['position_m']]
        return out

    def pix(self, uvs, r=2):
        sid = self.obs['step_id']
        res = self.ctx.value('g1_pixel_to_position', observation_step_id=sid,
                             selections=[{'u': u, 'v': v} for u, v in uvs], depth_window_radius=r)
        return [[round(x, 3) for x in s['position_pelvis_m']] if s.get('valid') else None
                for s in res['selections']]

    def base_row(self):
        if self.last is not None:
            return list(self.last)
        st = self.ctx.value('g1_status')
        last = (st.get('command') or {}).get('last_row')
        if last:
            self.last = list(last)
            return list(last)
        o = self.obs or self.observe()
        j = o['proprio']['joint_positions_rad']
        row = [0.0, 0.0, 0.78, 1, 0, 0, 1, 0, 0] + list(j) + [0.0, 0.0]
        self.last = row
        return list(row)

    # ---------------------------------------------------------------- motion
    def move(self, targets, secs=1.5, hands=None, tail=10, check=True):
        """Linear ramp from the last frame to `targets` (joint name or short name -> rad), hands=(l, r)."""
        start = self.base_row()
        end = list(start)
        for k, v in targets.items():
            name = k if k in IDX else k + '_joint'
            i = IDX[name]
            end[9 + i] = min(max(float(v), LO[i] + 0.06), HI[i] - 0.06)
        if hands is not None:
            if hands[0] is not None:
                end[38] = float(hands[0])
            if hands[1] is not None:
                end[39] = float(hands[1])
        maxd = max(abs(a - b) for a, b in zip(start[9:38], end[9:38]))
        n = max(int(secs * 50), int(maxd / MAX_STEP) + 1, 2)
        segs = max(1, -(-n // 90))
        per = -(-n // segs)
        r = None
        for k in range(segs):
            rows = []
            for i in range(1, per + 1):
                s = (k * per + i) / (segs * per)
                s = 0.5 - 0.5 * math.cos(math.pi * s)   # smooth
                rows.append([a + (b - a) * s for a, b in zip(start, end)])
            rows += [list(rows[-1])] * (tail if k == segs - 1 else 5)
            r = self.ctx.value('g1_step', actions=rows)
            self.last = list(rows[-1])
            fb = self.feedback(r)
            self.log(kind='move', targets=targets, hands=hands, fb=fb, step_id=r.get('step_id'))
            if fb.get('fallen') or (fb.get('tilt_deg') or 0) > 12:
                raise RuntimeError('tilt/fall: %s' % fb)
        return r

    def feedback(self, r):
        fb = (r.get('transition') or {}).get('feedback', {})
        return {k: fb.get(k) for k in ('tilt_deg', 'joint_tracking_rmse_rad', 'max_joint_tracking_error_rad',
                                         'reference_error_rmse_rad', 'fallen')}

    def report(self, side='left'):
        j = self.joints()
        keys = ['waist_yaw', 'waist_pitch', side + '_shoulder_pitch', side + '_shoulder_roll',
                side + '_shoulder_yaw', side + '_elbow', side + '_wrist_roll', side + '_wrist_pitch',
                side + '_wrist_yaw']
        cmd = self.last
        out = {}
        for k in keys:
            m = j[k + '_joint']
            c = cmd[9 + IDX[k + '_joint']] if cmd else None
            out[k] = (round(m, 3), None if c is None else round(c, 3))
        return out
