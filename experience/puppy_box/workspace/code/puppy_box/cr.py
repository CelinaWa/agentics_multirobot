"""Session 0878bd: box LEFT, puppy RIGHT with its head LEFT (toward the box). Wraps the cross-right-hand skill code.
  python code/puppy_box/cr.py perceive      # view right (puppy) + view B (box), head=left grasp shift
"""
import sys, os, json
ROOT = os.path.abspath(os.path.join(os.path.dirname(__file__), '..', '..'))
SK = os.path.join(ROOT, 'skill-library/skills/plush-pick-place-box-cross-right-hand/code')
sys.path.insert(0, SK); sys.path.insert(0, ROOT)
import pb3, lr  # noqa
_st = pb3.load()
if 'gyaw' in _st:      # grasp palm yaw for this layout (fingers across the body axis)
    pb3.GRASP_YAW = lr.GRASP_YAW = float(_st['gyaw'])

def perceive(ctx):
    # puppy on the right -> use the 'right' view (hand up-left, waist turned right), but body centre shift for head left
    va = ((0.24, 0.10, 0.32), 20, -0.35)
    r = pb3.waypoint(ctx, va[0], va[1], va[2], 5.0, max_jump=1.6)
    if not r['ok']:
        pb3.report(False, 'view move refused: ' + r['why'])
    import time; time.sleep(1.0)
    obA, pathA = pb3.c.observe(ctx, 'cr_puppy')
    r = pb3.waypoint(ctx, (0.24, 0.08, 0.32), 20, 0.0, 5.0, max_jump=1.6)
    obB, pathB = pb3.c.observe(ctx, 'cr_box')
    pb3.step_perceive(ctx, pb3.load(), 'left', move_hand=False, paths=(pathA, pathB))

def look(ctx, waist='-0.35'):
    """Right arm to the prep pose, then tucked like the left (out of the view), waist yaw turned; observe."""
    R = lambda d: pb3.c.mirror('right', d)
    pb3.c.pps.ramp(ctx, dict(R({'shoulder_pitch': 0.1, 'elbow': -0.9, 'wrist_roll': -1.3, 'wrist_pitch': 0.0, 'wrist_yaw': 0.0,
                               'shoulder_roll': -0.2, 'shoulder_yaw': 0.0}), waist_yaw_joint=0.0, waist_roll_joint=0.0), 4.0)
    pb3.c.pps.ramp(ctx, R({'shoulder_pitch': 0.45, 'elbow': -0.5, 'wrist_roll': 0.0}), 3.0)
    pb3.c.pps.ramp(ctx, {'waist_yaw_joint': float(waist)}, 3.0)
    import time; time.sleep(1.5)
    ob, path = pb3.c.observe(ctx, 'cr_look')
    print(json.dumps({'tilt': ob['tilt_deg'], 'waist': ob['proprio']['joint_positions_rad'][2], 'path': path,
                      'frame': ob['camera']['frame_id']}))

import time
import numpy as np
# lr.step_over with the elbow allowed low (z >= 0.03) behind the table edge (x < 0.15): 0878bd layout
def over(ctx):
    st = pb3.load()
    """Arm to (grasp, z 0.30) at the turned waist via a raised via-config (shoulder pitch -1.3 halfway). The pb3 prep
    arm cannot reach there in one IK move (jump ~2 rad); best() random seeds gave an overhead branch once (sp -2.97)."""
    gx, gy = st['grasp']
    row, _, _ = pb3.c.pps.last_row(ctx); cmd = np.array(row[9:38])
    b = cmd.copy()
    for k, v in pb3.c.mirror('right', {'shoulder_pitch': 0.68, 'shoulder_roll': -0.29, 'shoulder_yaw': -1.48, 'elbow': -0.2,
                                   'wrist_roll': -0.74, 'wrist_pitch': -0.51, 'wrist_yaw': 0.26}).items():
        b[pb3.c.J.index(k)] = v
    t, e, _ = pb3.c.pps.solve(list(b), 'right', (gx, gy, 0.30), pb3.c.pps.palm_quat(pb3.GRASP_YAW, 'right'))
    if e > 0.01:
        pb3.report(False, f'no IK above the grasp (err {e:.3f})')
    q2 = cmd.copy()
    for k, v in t.items():
        q2[pb3.c.J.index(k)] = v
    I = lambda n: pb3.c.J.index(f'right_{n}_joint')
    q1 = cmd + (q2 - cmd) * 0.5; q1[I('shoulder_pitch')] = -1.3; q1[I('elbow')] = min(q1[I('elbow')], 0.0)
    lows = [lr.path_min_z(cmd, q1), lr.path_min_z(q1, q2)]
    # only the elbow may pass low (z >= 0.09, x < 0.28: in front of the puppy, right of the box)
    for z, link, p in lows:
        if z < 0.15 and not (link == 'right_elbow_link' and ((z >= 0.09 and p[0] < 0.28) or (z >= 0.03 and p[0] < 0.15))):
            pb3.report(False, f'path too low: {link} at {p}', lows=lows)
    pb3.c.pps.ramp(ctx, {k: float(q1[pb3.c.J.index(k)]) for k in t}, 4.0)
    pb3.c.pps.ramp(ctx, t, 5.0)
    time.sleep(1.5)
    ob, _ = pb3.c.observe(ctx, 'lr_over')
    palm = pb3.c.pps.link_points(ob['proprio']['joint_positions_rad'], 'right')['right_palm']
    pb3.report(ob['tilt_deg'] < 6, f"tilt {ob['tilt_deg']}", palm=palm, lows=lows, frame=ob['camera']['frame_id'])




def grasp_soft(ctx, z_min='-0.005', tilt_max='6.3'):
    """Descend 1.5 cm per 2 s from the hover; stop when the measured palm stays > 2.5 cm above its command (pressed on
    the plush) or the tilt passes tilt_max (pushing the robot); then close 3 s and check the finger stall."""
    st = pb3.load(); gx, gy = st['grasp']; z_min = float(z_min); tilt_max = float(tilt_max)
    pb3.c.pps.ramp(ctx, {}, 1.5, hands=(0.0, 0.0))
    row, _, _ = pb3.c.pps.last_row(ctx)
    z = pb3.c.pps.link_points(row[9:38], 'right')['right_palm'][2]
    hist = []; why = 'z_min'
    while z > z_min + 1e-3:
        z = max(z_min, z - 0.015)
        r = pb3.move(ctx, (gx, gy, z), pb3.GRASP_YAW, 2.0); r.pop('ob', None)
        if not r['ok']:
            why = 'refused ' + r['why']; z += 0.015; break
        hist.append([round(z, 3), round(r['palm'][2], 3), r['tilt']])
        if r['tilt'] > tilt_max:
            why = 'tilt'; break
        if r['palm'][2] > z + 0.025 and len(hist) > 1 and r['palm'][2] > hist[-2][1] - 0.005:
            why = 'stall'      # above its command AND no longer descending (a lagging wrist turn is not a stall)
            z -= 0.02          # sink 2 cm more into the soft plush before closing (run 2a: closing at the stall = fist)
            r = pb3.move(ctx, (gx, gy, z), pb3.GRASP_YAW, 2.0); r.pop('ob', None)
            hist.append([round(z, 3), round(r.get('palm', [0, 0, 0])[2], 3), r.get('tilt')])
            break
    pb3.c.pps.ramp(ctx, {}, 3.0, hands=(0.0, 1.0))
    time.sleep(1.0)
    ob, _ = pb3.c.observe(ctx, 'cr_grasp')
    f = pb3.c.fingers(ob, 'right')
    st['press_z'] = z; st['grasp_palm'] = hist[-1] if hist else None; pb3.save(st)
    pb3.report(pb3.grip(f), why, hist=hist, fingers=[round(v, 2) for v in f], tilt=ob['tilt_deg'], frame=ob['camera']['frame_id'])


def carry(ctx, step='0.06', z='0.33'):
    """Arm-only carry (waist unchanged) from the lifted palm to the release, <= step per 5 s, palm yaw gyaw -> ryaw,
    finger-stall check (middle_0 or index_0 < 1.3) and tilt < 8 after each move."""
    st = pb3.load(); step = float(step); z = float(z)
    row, _, _ = pb3.c.pps.last_row(ctx)
    start = np.array(pb3.c.pps.link_points(row[9:38], 'right')['right_palm']); start[2] = z
    end = np.array(st['release'], float)
    n = max(2, int(np.ceil(np.linalg.norm((end - start)[:2]) / step)))
    hist = []
    for k in range(1, n + 1):
        a = k / n
        pos = start + a * (end - start)
        yaw = st['gyaw'] + a * (st['ryaw'] - st['gyaw'])
        r = pb3.move(ctx, tuple(pos), yaw, 5.0, max_jump=0.8); r.pop('ob', None)
        if not r['ok']:
            pb3.report(False, r['why'], history=hist, at=pos.round(3).tolist())
        hist.append({'cmd': pos.round(3).tolist(), 'palm': r['palm'], 'fingers': r['fingers'][3:6:2], 'tilt': r['tilt']})
        if not pb3.grip(r['fingers']) or r['tilt'] > 8:
            pb3.report(False, 'grip lost or tilt on the carry', history=hist, frame=r['frame'])
    pb3.report(True, palm=r['palm'], history=hist, frame=r['frame'])

if __name__ == '__main__':
    from api.runtime import Context
    with Context() as ctx:
        {'perceive': perceive, 'look': look, 'over': over, 'grasp_soft': grasp_soft, 'carry': carry}[sys.argv[1]](ctx, *sys.argv[2:])

