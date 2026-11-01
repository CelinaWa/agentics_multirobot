"""Claw pre-grasp with a path check: interpolate joints from the current command to the IK solution and
require every tip to stay above zmin (pelvis frame) before moving; then the skill's servo refines."""
import time
import numpy as np
from sh import g1mod, ik, servo
import run
from g1 import IDX


def path_min_z(base, a, side, n=8):
    zs = []
    for s in np.linspace(0, 1, n):
        j = dict(base)
        j.update({k: base[k] + (a[k] - base[k]) * s for k in a})
        t = ik.tips(j, side)
        zs.append(min(t[k][2] for k in ('palm', 'thumb_tip', 'index_tip', 'middle_tip')))
    return min(zs)


def claw_to(g, side, target, clearance, tilt=0.52, yaw=0.0, zmin=0.13, iters=2):
    g.observe()
    base = g.joints()
    R = ik.claw_rotation(side, tilt=tilt, yaw=yaw)
    palm = run.claw_palm_for_target(side, target, clearance, tilt, yaw)
    a, pe, re = ik.solve_multi(side, base, palm, R)
    if pe > 0.025:
        return {'ok': False, 'why': 'ik_err %.3f' % pe}
    zm = path_min_z(base, a, side)
    if zm < zmin:
        return {'ok': False, 'why': 'path dips to z %.3f < %.3f' % (zm, zmin)}
    g.move(a, secs=3.0)
    time.sleep(0.6)
    hist = servo.servo(g, list(palm), side=side, iters=iters, secs=2.0, R=R, max_jump=0.8)
    g.observe()
    t = ik.tips(g.joints(), side)
    return {'ok': hist[-1]['err'] < 0.03, 'hist': hist, 'tips': t, 'frame': g.obs['camera']['frame_id'],
            'tilt': g.obs.get('tilt_deg'), 'bias': servo.current_bias(g, side)}
