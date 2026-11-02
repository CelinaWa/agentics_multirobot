"""Leaned release at the reach limit (box behind the puppy): waist pitch to LEAN in its own ramp, arm re-solved on the
commanded row, release ONLY if the measured palm is >= MIN_X; then raise the hand and unlean at once (never hold a
leaned reach). python code/puppy_box/lean_release.py LEAN MIN_X x y z   (2026-11-02 6a0bfc)"""
import sys, os, json, time
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import controller as c
from mv import move


def run(ctx, lean, min_x, target):
    out = {}
    c.pps.ramp(ctx, {'waist_pitch_joint': lean}, 4.0)
    ob = ctx.value('g1_observe'); out['tilt_lean'] = ob['tilt_deg']
    if ob['tilt_deg'] > 6:
        c.pps.ramp(ctx, {'waist_pitch_joint': 0.1}, 3.0); out['why'] = 'tilt after lean'; return out
    r = None
    for dx in (0.0, -0.01, -0.02):
        r = move(ctx, (target[0] + dx, target[1], target[2]), -10, 3.0)
        if r['ok']:
            break
    if not r['ok']:
        c.pps.ramp(ctx, {'waist_pitch_joint': 0.1}, 3.0); out['why'] = r['why']; return out
    ob = r.pop('ob'); json.dump(ob, open(f"output/lean_rel_{ob['step_id']}.json", 'w'))
    out.update(palm=r['palm'], fingers=r['fingers'], tilt=r['tilt'], frame=r['frame'], waist=r['waist'])
    if r['palm'][0] < min_x or r['tilt'] > 7:
        c.pps.ramp(ctx, {'waist_pitch_joint': 0.1}, 3.0); out['why'] = 'palm short or tilt: NOT released'; return out
    c.pps.ramp(ctx, {}, 1.5, hands=(0.0, 0.0))          # open
    time.sleep(1.0)
    ob = ctx.value('g1_observe'); json.dump(ob, open(f"output/lean_open_{ob['step_id']}.json", 'w'))
    out['frame_open'] = ob['camera']['frame_id']
    row, _, _ = c.pps.last_row(ctx)
    pa = c.pps.link_points(row[9:38], 'right')['right_palm']
    r2 = move(ctx, (pa[0] - 0.10, pa[1], pa[2] + 0.04), -10, 3.0, max_jump=1.6); r2.pop('ob', None)
    out['raise'] = r2.get('palm') or r2.get('why')
    c.pps.ramp(ctx, {'waist_pitch_joint': 0.0}, 4.0)
    out['released'] = True
    return out


if __name__ == '__main__':
    a = [float(v) for v in sys.argv[1:]]
    from api.runtime import Context
    with Context() as ctx:
        print(json.dumps(run(ctx, a[0], a[1], a[2:5]), default=float))
