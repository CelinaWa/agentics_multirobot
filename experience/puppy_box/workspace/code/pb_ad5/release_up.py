"""Gated leaned release that lets go cleanly (ad5d0f): lean in its own ramp, palm to the target, open only if the measured palm is
>= MIN_X, switch to the FLAT default hand shape (the pinch 'open' shape stays curled and can hook the plush), open, wait 2 s, raise
STRAIGHT UP 10 cm, then back, then unlean. lean_release.py moved the hand back 10 cm 1 s after opening the curled hand: the puppy
ended on the floor (attempt 5). Usage: release_up.py LEAN MIN_X x y z"""
import sys, os, json, time
sys.path.insert(0, 'skill-library/skills/plush-pick-place-box-behind/code')
import lean_release as L
c, move = L.c, L.move


def run(ctx, lean, min_x, target):
    out = {}
    c.pps.ramp(ctx, {'waist_pitch_joint': lean}, 4.0)
    ob = ctx.value('g1_observe'); out['tilt_lean'] = ob['tilt_deg']
    if ob['tilt_deg'] > 6:
        c.pps.ramp(ctx, {'waist_pitch_joint': 0.1}, 3.0); out['why'] = 'tilt after lean'; return out
    r = None
    for dx in (0.0, -0.01, -0.02):
        r = move(ctx, (target[0] + dx, target[1], target[2]), 0, 3.0)
        if r['ok']:
            break
    if not r['ok']:
        c.pps.ramp(ctx, {'waist_pitch_joint': 0.1}, 3.0); out['why'] = r['why']; return out
    r.pop('ob', None); out.update(palm=r['palm'], fingers=r['fingers'], tilt=r['tilt'], frame=r['frame'])
    if r['palm'][0] < min_x or r['tilt'] > 7:
        c.pps.ramp(ctx, {'waist_pitch_joint': 0.1}, 3.0); out['why'] = 'palm short or tilt: NOT released'; return out
    ctx.value('g1_hand_shape', side='right')                 # flat default open
    c.pps.ramp(ctx, {}, 1.5, hands=(0.0, 0.0))
    time.sleep(2.0)
    ob = ctx.value('g1_observe'); out['frame_open'] = ob['camera']['frame_id']
    row, _, _ = c.pps.last_row(ctx)
    pa = c.pps.link_points(row[9:38], 'right')['right_palm']
    r2 = move(ctx, (pa[0], pa[1], pa[2] + 0.10), 0, 3.0, max_jump=1.6); r2.pop('ob', None)
    out['up'] = r2.get('palm') or r2.get('why')
    r3 = move(ctx, (pa[0] - 0.10, pa[1], pa[2] + 0.10), 0, 3.0, max_jump=1.6); r3.pop('ob', None)
    out['back'] = r3.get('palm') or r3.get('why')
    c.pps.ramp(ctx, {'waist_pitch_joint': 0.0}, 4.0)
    out['released'] = True
    return out


if __name__ == '__main__':
    a = [float(v) for v in sys.argv[1:]]
    from api.runtime import Context
    with Context() as ctx:
        print(json.dumps(run(ctx, a[0], a[1], a[2:5]), default=float))
