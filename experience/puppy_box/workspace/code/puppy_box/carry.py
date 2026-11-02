"""Carry waypoints that change the commanded waist yaw and the arm together (IK solved on the commanded row with
the NEW waist command, since SONIC tracks the whole-body pose). python carry.py 'x,y,z,palm_yaw,waist_yaw;...' [secs]"""
import sys, os, json, time
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import controller as c
from reach2 import best


def waypoint(ctx, pos, yaw_deg, waist, secs=3.5, side='right', max_err=0.012, max_jump=1.0):
    row, _, _ = c.pps.last_row(ctx)
    cmd = list(row[9:38]); cmd[2] = waist
    t, err, _ = c.pps.solve(cmd, side, pos, c.pps.palm_quat(yaw_deg, side))
    if err > max_err:
        err, t = best(cmd, pos, yaw_deg, side)
    jump = max(abs(t[k] - row[9 + c.J.index(k)]) for k in t)
    if err > max_err or jump > max_jump:
        return {'ok': False, 'why': f'refused: IK err {err:.3f} jump {jump:.2f}'}
    t = dict(t, waist_yaw_joint=waist)
    rep = c.pps.ramp(ctx, t, secs)
    time.sleep(0.5)
    ob = ctx.value('g1_observe')
    j = ob['proprio']['joint_positions_rad']
    json.dump(ob, open(f"output/carry_{ob['step_id']}.json", 'w'))
    return {'ok': True, 'cmd': list(pos), 'waist_cmd': waist, 'palm': c.pps.link_points(j, side)[f'{side}_palm'],
            'fingers': [round(v, 2) for v in c.fingers(ob, side)], 'tilt': ob['tilt_deg'], 'waist': round(j[2], 3),
            'frame': ob['camera']['frame_id'], 'step_id': ob['step_id'],
            'track': rep['transition']['feedback'].get('max_joint_tracking_error_rad')}


if __name__ == '__main__':
    wps = [[float(v) for v in w.split(',')] for w in sys.argv[1].split(';')]
    secs = float(sys.argv[2]) if len(sys.argv) > 2 else 3.5
    from api.runtime import Context
    with Context() as ctx:
        for w in wps:
            r = waypoint(ctx, w[:3], w[3], w[4], secs)
            print(json.dumps(r, default=float))
            if not r['ok'] or min(r['fingers'][3], r['fingers'][5]) > 1.3 or r['tilt'] > 8:
                print('STOP'); break
