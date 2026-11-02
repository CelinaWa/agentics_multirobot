"""Palm-down waypoint(s) with IK on the commanded row (library carry.waypoint), keeps waist roll/pitch of the last row.
python wp.py 'x,y,z,yaw_deg,waist_yaw;...' secs [max_jump]"""
import sys, os, json
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
L = os.path.join(os.path.dirname(os.path.abspath(__file__)), '../../skill-library/skills/plush-pick-place-box-cross-right-hand/code')
sys.path.insert(0, os.path.abspath(L))
import carry, palms
from api.runtime import Context
if __name__ == '__main__':
    wps = [[float(v) for v in w.split(',')] for w in sys.argv[1].split(';')]
    secs = float(sys.argv[2]); mj = float(sys.argv[3]) if len(sys.argv) > 3 else 1.0
    with Context() as ctx:
        for w in wps:
            r = carry.waypoint(ctx, w[:3], w[3], w[4], secs, max_jump=mj)
            print(json.dumps(r, default=float))
            if not r['ok'] or r['tilt'] > 8:
                print('STOP'); break
