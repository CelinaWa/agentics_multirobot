"""Observe once, save the JSON, print a compact summary (frame path, tilt, palms in pelvis frame)."""
import json, sys, os
sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..', '..'))
sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..', '..', 'skill-library/skills/palm-pose-servo/code'))
from api.runtime import Context
from common import link_points

def observe(ctx, tag='obs'):
    ob = ctx.value('g1_observe')
    sid = ob.get('step_id')
    path = f'output/{tag}_{sid}.json'
    json.dump(ob, open(path, 'w'))
    j = ob['proprio']['joint_positions_rad']
    s = {'step_id': sid, 'frame': f"runtime/frames/{ob['camera']['frame_id']}", 'tilt': ob.get('tilt_deg'),
         'R': link_points(j, 'right')['right_palm'], 'L': link_points(j, 'left')['left_palm'], 'json': path}
    return ob, s

if __name__ == '__main__':
    with Context() as ctx:
        ob, s = observe(ctx)
        print(json.dumps(s))
        print('keys', list(ob.keys()))
