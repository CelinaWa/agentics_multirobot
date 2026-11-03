"""Tuck both arms by the back path (library stance.tuck) unless 'notuck', then observe and print the edge (arms tucked)."""
import sys, os, json
ROOT = os.path.abspath(os.path.join(os.path.dirname(__file__), '..', '..'))
sys.path[:0] = [ROOT, os.path.join(ROOT, 'skill-library/skills/plush-pick-place-box-same-side/code'), os.path.join(ROOT, 'code/puppy_box')]
import stance as S
import layout2
from api.runtime import Context
with Context() as ctx:
    if 'notuck' not in sys.argv:
        S.tuck(ctx)
    ob, path = S.c.observe(ctx, 'stance')
    e, tz = S.edge_of(path)
    print(json.dumps(dict(edge=round(e, 3), table_z=round(tz, 4), yaw=S.yaw_deg(ob), tilt=ob['tilt_deg'], path=path,
                          frame=ob['camera']['frame_id'], pelvis=ob['proprio'].get('pelvis_height_est_m'))))
    try:
        print(json.dumps(layout2.layout(path), default=float))
    except Exception as ex:
        print('layout failed', ex)
