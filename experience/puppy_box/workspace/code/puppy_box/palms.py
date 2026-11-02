"""Print measured palm positions (pelvis frame) of both hands, waist joints, tilt. No motion."""
import sys, os, json
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import controller as c
from api.runtime import Context

def palms(j):
    return {s: [round(v, 3) for v in c.pps.link_points(j, s)[f'{s}_palm']] for s in ('left', 'right')}

if __name__ == '__main__':
    with Context() as ctx:
        st = ctx.value('g1_status')
        j = st['proprio']['joint_positions_rad']
        print(json.dumps({'palms': palms(j), 'waist_ypr': [round(j[2], 3), round(j[8], 3), round(j[5], 3)],
                          'L_sp_el': [round(j[11], 2), round(j[21], 2)], 'tilt': st.get('tilt_deg')}))
