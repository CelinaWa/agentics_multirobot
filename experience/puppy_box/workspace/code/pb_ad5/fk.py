"""FK of the right finger links (and left hand) from the newest saved observation json in output/."""
import sys, json, glob, os
sys.path.insert(0, 'skill-library/skills/palm-pose-servo/code')
from common import M
f = sys.argv[1] if len(sys.argv) > 1 else max(glob.glob('output/*_[0-9]*.json'), key=os.path.getmtime)
ob = json.load(open(f)); j = ob['proprio']['joint_positions_rad']; h = ob['proprio']['hand_state']['positions_rad']
P = M.fk(list(j), hand=dict(zip(M.hand_joints['right'] + M.hand_joints['left'], h[7:] + h[:7])))
for n in ['right_hand_index_1_link', 'right_hand_middle_1_link', 'right_hand_index_0_link', 'right_hand_thumb_2_link',
          'left_hand_index_1_link', 'left_wrist_yaw_link']:
    print(n, [round(float(a), 3) for a in P.frame_pose(n)[1]])
print('right hand', [round(x, 2) for x in h[7:]], 'tilt', ob.get('tilt_deg'), f)
