"""Send both arms + waist HOME and head .6 (after any operator placement/reset the backend may hold a stale command)."""
import sys
from pathlib import Path
WS = Path(__file__).resolve().parents[2]; sys.path.insert(0, str(WS))
from api.runtime import Context
ctx = Context()
H = {'left_shoulder_pitch_joint': 0.35, 'left_shoulder_roll_joint': 0.18, 'left_shoulder_yaw_joint': 0, 'left_elbow_joint': 0.87,
     'left_wrist_roll_joint': 0, 'right_shoulder_pitch_joint': 0.35, 'right_shoulder_roll_joint': -0.18,
     'right_shoulder_yaw_joint': 0, 'right_elbow_joint': 0.87, 'right_wrist_roll_joint': 0, 'waist_yaw_joint': 0,
     'waist_roll_joint': 0}
ctx.value('r1_gripper', arm='right', width_m=0.07, secs=0.5)
r = ctx.value('r1_move_joints', joints=H, secs=1.0)
r2 = ctx.value('r1_head', pitch=float(sys.argv[1]) if len(sys.argv) > 1 else 0.6, secs=1.0)
print('home tilt', r['tilt_deg'], r2['tilt_deg'])
