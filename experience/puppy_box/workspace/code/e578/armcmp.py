"""No-motion: commanded vs measured waist + right arm joints."""
import sys, os, json
W = os.getcwd()
for d in ('skill-library/skills/palm-pose-servo/code', 'code/puppy_box', '.'):
    sys.path.insert(0, os.path.join(W, d))
import controller as c
from api.runtime import Context
with Context() as ctx:
    row, meas, _ = c.pps.last_row(ctx)
for n in ['waist_yaw_joint', 'waist_roll_joint', 'waist_pitch_joint'] + [f'right_{k}_joint' for k in ('shoulder_pitch', 'shoulder_roll', 'shoulder_yaw', 'elbow', 'wrist_roll', 'wrist_pitch', 'wrist_yaw')] + ['left_shoulder_pitch_joint', 'left_elbow_joint']:
    i = c.J.index(n); print(f'{n:28s} cmd {row[9 + i]:+.2f} meas {meas[i]:+.2f}')
