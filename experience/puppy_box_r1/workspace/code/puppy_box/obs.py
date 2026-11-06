"""Observe once (head pitch kept) and print scene geometry. python obs.py [head_pitch]"""
import sys, json, subprocess
sys.path.insert(0, '/home/junjieye/rax-runtime-r1/sessions/20261106T010905Z_puppy_box_r1_f6b331/agent-workspace')
from api.runtime import Context
ctx = Context()
if len(sys.argv) > 1:
    ctx.value('r1_head', pitch=float(sys.argv[1]), secs=1.0)
o = ctx.value('r1_observe')
fid = o['camera']['frame_id']
pr = o['proprio']
print(json.dumps({'frame': fid, 'step': o['step_id'], 'tilt': o['tilt_deg'], 'grip': pr['gripper_width_m'],
                  'gp': pr['grasp_points_pelvis_m']}))
print(subprocess.run([sys.executable, '/home/junjieye/rax-runtime-r1/sessions/20261106T010905Z_puppy_box_r1_f6b331/agent-workspace/code/puppy_box/scene.py', fid], capture_output=True,
                     text=True).stdout)
