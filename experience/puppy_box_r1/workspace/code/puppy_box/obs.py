"""Observe once (head pitch kept) and print scene geometry. python obs.py [head_pitch]"""
import sys, json, subprocess
sys.path.insert(0, '/home/junjieye/workspace/multirobot/real_g1_release/rax/runtime/sessions/20261105T222431Z_puppy_box_r1_e0ac0c/agent-workspace')
from api.runtime import Context
ctx = Context()
if len(sys.argv) > 1:
    ctx.value('r1_head', pitch=float(sys.argv[1]), secs=1.0)
o = ctx.value('r1_observe')
fid = o['camera']['frame_id']
pr = o['proprio']
print(json.dumps({'frame': fid, 'step': o['step_id'], 'tilt': o['tilt_deg'], 'grip': pr['gripper_width_m'],
                  'gp': pr['grasp_points_pelvis_m']}))
print(subprocess.run([sys.executable, '/tmp/claude-1001/ws_e0ac0c/code/puppy_box/scene.py', fid], capture_output=True,
                     text=True).stdout)
