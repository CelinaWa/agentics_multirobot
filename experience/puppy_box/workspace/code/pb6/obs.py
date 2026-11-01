"""Observe and print a compact summary (tilt, heading yaw, frame id)."""
import sys, json, math
sys.path.insert(0, '.')
from api.runtime import Context
ctx = Context()
o = ctx.value('g1_observe')
w, x, y, z = o['proprio']['root_orientation_world_wxyz']
yaw = math.degrees(math.atan2(2 * (w * z + x * y), 1 - 2 * (y * y + z * z)))
print(json.dumps({'frame': o['camera']['frame_id'], 'step': o['step_id'], 'tilt': o['tilt_deg'], 'yaw_deg': round(yaw, 1),
                  'hand': o['proprio']['hand_state']['positions_rad'],
                  'cam': o['camera_poses']['cam_front']['position_m']}))
