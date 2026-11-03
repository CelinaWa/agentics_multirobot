"""Offline replay of run.near_hand (plush still on the open hand?) on archived frames.
python code/puppy_box/replay_release.py <frame_id> ..."""
import sys, os, json
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import run


def ob_from_frame(fid):
    m = json.load(open(f'runtime/frames/{fid}/meta.json'))
    j = [m['joints'][k] for k in run.c.J]
    return {'step_id': 'replay_' + fid, 'camera': {'frame_id': fid}, 'camera_intrinsics': {'cam_front': {'matrix': m['intrinsics_full']}},
            'camera_poses': {'cam_front': m['camera_pose_pelvis']},
            'proprio': {'joint_positions_rad': j, 'root_orientation_world_wxyz': [1, 0, 0, 0]}}


if __name__ == '__main__':
    for fid in sys.argv[1:]:
        print(fid, run.near_hand(ob_from_frame(fid), {}))
