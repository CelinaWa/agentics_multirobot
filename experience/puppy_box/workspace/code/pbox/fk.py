import os, sys, json
sys.path.insert(0, os.path.join(os.path.dirname(__file__), '../../.agents/skills/robot-interface/scripts'))
import g1_model
M = g1_model.load()
def hand_pts(joints29, side='right'):
    P = M.fk(joints29)
    out = {}
    for n in (f'{side}_palm', f'{side}_hand_thumb_2_link', f'{side}_hand_index_1_link', f'{side}_hand_middle_1_link', f'{side}_wrist_yaw_link', f'{side}_elbow_link'):
        try: out[n] = [round(v, 3) for v in P.point(n, 'pelvis')['position_m']]
        except Exception as e: out[n] = str(e)
    return out
if __name__ == '__main__':
    j = [float(v) for v in sys.argv[1].split(',')]
    for k, v in hand_pts(j, sys.argv[2] if len(sys.argv) > 2 else 'right').items(): print(k, v)
