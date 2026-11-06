"""Walk (optional) + observe + scene report for stance planning. No arm motion.
python nav.py look
python nav.py walk <forward_m> [turn_deg]
Prints JSON: walk reply essentials (walked_m, turned_deg, pelvis odometry, tilt), then the scene: table edge (distance
along its normal, normal yaw deg: + = edge normal points left of the robot's x), paw sphere fits with overhang past
the edge, box extent. Exits 2 on tilt > 8 deg or a fall."""
import sys, json, time, math
import numpy as np
WS = '/home/junjieye/rax-runtime-r1/sessions/20261106T060138Z_puppy_box_r1_2fbfb7/agent-workspace'
sys.path.insert(0, WS)
sys.path.insert(0, WS + '/code/puppy_box')
from api.runtime import Context


def scene_report(fid):
    import scene, pawfit, edges
    out = {'frame': fid}
    try:
        b = scene.brief(fid)
        out['table_z'] = b['table_z']
        L = edges.lines(fid, b['table_z'])['lines']
        # front edge = the line with the most inliers whose normal is within 60 deg of +x
        fr = [l for l in L if abs(l['normal_yaw_deg']) < 60]
        e = max(fr, key=lambda l: l['n_in']) if fr else None
        out['edge'] = e and {'dist': e['dist'], 'yaw': e['normal_yaw_deg'], 'n': e['n_in']}
        out['edge_lines'] = [(l['dist'], l['normal_yaw_deg'], l['n_in']) for l in L]
        paws = []
        for pb in b['paw_blobs']:
            if pb['area'] < 150:
                continue
            ft = pawfit.fit(fid, *pb['pix'])
            if not ft:
                continue
            c = np.array(ft['centre'])
            d = {'pix': pb['pix'], 'c': ft['centre'], 'n': ft['n']}
            if e:
                th = math.radians(e['normal_yaw_deg'])
                nrm = np.array([math.cos(th), math.sin(th)])
                d['overhang'] = round(float(e['dist'] - nrm @ c[:2]), 3)      # >0 = paw in front of the edge
                d['along'] = round(float(np.array([-nrm[1], nrm[0]]) @ c[:2]), 3)  # + = left along the edge
            paws.append(d)
        out['paws'] = paws
        out['box'] = b.get('box')
    except Exception as ex:
        out['percep_error'] = repr(ex)
    return out


def observe(ctx):
    o = ctx.value('r1_observe')
    pr = o['proprio']
    return o['camera']['frame_id'], {'hand_r': pr['grasp_points_pelvis_m']['right'],
                                     'width_r': pr['gripper_width_m']['right'],
                                     'waist_yaw': round(pr['joint_positions_rad'][5], 3), 'tilt': o['tilt_deg'],
                                     'imu': pr['imu_orientation_wxyz']}


def main():
    ctx = Context()
    step = sys.argv[1]
    res = {'t': time.strftime('%H:%M:%S')}
    if step == 'walk':
        fwd = float(sys.argv[2])
        turn = float(sys.argv[3]) if len(sys.argv) > 3 else 0.0
        kw = {'forward_m': fwd}
        if turn:
            kw['turn_deg'] = turn
        r = ctx.value('r1_walk', **kw)
        res['walk'] = {k: v for k, v in r.items() if k not in ('proprio', 'robot_state')}
        if r.get('fallen') or (r.get('tilt_deg') or 0) > 8:
            print(json.dumps(res, default=str))
            sys.exit(2)
        time.sleep(0.5)
    fid, pr = observe(ctx)
    res['robot'] = pr
    res['scene'] = scene_report(fid)
    print(json.dumps(res, default=str))


if __name__ == '__main__':
    main()
