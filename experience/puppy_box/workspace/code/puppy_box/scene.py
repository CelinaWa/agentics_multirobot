"""Observe (no motion), save the obs json, print table z, front edge x, puppy blob stats; write a gridded image
for picking clear-box corners. Usage: python code/puppy_box/scene.py [tag] [obs.json]"""
import sys, os, json, numpy as np, cv2
HERE = os.path.dirname(os.path.abspath(__file__)); sys.path.insert(0, HERE)
import percep as p
ROOT = os.path.dirname(os.path.dirname(HERE))

def edge_x(P, tz):
    _, lev = P['_lev']
    m = (np.abs(lev[..., 2] - tz) < 0.01) & (P['depth'] > 0) & (np.abs(lev[..., 1]) < 0.15)
    return float(np.percentile(lev[..., 0][m], 1)) if m.sum() > 50 else None

def main(tag='scene', path=None):
    if path is None:
        from api.runtime import Context
        with Context() as ctx:
            ob = ctx.value('g1_observe')
        path = os.path.join(ROOT, 'output', f"sc_{tag}_{ob['step_id']}.json")
        json.dump(ob, open(path, 'w'))
    P = p.load(path); P['_lev'] = p.cloud(P)
    tz = p.table_z(P)
    pp = p.puppy(P, tz)
    img = P['img'].copy()
    for u in range(0, 640, 40):
        cv2.line(img, (u, 0), (u, 479), (0, 255, 0) if u % 200 == 0 else (0, 120, 0), 1)
        cv2.putText(img, str(u), (u + 1, 10), 0, 0.3, (0, 255, 0))
    for v in range(0, 480, 40):
        cv2.line(img, (0, v), (639, v), (0, 255, 0) if v % 200 == 0 else (0, 120, 0), 1)
        cv2.putText(img, str(v), (1, v - 1), 0, 0.3, (0, 255, 0))
    if pp is not None:
        img[pp['mask']] = (0.5 * img[pp['mask']] + [0, 0, 128]).astype(np.uint8)
    out = os.path.join(ROOT, 'output', f'sc_{tag}_grid.png'); cv2.imwrite(out, img)
    res = dict(path=path, frame=P['fid'], tilt=json.load(open(path))['tilt_deg'], table_z=tz, edge_x=edge_x(P, tz), grid=out)
    if pp is not None:
        res['puppy'] = {k: (round(v, 3) if isinstance(v, float) else v) for k, v in pp.items() if k not in ('mask', 'pts')}
    print(json.dumps(res, default=float))
    return res

if __name__ == '__main__':
    main(sys.argv[1] if len(sys.argv) > 1 else 'scene', sys.argv[2] if len(sys.argv) > 2 else None)
