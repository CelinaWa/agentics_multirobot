"""python code/puppy_box/find_puppy.py obs.json -> puppy blob (pelvis frame), body centre (densest high region),
long-axis direction and a mask image output/mask_<step>.jpg."""
import sys, os, json, numpy as np, cv2
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import percep as p

def find(path, exclude=None):
    P = p.load(path); tz = p.table_z(P)
    pp = p.puppy(P, tz, exclude=exclude)
    if pp is None:
        return None
    pel, lev = p.cloud(P)
    pts = pel[pp['mask']]; zl = lev[pp['mask']][:, 2]
    hi = pts[zl > tz + 0.075]                     # torso and head stand >7.5 cm above the table, legs do not
    c = hi[:, :2].mean(0)
    ev, evec = np.linalg.eigh(np.cov(hi[:, :2].T))
    ax = evec[:, -1]
    s = (hi[:, :2] - c) @ ax
    out = dict(step=P['ob']['step_id'], table_z_level=tz, n=len(pts), top_pelvis=float(pts[:, 2].max()),
               high_centre=c.round(3).tolist(), long_axis=ax.round(3).tolist(), long_span=[float(s.min()), float(s.max())],
               high_x=[float(np.percentile(hi[:, 0], 5)), float(np.percentile(hi[:, 0], 95))],
               high_y=[float(np.percentile(hi[:, 1], 5)), float(np.percentile(hi[:, 1], 95))])
    img = P['img'].copy(); img[pp['mask']] = (0, 0, 255)
    cv2.imwrite(f"output/mask_{out['step']}.jpg", img)
    return out

if __name__ == '__main__':
    ex = tuple(float(v) for v in sys.argv[2].split(',')) if len(sys.argv) > 2 else None
    print(json.dumps(find(sys.argv[1], ex)))
