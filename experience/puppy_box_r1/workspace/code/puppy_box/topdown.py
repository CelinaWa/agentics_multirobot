"""Top-down render of a frame's point cloud (pelvis frame), coloured by RGB, with a 5 cm grid.
python topdown.py <frame> [x0 x1 y0 y1] [zmin]  -> output/td_<frame>.png ; scale 1 px = 2 mm"""
import sys
import numpy as np, cv2
sys.path.insert(0, '/home/junjieye/rax-runtime-r1/sessions/20261106T010905Z_puppy_box_r1_f6b331/agent-workspace/code/puppy_box')
from percep import load_frame, cloud
WS = '/home/junjieye/rax-runtime-r1/sessions/20261106T010905Z_puppy_box_r1_f6b331/agent-workspace'


def render(fid, x0=0.1, x1=0.7, y0=-0.5, y1=0.1, zmin=-0.03, res=0.002):
    meta, rgb, dep = load_frame(fid)
    P, ok = cloud(meta, dep, stride=1)
    H, W = int((x1 - x0) / res), int((y1 - y0) / res)
    img = np.full((H, W, 3), 40, np.uint8)
    zb = np.full((H, W), -9.0)
    sel = ok & (P[..., 2] > zmin)
    pts, col = P[sel], rgb[sel]
    r = ((x1 - pts[:, 0]) / res).astype(int)   # x forward = up in the image
    c = ((y1 - pts[:, 1]) / res).astype(int)   # y left = left in the image
    m = (r >= 0) & (r < H) & (c >= 0) & (c < W)
    for rr, cc, zz, co in zip(r[m], c[m], pts[m, 2], col[m]):
        if zz > zb[rr, cc]:
            zb[rr, cc] = zz
            img[rr, cc] = co
    for gx in np.arange(np.ceil(x0 / 0.05) * 0.05, x1, 0.05):
        rr = min(int((x1 - gx) / res), H - 1)
        img[rr, ::4] = (0, 0, 255) if abs(gx * 10 - round(gx * 10)) < 1e-6 else (0, 128, 255)
    for gy in np.arange(np.ceil(y0 / 0.05) * 0.05, y1, 0.05):
        cc = min(int((y1 - gy) / res), W - 1)
        img[::4, cc] = (255, 0, 0) if abs(gy * 10 - round(gy * 10)) < 1e-6 else (255, 160, 0)
    out = f'{WS}/output/td_{fid}.png'
    cv2.imwrite(out, img)
    return out


if __name__ == '__main__':
    a = sys.argv[2:]
    print(render(sys.argv[1], *[float(v) for v in a]))
