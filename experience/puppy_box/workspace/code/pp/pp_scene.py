"""Scene measurement from a saved frame: table height, puppy blob, top-down height map."""
import os, sys, json
import numpy as np, cv2
sys.path.insert(0, os.path.join(os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__)))), 'code', 'pbox'))
from scene import cloud, FR  # noqa

def table_z(P):
    z = P[..., 2]; x = P[..., 0]
    m = (x < 0.75) & np.isfinite(z) & (z < 0.1) & (z > -0.3)
    h, e = np.histogram(z[m], bins=np.arange(-0.3, 0.1, 0.005))
    return float(e[np.argmax(h)] + 0.0025)

def heightmap(fid, out=None, x0=0.15, x1=0.85, y0=-0.6, y1=0.6, res=0.005):
    P = cloud(fid); t = table_z(P)
    nx, ny = int((x1 - x0) / res), int((y1 - y0) / res)
    H = np.full((nx, ny), np.nan)
    v = np.isfinite(P[..., 2])
    p = P[v]
    i = ((p[:, 0] - x0) / res).astype(int); j = ((y1 - p[:, 1]) / res).astype(int)
    k = (i >= 0) & (i < nx) & (j >= 0) & (j < ny)
    for a, b, z in zip(i[k], j[k], p[k, 2] - t):
        if not (H[a, b] >= z): H[a, b] = z
    if out:
        img = np.clip(np.nan_to_num(H, nan=-0.05) / 0.2, 0, 1)
        img = cv2.applyColorMap((img * 255).astype(np.uint8), cv2.COLORMAP_JET)
        img = cv2.resize(img[::-1], (ny * 3, nx * 3), interpolation=cv2.INTER_NEAREST)
        for xx in np.arange(0.2, x1, 0.1):
            r = int((x1 - xx) / res * 3); cv2.line(img, (0, r), (ny * 3, r), (255, 255, 255), 1)
            cv2.putText(img, f'x{xx:.1f}', (2, r - 2), 0, 0.4, (255, 255, 255), 1)
        for yy in np.arange(-0.5, 0.6, 0.1):
            c = int((y1 - yy) / res * 3); cv2.line(img, (c, 0), (c, nx * 3), (255, 255, 255), 1)
            cv2.putText(img, f'y{yy:+.1f}', (c + 2, 12), 0, 0.4, (255, 255, 255), 1)
        cv2.imwrite(out, img)
    return H, t

if __name__ == '__main__':
    fid = sys.argv[1]
    H, t = heightmap(fid, f'output/hm_{fid}.png')
    print('table z', round(t, 3))
    x0, y1, res = 0.15, 0.6, 0.005
    for xi in range(0, H.shape[0], 4):
        row = H[xi]
        s = ''.join(' ' if not np.isfinite(z) else ('.' if z < 0.015 else str(min(9, int(z / 0.02)))) for z in row[::3])
        print(f'{x0 + xi * res:.3f} {s}')
