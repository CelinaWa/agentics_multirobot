"""Clear-box rim from the colour image (depth sees through it): the pale, low-saturation region over the wood on
the box side. Far rim = top row of the region at rim height, near wall foot = bottom row at table height, inner wall
= the region's inner column at rim height. python code/puppy_box/boxdet.py obs.json left|right"""
import sys, os, json, numpy as np, cv2
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import percep as p

RIM_H = 0.14


def locate_box(P, side, tz=None, size_y=0.39):
    tz = p.table_z(P) if tz is None else tz
    hsv = cv2.cvtColor(P['img'], cv2.COLOR_BGR2HSV)
    _, lev = cloud = p.cloud(P)
    # pale pixels whose depth lands on the table plane (the box walls are invisible in depth)
    m = (hsv[..., 1] < 80) & (hsv[..., 2] > 150) & ((np.abs(lev[..., 2] - tz) < 0.03) | (np.abs(lev[..., 2] - tz - RIM_H) < 0.03)) & (P['depth'] > 0)
    h, w = m.shape
    if side == 'left':
        m[:, w // 2 + 40:] = False
    else:
        m[:, :w // 2 - 40] = False
    m = cv2.morphologyEx(m.astype(np.uint8), cv2.MORPH_OPEN, np.ones((7, 7), np.uint8))
    n, lab, st, _ = cv2.connectedComponentsWithStats(m)
    if n < 2:
        return None
    k = 1 + int(np.argmax(st[1:, cv2.CC_STAT_AREA]))
    x0, y0, bw, bh, area = st[k]
    if area < 3000:
        return None
    rows = np.where((lab == k).any(1))[0]
    top, bot = int(rows.min()), int(rows.max())
    mid = (top + bot) // 2
    cols = np.where(lab[mid] == k)[0]
    inner_u = int(cols.max()) if side == 'left' else int(cols.min())
    far = p.plane_hit(P, (cols.min() + cols.max()) / 2, top, tz + RIM_H)
    near = p.plane_hit(P, (cols.min() + cols.max()) / 2, bot, tz)
    inner = p.plane_hit(P, inner_u, mid, tz + RIM_H)
    s = 1 if side == 'left' else -1
    x_near, x_far = float(near[0]), float(far[0])
    y_in = float(inner[1]); y_out = y_in + s * size_y
    return dict(x=[x_near, x_far], y=sorted([y_in, y_out]), y_inner=y_in, rim_z=tz + RIM_H, table_z=tz,
                centre=[(x_near + x_far) / 2, (y_in + y_out) / 2], px=dict(top=top, bot=bot, inner_u=inner_u),
                touches_edge=bool(x0 <= 1 or x0 + bw >= w - 1))


if __name__ == '__main__':
    P = p.load(sys.argv[1]); b = locate_box(P, sys.argv[2])
    print(json.dumps(b))
    if b:
        p.draw_box(P, b['x'][0], b['x'][1], b['y'][0], b['y'][1], b['rim_z'], b['table_z'], f"output/boxdet_{P['ob']['step_id']}.jpg")
