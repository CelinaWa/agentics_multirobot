"""Calibrate R1's eye camera (videohub, 1280x720 wide-angle) with the printed A4 checkerboard: fisheye intrinsics.

Runs ON the R1 Jetson with MA-Teleop's .venv-r1 (cv2, unitree_sdk2py). Read-only: it only asks videohub for frames.

    cd ~/rax_bridge && ~/MA-Teleop/.venv-r1/bin/python r1_cam_calib.py capture --out ~/r1_calib   # move the board around
    cd ~/rax_bridge && ~/MA-Teleop/.venv-r1/bin/python r1_cam_calib.py solve --out ~/r1_calib     # -> ~/r1_calib/intrinsics.json

capture keeps a frame only when the whole board is found and it sits clearly elsewhere in the image than the frames kept
so far (or at another size/tilt); it stops at --frames (default 25). Cover the centre, all four corners and the edges, and
tilt the board up to ~45 deg: the fisheye edges are what the calibration needs most. Board: 7 x 10 squares of 25 mm
(6 x 9 inner corners), ~/r1_checkerboard_A4_25mm.pdf printed at 100 %."""
import argparse
import json
import os
import sys
import time
from pathlib import Path

import cv2
import numpy as np

PATTERN = (6, 9)


def frames(interface):
    os.environ.pop('CYCLONEDDS_URI', None)
    sys.path.insert(0, str(Path.home() / 'MA-Teleop' / 'tools' / 'r1'))
    sys.path.insert(0, str(Path.home() / 'MA-Teleop'))
    from dds_runtime import initialize                       # MA-Teleop tools/r1: the lab's DDS setup for videohub
    from unitree_sdk2py.go2.video.video_client import VideoClient
    initialize(interface)
    client = VideoClient(); client.SetTimeout(1.0); client.Init()
    while True:
        code, data = client.GetImageSample()
        if code != 0:
            print(f'videohub returned {code}', flush=True); time.sleep(0.5); continue
        img = cv2.imdecode(np.frombuffer(bytes(data), np.uint8), cv2.IMREAD_COLOR)
        if img is not None:
            yield img


def find(gray):
    ok, c = cv2.findChessboardCornersSB(gray, PATTERN, cv2.CALIB_CB_EXHAUSTIVE | cv2.CALIB_CB_ACCURACY)
    return c.reshape(-1, 2) if ok else None


def capture(a):
    out = Path(a.out).expanduser(); out.mkdir(parents=True, exist_ok=True)
    kept = []                                   # (centre xy, area, corner-quad) of kept frames
    n = len(list(out.glob('board_*.png')))
    print(f'capturing into {out} ({n} already there); Ctrl-C to stop', flush=True)
    last_print = 0
    for img in frames(a.interface):
        gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
        c = find(gray)
        now = time.time()
        if c is None:
            if now - last_print > 2: print('  board not found', flush=True); last_print = now
            continue
        centre = c.mean(0); area = cv2.contourArea(cv2.convexHull(c.astype(np.float32)))
        quad = c[[0, PATTERN[0] - 1, -PATTERN[0], -1]]
        new = all(np.linalg.norm(centre - k[0]) > 80 or abs(area / k[1] - 1) > 0.3 or np.abs(quad - k[2]).max() > 60
                  for k in kept)
        if not new:
            if now - last_print > 2: print('  board found, but too close to a kept view: move/tilt it', flush=True); last_print = now
            continue
        kept.append((centre, area, quad)); n += 1
        cv2.imwrite(str(out / f'board_{n:02d}.png'), img)
        print(f'KEPT {n}: centre ({centre[0]:.0f}, {centre[1]:.0f}) px, {100 * area / gray.size:.1f}% of the image', flush=True)
        if n >= a.frames:
            print('enough frames; run solve'); return
        time.sleep(0.7)


def solve(a):
    out = Path(a.out).expanduser()
    obj = np.zeros((1, PATTERN[0] * PATTERN[1], 3), np.float64)
    obj[0, :, :2] = np.mgrid[0:PATTERN[0], 0:PATTERN[1]].T.reshape(-1, 2) * a.square_m
    objs, imgs, names, size = [], [], [], None
    for f in sorted(out.glob('board_*.png')):
        gray = cv2.imread(str(f), cv2.IMREAD_GRAYSCALE); size = gray.shape[::-1]
        c = find(gray)
        if c is not None:
            objs.append(obj); imgs.append(c.reshape(1, -1, 2).astype(np.float64)); names.append(f.name)
    if len(objs) < 10:
        sys.exit(f'only {len(objs)} usable frames; capture more (need >= 10, best 20+)')
    flags = cv2.fisheye.CALIB_RECOMPUTE_EXTRINSIC | cv2.fisheye.CALIB_FIX_SKEW
    crit = (cv2.TERM_CRITERIA_COUNT + cv2.TERM_CRITERIA_EPS, 200, 1e-9)
    while True:                                 # drop the worst-conditioned frame until the solver accepts the set
        try:
            rms, K, D, rv, tv = cv2.fisheye.calibrate(objs, imgs, size, None, None, flags=flags, criteria=crit)
            break
        except cv2.error as e:
            if len(objs) <= 10: raise
            msg = str(e); i = int(msg.split('input array ')[1].split()[0]) if 'input array ' in msg else len(objs) - 1
            print(f'  dropping {names[i]} (ill-conditioned)'); del objs[i], imgs[i], names[i]
    per = []
    for o, im, r, t in zip(objs, imgs, rv, tv):
        p, _ = cv2.fisheye.projectPoints(o, r, t, K, D)
        per.append(float(np.sqrt(((p - im) ** 2).sum(-1).mean())))
    res = {'model': 'opencv_fisheye', 'width': size[0], 'height': size[1], 'K': K.tolist(), 'D': D.ravel().tolist(),
           'rms_px': float(rms), 'frames': len(objs), 'per_frame_rms_px': dict(zip(names, per)), 'square_m': a.square_m,
           'pattern_inner_corners': list(PATTERN), 'date': time.strftime('%Y-%m-%d %H:%M')}
    (out / 'intrinsics.json').write_text(json.dumps(res, indent=1))
    print(f'rms {rms:.3f} px over {len(objs)} frames; fx {K[0,0]:.1f} fy {K[1,1]:.1f} cx {K[0,2]:.1f} cy {K[1,2]:.1f}; D {np.round(D.ravel(), 4).tolist()}')
    print(f'worst frames: {sorted(zip(per, names))[-3:]}')
    print(f'wrote {out / "intrinsics.json"}  (good: rms < 0.5 px)')


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('cmd', choices=('capture', 'solve'))
    ap.add_argument('--out', default='~/r1_calib')
    ap.add_argument('--interface', default='eth10')
    ap.add_argument('--frames', type=int, default=25)
    ap.add_argument('--square-m', type=float, default=0.025, help='measured square size of the print, metres')
    a = ap.parse_args()
    capture(a) if a.cmd == 'capture' else solve(a)


if __name__ == '__main__':
    main()
