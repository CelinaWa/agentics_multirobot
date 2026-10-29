"""Check the g1sonic backend's MA-Teleop camera client against the lab's Jetson stream.

Live, on the lab network:  camera_check.py --host 192.168.123.164 [--port 5620] [--seconds 5]
Offline, against MA-Teleop's own publisher code with a synthetic capture (no camera):
    camera_check.py --ma-teleop <MA-Teleop checkout>
Both read frames through `services.g1sonic.camera.MaTeleopCamera` (the same class the backend uses
with `camera: 'ma-teleop'`), report rate and shape, and save the last frame (--out).
"""
import argparse
import sys
import threading
import time
from pathlib import Path

import cv2
import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[2]))
from services.g1sonic.camera import MaTeleopCamera  # noqa: E402

ZED_HD720 = [[700.01, 0.0, 669.775], [0.0, 700.01, 322.859], [0.0, 0.0, 1.0]]  # MA-Teleop configs/robots/g1.yaml


class SyntheticCapture:
    """What g1_camera_publisher.run_camera expects of a camera backend: a moving test pattern."""

    def __init__(self, width, height, fps):
        self.width, self.height, self.fps = width, height, fps
        self.frames = 0
        self.started = None

    def start(self):
        self.started = time.time()
        return True

    def get_frame(self):
        self.frames += 1
        v, u = np.mgrid[0:self.height, 0:self.width]
        shift = (self.frames * 7) % self.width
        bgr = np.stack([(u + shift) % 256, v % 256, np.full_like(u, 128)], axis=-1).astype(np.uint8)
        cv2.putText(bgr, f"frame {self.frames}", (20, 40), cv2.FONT_HERSHEY_SIMPLEX, 1.0, (255, 255, 255), 2)
        time.sleep(1.0 / self.fps)
        return bgr

    def fps_estimate(self, reset=True):
        return self.frames


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--host")
    ap.add_argument("--port", type=int, default=5620)
    ap.add_argument("--ma-teleop", type=Path, help="run MA-Teleop's g1_camera_publisher.run_camera with a synthetic capture")
    ap.add_argument("--seconds", type=float, default=5.0)
    ap.add_argument("--out", type=Path, default=Path("/tmp/camera_check_last.jpg"))
    args = ap.parse_args()
    if not args.host and not args.ma_teleop:
        ap.error("--host (live) or --ma-teleop (offline) is required")
    host, port = args.host or "127.0.0.1", args.port
    if args.ma_teleop:
        sys.path.insert(0, str(args.ma_teleop / "teleop/clients/g1"))
        import zmq
        import g1_camera_publisher as publisher
        spec = {"width": 1280, "height": 720, "fps": 15, "port": port, "jpeg": True, "jpeg_quality": 80}
        capture = SyntheticCapture(spec["width"], spec["height"], spec["fps"])
        threading.Thread(target=publisher.run_camera, args=("ego_zed", spec, capture, zmq.Context.instance()),
                         kwargs={"warmup": False}, daemon=True).start()
        print(f"MA-Teleop run_camera publishing a synthetic {spec['width']}x{spec['height']} JPEG stream on {port}")
    camera = MaTeleopCamera(host, port, ZED_HD720).start()
    first = camera.read()
    if first is None:
        sys.exit(f"no frame from tcp://{host}:{port} within {camera.timeout:g} s ({camera.error})")
    started, count0 = time.time(), camera.received
    time.sleep(args.seconds)
    stamp, rgb, depth, K = camera.read()
    rate = (camera.received - count0) / (time.time() - started)
    print(f"frames: {camera.received} ({rate:.1f} Hz), shape {rgb.shape}, depth all-NaN={bool(np.isnan(depth).all())}, "
          f"fx={K[0, 0]:.1f}, age {time.time() - stamp:.3f} s, decode errors: {camera.error}")
    cv2.imwrite(str(args.out), cv2.cvtColor(rgb, cv2.COLOR_RGB2BGR))
    print(f"saved {args.out}")
    camera.close()


if __name__ == "__main__":
    main()
