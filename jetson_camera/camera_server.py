"""Serve the robot's head camera to a session on another machine (`[real] camera = "network"`):
on the robot computer, the ZED (or a RealSense) is opened here and its frames go out over TCP.

    python tools/camera_server.py --camera zed --calibration /usr/local/zed/settings --port 7790
"""
import argparse
import sys
import time
from pathlib import Path

HERE = Path(__file__).resolve()
sys.path[:0] = [str(HERE.parents[1] / "src"), str(HERE.parent)]
try:
    from homebody.backends.real.camera import CameraServer, RealSenseCamera, ZedCamera, ZedSdkCamera
except ImportError:
    from camera import CameraServer, RealSenseCamera, ZedCamera, ZedSdkCamera


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--camera", choices=("zedsdk", "zed", "realsense"), default="zedsdk",
                        help="zedsdk: the ZED SDK's own depth (pyzed on the robot computer); zed: the raw pair, depth on the host")
    parser.add_argument("--calibration", default="/usr/local/zed/settings")
    parser.add_argument("--device", type=int, default=0)
    parser.add_argument("--fps", type=int, default=15)
    parser.add_argument("--port", type=int, default=7790)
    parser.add_argument("--quality", type=int, default=90)
    parser.add_argument("--depth-mode", default="NEURAL",
                        help="zedsdk: the SDK depth mode (NEURAL, NEURAL_PLUS, ULTRA, QUALITY, PERFORMANCE); a neural mode "
                             "whose model is not installed falls back to ULTRA")
    args = parser.parse_args(argv)
    if args.camera == "zedsdk":
        camera = ZedSdkCamera(fps=args.fps, depth_mode=args.depth_mode).start()
    elif args.camera == "zed":
        camera = ZedCamera(args.calibration, device=args.device, fps=args.fps).start()
    else:
        camera = RealSenseCamera(640, 480, args.fps, "any", stereo=True).start()
    server = CameraServer(camera, args.port, quality=args.quality)
    print(f"serving {args.camera} on port {server.port}", flush=True)
    try:
        while True:
            time.sleep(5.0)
            print(f"frames sent {server.sent}", flush=True)
    except KeyboardInterrupt:
        pass
    finally:
        server.close()
        camera.close()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
