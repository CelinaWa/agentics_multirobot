"""The head camera on the real G1: an Intel RealSense D435i giving colour and depth aligned to
it, with the option the authors used of estimating depth from its stereo infrared pair with
Fast-FoundationStereo instead of the camera's own matcher. `FakeCamera` replays frames."""
from __future__ import annotations

import time
from dataclasses import dataclass
from pathlib import Path

import numpy as np

class NotReleased(RuntimeError):
    pass


@dataclass
class CameraFrame:
    rgb: np.ndarray
    depth: np.ndarray  # metres along the optical axis, NaN where unknown
    intrinsics: np.ndarray  # 3x3 of the colour image
    stamp: float
    infrared: tuple | None = None  # (left, right) images and their baseline in metres


ZED_RESOLUTIONS = {"HD": (1280, 720, "HD"), "VGA": (672, 376, "VGA"), "FHD": (1920, 1080, "FHD")}


class Unavailable(RuntimeError):
    """A component the real backend can use is not installed or not reachable here."""


class RealSenseCamera:
    """A D435i streaming colour and aligned depth, and the infrared pair when STEREO is set. With
    BAG, a RealSense recording plays back through the same pipeline instead of a device."""

    def __init__(self, width=640, height=480, fps=30, serial=None, stereo=False, clock=time.monotonic, bag=None):
        self.width, self.height, self.fps, self.serial, self.stereo, self.clock = width, height, fps, serial, stereo, clock
        self.bag = bag
        self.pipeline = None

    def start(self):
        try:
            import pyrealsense2 as rs
        except ImportError as error:
            raise Unavailable("pyrealsense2 is not installed") from error
        self.rs = rs
        config = rs.config()
        if self.bag:
            config.enable_device_from_file(str(self.bag), repeat_playback=False)
            config.enable_all_streams()
        else:
            if self.serial and self.serial != "any":
                config.enable_device(self.serial)
            config.enable_stream(rs.stream.color, self.width, self.height, rs.format.rgb8, self.fps)
            config.enable_stream(rs.stream.depth, self.width, self.height, rs.format.z16, self.fps)
            if self.stereo:
                config.enable_stream(rs.stream.infrared, 1, self.width, self.height, rs.format.y8, self.fps)
                config.enable_stream(rs.stream.infrared, 2, self.width, self.height, rs.format.y8, self.fps)
        self.pipeline = rs.pipeline()
        profile = self.pipeline.start(config)
        if self.bag:
            profile.get_device().as_playback().set_real_time(False)
            streams = {stream.stream_type() for stream in profile.get_streams()}
            self.stereo = self.stereo and rs.stream.infrared in streams
        self.scale = profile.get_device().first_depth_sensor().get_depth_scale()
        colour = profile.get_stream(rs.stream.color).as_video_stream_profile().get_intrinsics()
        self.intrinsics = np.array([[colour.fx, 0, colour.ppx], [0, colour.fy, colour.ppy], [0, 0, 1]])
        self.align = rs.align(rs.stream.color)
        self.baseline = None
        if self.stereo:
            try:
                left = profile.get_stream(rs.stream.infrared, 1).as_video_stream_profile()
                right = profile.get_stream(rs.stream.infrared, 2).as_video_stream_profile()
                self.baseline = abs(left.get_extrinsics_to(right).translation[0])
            except RuntimeError:  # one infrared stream only
                self.stereo = False
        return self

    def read(self, timeout_ms=1000):
        frames = self.align.process(self.pipeline.wait_for_frames(timeout_ms))
        colour, depth = frames.get_color_frame(), frames.get_depth_frame()
        if not colour or not depth:
            raise RuntimeError("the camera delivered no colour and depth pair")
        rgb = np.asanyarray(colour.get_data()).copy()
        metres = np.asanyarray(depth.get_data()).astype(np.float32) * self.scale
        metres[metres <= 0] = np.nan
        infrared = None
        if self.stereo:
            infrared = (np.asanyarray(frames.get_infrared_frame(1).get_data()).copy(),
                        np.asanyarray(frames.get_infrared_frame(2).get_data()).copy(), self.baseline)
        return CameraFrame(rgb, metres, self.intrinsics.copy(), self.clock(), infrared)

    def close(self):
        if self.pipeline is not None:
            self.pipeline.stop()
            self.pipeline = None


class FastFoundationStereo:
    """Depth from a rectified stereo pair with NVIDIA's Fast-FoundationStereo, the model the
    authors ran on the D435i's infrared pair: its released ONNX graph (nvidia/c-fast-foundationstereo,
    inputs `left_image` and `right_image` as RGB 0-255 at 480x736, output disparity in pixels)
    through onnxruntime. Disparity becomes metres through the pair's focal length and baseline."""

    WIDTH, HEIGHT = 736, 480

    def __init__(self, weights, device="cuda"):
        self.weights, self.device, self.session = Path(weights), device, None

    def load(self):
        try:
            import onnxruntime as ort
        except ImportError as error:
            raise Unavailable("onnxruntime is not installed") from error
        path = self.weights if self.weights.is_file() else next(iter(sorted(self.weights.rglob("*.onnx"))), None)
        if path is None:
            raise Unavailable(f"no Fast-FoundationStereo ONNX model under {self.weights}")
        providers = ["CUDAExecutionProvider", "CPUExecutionProvider"] if self.device == "cuda" else ["CPUExecutionProvider"]
        self.session = ort.InferenceSession(str(path), providers=providers)
        self.inputs = [item.name for item in self.session.get_inputs()]
        return self

    def disparity(self, left, right):
        """Disparity in pixels of LEFT against RIGHT (H x W, grey or RGB), at the images' size."""
        if self.session is None:
            self.load()
        import cv2
        height, width = left.shape[:2]
        feeds = {}
        for name, image in zip(self.inputs, (left, right)):
            image = np.asarray(image)
            if image.ndim == 2:
                image = np.repeat(image[:, :, None], 3, axis=2)
            image = cv2.resize(image, (self.WIDTH, self.HEIGHT), interpolation=cv2.INTER_AREA)
            feeds[name] = np.ascontiguousarray(image.transpose(2, 0, 1)[None], dtype=np.float32)
        disparity = self.session.run(None, feeds)[0][0, 0]
        disparity = cv2.resize(disparity, (width, height), interpolation=cv2.INTER_LINEAR) * (width / self.WIDTH)
        return disparity

    def depth(self, frame: CameraFrame):
        if frame.infrared is None:
            raise Unavailable("the camera streams no infrared pair")
        left, right, baseline = frame.infrared
        disparity = self.disparity(left, right)
        focal = float(frame.intrinsics[0, 0]) * (left.shape[1] / frame.rgb.shape[1])
        with np.errstate(divide="ignore", invalid="ignore"):
            metres = focal * baseline / np.asarray(disparity, dtype=np.float32)
        metres[~np.isfinite(metres) | (metres <= 0) | (disparity < 0.5)] = np.nan
        return metres


class ZedCamera:
    """A Stereolabs ZED as a plain UVC stereo camera: its side-by-side frame is split, each half
    is undistorted and rectified from the factory calibration the ZED SDK installs
    (`/usr/local/zed/settings/SN*.conf`), and the rectified pair is handed to Fast-FoundationStereo
    for depth, as the authors did with the D435i's infrared pair. No ZED SDK is needed."""

    def __init__(self, calibration, device=0, resolution="HD", fps=30, clock=time.monotonic):
        self.calibration, self.device, self.fps, self.clock = Path(calibration), device, fps, clock
        self.width, self.height, self.key = ZED_RESOLUTIONS[resolution]
        self.capture = None

    @staticmethod
    def parse(path):
        """The calibration file's sections as {section: {key: float}}."""
        sections, current = {}, None
        for line in Path(path).read_text().splitlines():
            line = line.strip()
            if line.startswith("[") and line.endswith("]"):
                current = sections.setdefault(line[1:-1], {})
            elif "=" in line and current is not None:
                key, _, value = line.partition("=")
                try:
                    current[key.strip()] = float(value)
                except ValueError:
                    pass
        return sections

    def start(self):
        import cv2
        path = self.calibration
        if path.is_dir():
            path = next(iter(sorted(path.glob("SN*.conf"))), None)
            if path is None:
                raise Unavailable(f"no ZED calibration under {self.calibration}")
        cal = self.parse(path)
        left, right, stereo = cal[f"LEFT_CAM_{self.key}"], cal[f"RIGHT_CAM_{self.key}"], cal["STEREO"]
        K1 = np.array([[left["fx"], 0, left["cx"]], [0, left["fy"], left["cy"]], [0, 0, 1]])
        K2 = np.array([[right["fx"], 0, right["cx"]], [0, right["fy"], right["cy"]], [0, 0, 1]])
        D1 = np.array([left.get(k, 0.0) for k in ("k1", "k2", "p1", "p2", "k3")])
        D2 = np.array([right.get(k, 0.0) for k in ("k1", "k2", "p1", "p2", "k3")])
        rotation, _ = cv2.Rodrigues(np.array([stereo.get(f"RX_{self.key}", 0.0), stereo.get(f"CV_{self.key}", 0.0),
                                              stereo.get(f"RZ_{self.key}", 0.0)]))
        self.baseline = stereo["Baseline"] / 1000.0
        translation = np.array([-self.baseline, stereo.get("TY", 0.0) / 1000.0, stereo.get("TZ", 0.0) / 1000.0])
        size = (self.width, self.height)
        R1, R2, P1, P2, _, _, _ = cv2.stereoRectify(K1, D1, K2, D2, size, rotation, translation, alpha=0)
        self.maps = (cv2.initUndistortRectifyMap(K1, D1, R1, P1, size, cv2.CV_32FC1),
                     cv2.initUndistortRectifyMap(K2, D2, R2, P2, size, cv2.CV_32FC1))
        self.intrinsics = P1[:3, :3].copy()
        self.capture = cv2.VideoCapture(self.device, cv2.CAP_V4L2)
        self.capture.set(cv2.CAP_PROP_FRAME_WIDTH, 2 * self.width)
        self.capture.set(cv2.CAP_PROP_FRAME_HEIGHT, self.height)
        self.capture.set(cv2.CAP_PROP_FPS, self.fps)
        if not self.capture.isOpened():
            raise Unavailable(f"ZED video device {self.device} did not open")
        return self

    def rectified(self, frame):
        """(left, right) rectified RGB halves of a side-by-side FRAME (BGR, as OpenCV reads it)."""
        import cv2
        half = frame.shape[1] // 2
        left, right = frame[:, :half, ::-1], frame[:, half:, ::-1]
        (m1x, m1y), (m2x, m2y) = self.maps
        return (cv2.remap(np.ascontiguousarray(left), m1x, m1y, cv2.INTER_LINEAR),
                cv2.remap(np.ascontiguousarray(right), m2x, m2y, cv2.INTER_LINEAR))

    def read(self, timeout_ms=None):
        ok, frame = self.capture.read()
        if not ok:
            raise RuntimeError("the ZED delivered no frame")
        left, right = self.rectified(frame)
        depth = np.full(left.shape[:2], np.nan, dtype=np.float32)  # Fast-FoundationStereo fills it
        return CameraFrame(left, depth, self.intrinsics.copy(), self.clock(), (left, right, self.baseline))

    def close(self):
        if self.capture is not None:
            self.capture.release()
            self.capture = None


def frame_from_sdk(bgra, depth_m, intrinsics, stamp):
    """A CameraFrame from the ZED SDK's left image (BGRA) and depth measure (metres, non-finite
    where unknown)."""
    rgb = np.ascontiguousarray(np.asarray(bgra)[:, :, :3][:, :, ::-1])
    depth = np.array(depth_m, dtype=float)
    depth[~np.isfinite(depth) | (depth <= 0)] = np.nan
    return CameraFrame(rgb, depth, np.asarray(intrinsics, dtype=float), stamp)


ZED_RESOURCES = Path("/usr/local/zed/resources")
NEURAL_DEPTH_MODELS = {"NEURAL": "neural_depth", "NEURAL_PLUS": "neural_plus_depth", "NEURAL_LIGHT": "neural_light_depth"}


def sdk_depth_mode(requested, resources=ZED_RESOURCES):
    """The ZED SDK depth mode to open and why it differs from REQUESTED, if it does: a neural
    mode whose model is not on this machine (no file, or the empty file of an aborted download)
    would make the SDK wait for a download, forever on an offline robot, so ULTRA, the best
    mode that needs no model, opens instead."""
    prefix = NEURAL_DEPTH_MODELS.get(str(requested).upper())
    if prefix is None:
        return requested, None
    models = [p for p in Path(resources).glob(f"{prefix}*.model") if p.is_file() and p.stat().st_size > 0]
    if models:
        return requested, None
    return "ULTRA", f"the {requested} depth model is not in {resources}; the SDK would wait for a download"


class ZedSdkCamera:
    """The ZED through Stereolabs' SDK on the robot computer: the left image and the SDK's own
    depth, so a session on another machine needs no stereo network. Serve it with
    tools/camera_server.py --camera zedsdk. NEURAL depth when its model is installed, else ULTRA."""

    def __init__(self, resolution="HD720", fps=15, depth_mode="NEURAL", clock=time.monotonic, resources=ZED_RESOURCES):
        self.resolution, self.fps, self.depth_mode, self.clock = resolution, fps, depth_mode, clock
        self.resources = resources
        self.camera = self.intrinsics = None

    def start(self):
        try:
            from pyzed import sl
        except ImportError as error:
            raise Unavailable("pyzed (the ZED SDK's Python API) is not installed here") from error
        self._sl = sl
        init = sl.InitParameters()
        init.camera_resolution = getattr(sl.RESOLUTION, self.resolution)
        init.camera_fps = int(self.fps)
        self.depth_mode, reason = sdk_depth_mode(self.depth_mode, self.resources)
        if reason:
            print(f"ZED depth mode {self.depth_mode}: {reason}", flush=True)
        init.depth_mode = getattr(sl.DEPTH_MODE, self.depth_mode)
        init.coordinate_units = sl.UNIT.METER
        self.camera = sl.Camera()
        status = self.camera.open(init)
        if status != sl.ERROR_CODE.SUCCESS:
            raise Unavailable(f"The ZED did not open: {status}")
        left = self.camera.get_camera_information().camera_configuration.calibration_parameters.left_cam
        self.intrinsics = np.array([[left.fx, 0.0, left.cx], [0.0, left.fy, left.cy], [0.0, 0.0, 1.0]])
        self._runtime, self._image, self._depth = sl.RuntimeParameters(), sl.Mat(), sl.Mat()
        return self

    def read(self, timeout_ms=1000):
        sl = self._sl
        deadline = self.clock() + timeout_ms / 1000.0
        while self.camera.grab(self._runtime) != sl.ERROR_CODE.SUCCESS:
            if self.clock() > deadline:
                raise Unavailable("The ZED produced no frame in time")
        self.camera.retrieve_image(self._image, sl.VIEW.LEFT)
        self.camera.retrieve_measure(self._depth, sl.MEASURE.DEPTH)
        return frame_from_sdk(self._image.get_data(), self._depth.get_data(), self.intrinsics, self.clock())

    def close(self):
        if self.camera is not None:
            self.camera.close()


class FakeCamera:
    """Replays CameraFrames in order, the last one forever."""

    def __init__(self, frames, clock=time.monotonic):
        self.frames, self.clock, self.index = list(frames), clock, 0
        if not self.frames:
            raise ValueError("FakeCamera needs at least one frame")

    def start(self):
        return self

    def read(self, timeout_ms=None):
        frame = self.frames[min(self.index, len(self.frames) - 1)]
        self.index += 1
        return CameraFrame(frame.rgb, frame.depth, frame.intrinsics, self.clock(), frame.infrared)

    def close(self):
        pass


WIRE_MAGIC = b"HBCF"


_CODEC = None


def jpeg_codec():
    """"cv2" when OpenCV can encode an image here, else "pil": a robot environment can pair an
    OpenCV wheel with a NumPy it was not built for, and then every imencode fails."""
    global _CODEC
    if _CODEC is None:
        try:
            import cv2
        except ImportError:
            _CODEC = "pil"
            return _CODEC
        try:
            ok, _ = cv2.imencode(".jpg", np.zeros((2, 2, 3), np.uint8))
        except cv2.error:
            ok = False
        _CODEC = "cv2" if ok else "pil"
    return _CODEC


def jpeg_encode(image, quality=90):
    """IMAGE, uint8 RGB or grey, as JPEG bytes."""
    image = np.ascontiguousarray(image)
    if jpeg_codec() == "cv2":
        import cv2
        ok, data = cv2.imencode(".jpg", image if image.ndim == 2 else np.ascontiguousarray(image[:, :, ::-1]),
                                [cv2.IMWRITE_JPEG_QUALITY, int(quality)])
        if not ok:
            raise RuntimeError("JPEG encoding failed")
        return data.tobytes()
    import io

    from PIL import Image
    buffer = io.BytesIO()
    Image.fromarray(image).save(buffer, format="JPEG", quality=int(quality))
    return buffer.getvalue()


def jpeg_decode(data, gray=False):
    """JPEG bytes as a uint8 RGB array, or a grey one when GRAY."""
    if jpeg_codec() == "cv2":
        import cv2
        image = cv2.imdecode(np.frombuffer(data, dtype=np.uint8), cv2.IMREAD_GRAYSCALE if gray else cv2.IMREAD_COLOR)
        return image if gray else image[:, :, ::-1].copy()
    import io

    from PIL import Image
    with Image.open(io.BytesIO(data)) as image:
        return np.array(image.convert("L" if gray else "RGB"))


def encode_frame(frame: CameraFrame, quality=90):
    """A camera frame as one wire message: a JSON header, then JPEG colour, JPEG right image
    (stereo pairs) and 16-bit millimetre depth, zlib-compressed, where the frame has them."""
    import json
    import struct
    import zlib
    blobs, header = [], {"stamp": frame.stamp, "intrinsics": np.asarray(frame.intrinsics, dtype=float).reshape(-1).tolist(),
                         "shape": list(frame.rgb.shape[:2]), "parts": []}
    blobs.append(jpeg_encode(frame.rgb, quality))
    header["parts"].append(["rgb", len(blobs[-1])])
    depth = np.asarray(frame.depth, dtype=float)
    if np.isfinite(depth).any():
        millimetres = np.where(np.isfinite(depth) & (depth > 0), np.clip(depth * 1000.0, 0, 65535), 0).astype(np.uint16)
        blobs.append(zlib.compress(millimetres.tobytes(), 3))
        header["parts"].append(["depth", len(blobs[-1])])
    if frame.infrared is not None:
        left, right, baseline = frame.infrared
        header["baseline"] = float(baseline)
        header["left_is_rgb"] = left is frame.rgb or (left.shape == frame.rgb.shape and np.array_equal(left, frame.rgb))
        if not header["left_is_rgb"]:
            blobs.append(jpeg_encode(left, quality))
            header["parts"].append(["left", len(blobs[-1])])
        blobs.append(jpeg_encode(right, quality))
        header["parts"].append(["right", len(blobs[-1])])
        header["right_gray"] = right.ndim == 2
    encoded = json.dumps(header).encode()
    return WIRE_MAGIC + struct.pack("<I", len(encoded)) + encoded + b"".join(blobs)


def decode_frame(header, blobs):
    """The CameraFrame of a wire message's HEADER and its named BLOBS."""
    import zlib
    rgb = jpeg_decode(blobs["rgb"])
    height, width = header["shape"]
    depth = np.full((height, width), np.nan)
    if "depth" in blobs:
        millimetres = np.frombuffer(zlib.decompress(blobs["depth"]), dtype=np.uint16).reshape(height, width)
        depth = np.where(millimetres > 0, millimetres / 1000.0, np.nan)
    infrared = None
    if "right" in blobs:
        right = jpeg_decode(blobs["right"], gray=bool(header.get("right_gray")))
        left = rgb if header.get("left_is_rgb", True) else jpeg_decode(blobs["left"], gray=True)
        infrared = (left, right, float(header["baseline"]))
    return CameraFrame(rgb, depth, np.array(header["intrinsics"], dtype=float).reshape(3, 3), float(header["stamp"]), infrared)


def read_message(stream):
    """The next (header, blobs) from a wire stream; None at its end."""
    import json
    import struct

    def exact(count):
        chunks, remaining = [], count
        while remaining > 0:
            chunk = stream.recv(remaining) if hasattr(stream, "recv") else stream.read(remaining)
            if not chunk:
                return None
            chunks.append(chunk)
            remaining -= len(chunk)
        return b"".join(chunks)
    magic = exact(4)
    if magic is None:
        return None
    if magic != WIRE_MAGIC:
        raise ValueError(f"camera stream out of step at {magic!r}")
    header = json.loads(exact(struct.unpack("<I", exact(4))[0]))
    blobs = {}
    for name, size in header["parts"]:
        blobs[name] = exact(size)
        if blobs[name] is None:
            return None
    return header, blobs


class CameraServer:
    """Serves a camera's frames to one `NetworkCamera` at a time over TCP, as the robot computer
    does for a session running on another machine on the robot's network."""

    def __init__(self, camera, port, host="0.0.0.0", quality=90, fps=15.0, clock=time.monotonic):
        import socket
        import threading
        self.camera, self.quality, self.clock, self.fps = camera, quality, clock, float(fps)
        self.socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.socket.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self.socket.bind((host, port))
        self.socket.listen(1)
        self.port = self.socket.getsockname()[1]
        self.sent, self._running = 0, True
        self._thread = threading.Thread(target=self._serve, name="camera-server", daemon=True)
        self._thread.start()

    def _serve(self):
        import socket
        while self._running:
            try:
                self.socket.settimeout(0.5)
                client, _ = self.socket.accept()
            except TimeoutError:
                continue
            except OSError:
                return
            with client:
                client.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)
                try:
                    while self._running:
                        started = time.monotonic()
                        client.sendall(encode_frame(self.camera.read(), self.quality))
                        self.sent += 1
                        # No faster than FPS: a camera that answers at once (a replay) must not
                        # flood the receiver, whose decoding would starve the control loop.
                        time.sleep(max(0.0, 1.0 / self.fps - (time.monotonic() - started)))
                except OSError:
                    continue

    def close(self):
        self._running = False
        try:
            self.socket.close()
        finally:
            self._thread.join(timeout=2.0)


class NetworkCamera:
    """The camera seam over a `CameraServer`: `read()` is the newest frame received."""

    def __init__(self, host, port, timeout=5.0, clock=time.monotonic):
        self.host, self.port, self.timeout, self.clock = host, int(port), timeout, clock
        self.frame, self.received, self._running, self._thread, self._socket = None, 0, False, None, None

    def start(self):
        import socket
        import threading
        self._socket = socket.create_connection((self.host, self.port), timeout=self.timeout)
        # Blocking from here on: a receive timeout would end the stream whenever this process is
        # busy (loading a model) for longer than it, and a partial message cannot be resumed.
        self._socket.settimeout(None)
        self._running = True
        self._event = threading.Event()
        self._thread = threading.Thread(target=self._receive, name="network-camera", daemon=True)
        self._thread.start()
        return self

    def _receive(self):
        while self._running:
            try:
                message = read_message(self._socket)
            except OSError:
                break
            if message is None:
                break
            header, blobs = message
            self.frame, self.received = decode_frame(header, blobs), self.received + 1
            self._event.set()
        self._running = False

    def read(self, timeout_ms=None):
        if self.frame is None and not self._event.wait((timeout_ms or self.timeout * 1000) / 1000):
            raise Unavailable(f"No frames from the camera server at {self.host}:{self.port}")
        if self.frame is None:
            raise Unavailable(f"The camera server at {self.host}:{self.port} closed")
        return self.frame

    def close(self):
        self._running = False
        if self._socket is not None:
            try:
                self._socket.shutdown(2)
            except OSError:
                pass
            self._socket.close()
        if self._thread is not None:
            self._thread.join(timeout=2.0)


__all__ = ["CameraFrame", "CameraServer", "FakeCamera", "FastFoundationStereo", "NetworkCamera", "NotReleased",
           "RealSenseCamera", "Unavailable", "ZedCamera", "ZedSdkCamera", "decode_frame", "encode_frame", "frame_from_sdk",
           "read_message"]
