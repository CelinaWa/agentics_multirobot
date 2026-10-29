"""The head camera: HomeBody's camera wire over TCP (`NetworkCamera`: a JSON header, a JPEG colour
image and zlib-compressed 16-bit millimetre depth), MA-Teleop's Jetson publisher over ZMQ
(`MaTeleopCamera`: the lab's ZED head view, colour only), and a fake for tests. `read()` returns
`(stamp, rgb, depth_m, intrinsics)`; depth is NaN where invalid or absent."""

from __future__ import annotations

import io
import json
import pickle
import struct
import threading
import time
import zlib

import numpy as np

WIRE_MAGIC = b"HBCF"


def jpeg_decode(data):
    import cv2
    image = cv2.imdecode(np.frombuffer(data, dtype=np.uint8), cv2.IMREAD_COLOR)
    if image is None:
        raise ValueError("undecodable JPEG from the camera server")
    return image[:, :, ::-1].copy()


def encode_frame(stamp, rgb, depth_m, intrinsics, quality=90):
    """One wire message: `HBCF`, a JSON header (stamp, shape, intrinsics, parts), JPEG colour and,
    when `depth_m` is given, zlib-compressed uint16 millimetres (0 where invalid)."""
    import cv2
    height, width = rgb.shape[:2]
    ok, jpeg = cv2.imencode(".jpg", np.ascontiguousarray(rgb[:, :, ::-1]), [cv2.IMWRITE_JPEG_QUALITY, quality])
    if not ok:
        raise ValueError("JPEG encoding failed")
    parts, blobs = [["rgb", len(jpeg)]], [jpeg.tobytes()]
    if depth_m is not None:
        depth_m = np.asarray(depth_m, dtype=float)
        valid = np.isfinite(depth_m) & (depth_m > 0.0) & (depth_m < 65.0)
        packed = zlib.compress(np.where(valid, np.round(depth_m * 1000.0), 0).astype(np.uint16).tobytes(), 3)
        parts.append(["depth", len(packed)])
        blobs.append(packed)
    header = json.dumps({"stamp": float(stamp), "shape": [height, width],
                         "intrinsics": np.asarray(intrinsics, dtype=float).tolist(), "parts": parts}).encode()
    return b"".join([WIRE_MAGIC, struct.pack("<I", len(header)), header, *blobs])


def read_message(stream):
    """The next (header, blobs) of a camera wire stream; None at its end."""
    def exact(count):
        chunks, remaining = [], count
        while remaining > 0:
            chunk = stream.recv(remaining)
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


def decode_frame(header, blobs):
    rgb = jpeg_decode(blobs["rgb"])
    height, width = header["shape"]
    depth = np.full((height, width), np.nan)
    if "depth" in blobs:
        millimetres = np.frombuffer(zlib.decompress(blobs["depth"]), dtype=np.uint16).reshape(height, width)
        depth = np.where(millimetres > 0, millimetres / 1000.0, np.nan)
    intrinsics = np.array(header["intrinsics"], dtype=float).reshape(3, 3)
    return float(header["stamp"]), rgb, depth, intrinsics


class NetworkCamera:
    """Frames from a `CameraServer`; `read()` is the newest frame received, stamped on arrival. The
    client connects in its own thread and reconnects after a dropped stream (the camera computer or
    its server restarting), so a session survives a restart of the robot's sensors."""

    RECONNECT_S = 2.0

    def __init__(self, host, port, timeout=5.0, clock=time.time):
        self.host, self.port, self.timeout, self.clock = host, int(port), timeout, clock
        self.frame, self.received, self.error = None, 0, None
        self.connections = 0
        self._running, self._thread, self._socket = False, None, None
        self._event = threading.Event()

    def start(self):
        self._running = True
        self._thread = threading.Thread(target=self._run, name="g1sonic-camera", daemon=True)
        self._thread.start()
        return self

    def _connect(self):
        import socket
        sock = socket.create_connection((self.host, self.port), timeout=self.timeout)
        sock.settimeout(self.timeout)
        return sock

    def _run(self):
        while self._running:
            try:
                self._socket = self._connect()
                self.connections += 1
                self.error = None
                self._receive()
            except OSError as error:
                self.error = str(error)[:200]
            finally:
                sock, self._socket = self._socket, None
                if sock is not None:
                    try:
                        sock.close()
                    except OSError:
                        pass
            if self._running:
                self._event.wait(self.RECONNECT_S) if self.frame is None else time.sleep(self.RECONNECT_S)

    def _receive(self):
        while self._running:
            try:
                message = read_message(self._socket)
            except ValueError as error:
                self.error = str(error)
                return
            if message is None:
                self.error = "the camera server closed the stream"
                return
            _stamp, rgb, depth, intrinsics = decode_frame(*message)
            self.frame = (self.clock(), rgb, depth, intrinsics)
            self.received += 1
            self._event.set()

    def read(self):
        if self.frame is None:
            self._event.wait(self.timeout)
        return self.frame

    def close(self):
        self._running = False
        sock = self._socket
        if sock is not None:
            try:
                sock.shutdown(2)
            except OSError:
                pass
        if self._thread is not None:
            self._thread.join(timeout=self.timeout + 1.0)
            self._thread = None


class RestrictedUnpickler(pickle.Unpickler):
    """Unpickles MA-Teleop's frame dicts and nothing else: builtins, bytes and numpy arrays."""
    ALLOWED = {('numpy', 'ndarray'), ('numpy', 'dtype'), ('numpy.core.multiarray', '_reconstruct'),
               ('numpy._core.multiarray', '_reconstruct'), ('numpy.core.multiarray', 'scalar'),
               ('numpy._core.multiarray', 'scalar')}

    def find_class(self, module, name):
        if (module, name) in self.ALLOWED:
            return super().find_class(module, name)
        raise pickle.UnpicklingError(f"refused {module}.{name} in a camera frame")


class MaTeleopCamera:
    """Frames from MA-Teleop's Jetson camera publisher (`teleop/clients/g1/g1_camera_publisher.py`:
    a ZMQ PUB of pickled `{"timestamp", "jpeg" | "image" (BGR)}` dicts on port 5620). The stream
    has no depth and no intrinsics, so the ZED's calibration comes from the configuration. The
    pickles are unpacked with `RestrictedUnpickler`: the robot network is trusted, not authenticated."""

    def __init__(self, host, port, intrinsics, timeout=5.0, clock=time.time):
        if intrinsics is None:
            raise ValueError("the MA-Teleop stream carries no intrinsics: set camera_intrinsics (3x3, the head "
                             "camera's calibration at the streamed size)")
        self.host, self.port, self.timeout, self.clock = host, int(port), timeout, clock
        self.intrinsics = np.asarray(intrinsics, dtype=float).reshape(3, 3)
        self.frame, self.received, self.error = None, 0, None
        self._running, self._thread, self._socket = False, None, None
        self._event = threading.Event()

    def start(self):
        import zmq
        self._socket = zmq.Context.instance().socket(zmq.SUB)
        self._socket.setsockopt(zmq.SUBSCRIBE, b"")
        self._socket.setsockopt(zmq.CONFLATE, 1)
        self._socket.setsockopt(zmq.RCVTIMEO, 1000)
        self._socket.connect(f"tcp://{self.host}:{self.port}")
        self._running = True
        self._thread = threading.Thread(target=self._receive, name="g1sonic-camera-zmq", daemon=True)
        self._thread.start()
        return self

    @staticmethod
    def decode(raw):
        """`(rgb, stamp)` of one published message; ValueError when it is not a frame."""
        message = RestrictedUnpickler(io.BytesIO(raw)).load()
        if not isinstance(message, dict):
            raise ValueError("camera message is not a dict")
        if message.get("jpeg") is not None:
            rgb = jpeg_decode(bytes(message["jpeg"]))
        elif message.get("image") is not None:
            bgr = np.asarray(message["image"])
            if bgr.ndim != 3 or bgr.shape[2] != 3:
                raise ValueError(f"camera image has shape {bgr.shape}")
            rgb = bgr[:, :, ::-1].copy()
        else:
            raise ValueError("camera message has neither jpeg nor image")
        return rgb, float(message.get("timestamp", 0.0))

    def _receive(self):
        import zmq
        while self._running:
            try:
                raw = self._socket.recv()
            except zmq.Again:
                continue
            except zmq.ZMQError as error:
                self.error = str(error)
                break
            try:
                rgb, _stamp = self.decode(raw)
            except (ValueError, pickle.UnpicklingError, EOFError) as error:
                self.error = str(error)[:200]
                continue
            depth = np.full(rgb.shape[:2], np.nan)
            self.frame = (self.clock(), rgb, depth, self.intrinsics.copy())
            self.received += 1
            self._event.set()
        self._running = False

    def read(self):
        if self.frame is None:
            self._event.wait(self.timeout)
        return self.frame

    def close(self):
        self._running = False
        if self._thread is not None:
            self._thread.join(timeout=2.5)
            self._thread = None
        if self._socket is not None:
            self._socket.close(0)
            self._socket = None


class FakeCamera:
    """A fixed frame, stamped now at every read."""

    def __init__(self, rgb, depth, intrinsics, clock=time.time):
        self.rgb, self.depth, self.intrinsics, self.clock = rgb, depth, intrinsics, clock
        self.error = None

    def start(self):
        return self

    def read(self):
        return self.clock(), self.rgb, self.depth, self.intrinsics

    def close(self):
        pass
