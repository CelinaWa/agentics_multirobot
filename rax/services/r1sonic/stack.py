"""Client of the r1 stack protocol (JSON over ZMQ REQ/REP): the R1 sim server (r1/sim/r1_sim_server.py) or the
real-robot bridge. One request at a time; a timeout drops and recreates the socket so a lost reply cannot wedge it."""
import json
import threading
import numpy as np
import zmq


class StackError(RuntimeError):
    pass


class StackClient:
    def __init__(self, host, port, timeout_s=3.0):
        self.endpoint = f'tcp://{host}:{port}'
        self.timeout_ms = int(timeout_s * 1000)
        self.lock = threading.Lock()
        self.ctx = zmq.Context.instance()
        self.sock = None
        self.requests = 0
        self.failures = 0

    def _socket(self):
        if self.sock is None:
            self.sock = self.ctx.socket(zmq.REQ)
            self.sock.setsockopt(zmq.LINGER, 0)
            self.sock.setsockopt(zmq.RCVTIMEO, self.timeout_ms)
            self.sock.setsockopt(zmq.SNDTIMEO, self.timeout_ms)
            self.sock.connect(self.endpoint)
        return self.sock

    def request(self, op, **fields):
        with self.lock:
            sock = self._socket()
            try:
                sock.send(json.dumps({'op': op, **fields}).encode())
                parts = sock.recv_multipart()
            except zmq.ZMQError as exc:
                self.sock.close(); self.sock = None; self.failures += 1
                raise StackError(f'no reply from the r1 stack at {self.endpoint} ({exc})') from None
            self.requests += 1
        reply = json.loads(parts[0])
        if not reply.get('ok'):
            raise StackError(reply.get('error', 'the r1 stack refused the request'))
        return reply, parts[1:]

    def walk(self, forward_m, turn_deg=0.0):
        """A walk blocks for the gait duration plus a settle; use a long receive timeout for this call only."""
        with self.lock:
            sock = self._socket()
            sock.setsockopt(zmq.RCVTIMEO, 40000)
            try:
                sock.send(json.dumps({'op': 'walk', 'forward_m': float(forward_m), 'turn_deg': float(turn_deg)}).encode())
                parts = sock.recv_multipart()
            except zmq.ZMQError as exc:
                self.sock.close(); self.sock = None; self.failures += 1
                raise StackError(f'no reply from the r1 stack walk at {self.endpoint} ({exc})') from None
            finally:
                if self.sock is not None:
                    try:
                        self.sock.setsockopt(zmq.RCVTIMEO, self.timeout_ms)
                    except Exception:
                        pass
            self.requests += 1
        reply = json.loads(parts[0])
        if not reply.get('ok'):
            raise StackError(reply.get('error', 'the r1 stack refused the walk'))
        return reply

    def _blocking(self, op, **fields):
        """A manipulation primitive that blocks for the motion plus a settle; use a long receive timeout."""
        with self.lock:
            sock = self._socket()
            sock.setsockopt(zmq.RCVTIMEO, 120000)
            try:
                sock.send(json.dumps({'op': op, **fields}).encode())
                parts = sock.recv_multipart()
            except zmq.ZMQError as exc:
                self.sock.close(); self.sock = None; self.failures += 1
                raise StackError(f'no reply from the r1 stack {op} at {self.endpoint} ({exc})') from None
            finally:
                if self.sock is not None:
                    try:
                        self.sock.setsockopt(zmq.RCVTIMEO, self.timeout_ms)
                    except Exception:
                        pass
            self.requests += 1
        reply = json.loads(parts[0])
        if not reply.get('ok'):
            raise StackError(reply.get('error', f'the r1 stack refused {op}'))
        return reply

    def grasp_paw(self, cmdopen=0.05, wedge=0.06, liftz=0.20):
        return self._blocking('grasp_paw', cmdopen=float(cmdopen), wedge=float(wedge), liftz=float(liftz))

    def place_in_box(self, side, release=True):
        return self._blocking('place_in_box', side=side, release=bool(release))

    def slide_table(self, dx, dy=0.0):
        return self.request('slide_table', dx=float(dx), dy=float(dy))[0]

    def state(self):
        return self.request('state')[0]

    def stream(self, rows):
        return self.request('stream', rows=np.asarray(rows, dtype=float).round(6).tolist())[0]

    def hold(self):
        return self.request('hold')[0]

    def camera(self):
        meta, (rgb, depth) = self.request('camera')
        h, w = meta['height'], meta['width']
        return (meta, np.frombuffer(rgb, dtype=np.uint8).reshape(h, w, 3),
                np.frombuffer(depth, dtype=np.float32).reshape(h, w))

    def close(self):
        with self.lock:
            if self.sock is not None:
                self.sock.close(); self.sock = None
