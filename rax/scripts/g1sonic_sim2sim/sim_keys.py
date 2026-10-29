"""Keys to sim_headless.py over its ZMQ key port: lower the hanging G1 to the floor, then release it."""
import time

import zmq

KEY_PORT = 5590


def send_keys(keys, pause=0.8, port=KEY_PORT):
    pub = zmq.Context.instance().socket(zmq.PUB)
    pub.bind(f"tcp://127.0.0.1:{port}")
    time.sleep(0.5)  # the sim's SUB joins
    for key in keys:
        pub.send_string(key)
        time.sleep(pause)
    pub.close(0)


def lower_and_release(steps=2):
    """The gantry: lower the band 0.1 m per step (the default hang leaves the feet 0.2 m up), then let go."""
    send_keys([f"band={-0.1 * steps:.1f}", "release"])
