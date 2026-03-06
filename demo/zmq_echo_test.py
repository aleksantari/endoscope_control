"""ZMQ echo test — publish and receive a RobotCommand in the same process."""

import json
import threading
import time

import zmq


def publisher(port: int) -> None:
    """Publish a single test command."""
    ctx = zmq.Context()
    sock = ctx.socket(zmq.PUB)
    sock.bind(f"tcp://*:{port}")
    time.sleep(0.3)  # slow joiner
    cmd = {
        "action": "MOVE_UP",
        "magnitude": "SMALL",
        "frame": "CAMERA",
        "confidence": 0.95,
        "value_mm": 2.0,
        "raw_text": "test",
    }
    sock.send_string(json.dumps(cmd))
    time.sleep(0.1)
    sock.close()
    ctx.term()


def subscriber(port: int) -> None:
    """Subscribe and verify the test command arrives."""
    ctx = zmq.Context()
    sock = ctx.socket(zmq.SUB)
    sock.connect(f"tcp://localhost:{port}")
    sock.setsockopt_string(zmq.SUBSCRIBE, "")
    poller = zmq.Poller()
    poller.register(sock, zmq.POLLIN)
    socks = dict(poller.poll(2000))
    if sock in socks:
        msg = sock.recv_string()
        data = json.loads(msg)
        print(f"RECEIVED: {data}")
        assert data["action"] == "MOVE_UP"
        print("ZMQ echo test PASSED")
    else:
        print("ZMQ echo test FAILED — no message received")
    sock.close()
    ctx.term()


if __name__ == "__main__":
    port = 5557  # different port to avoid conflicts
    t = threading.Thread(target=publisher, args=(port,))
    t.start()
    subscriber(port)
    t.join()
