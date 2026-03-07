"""Tests for ZMQ interface and command serialization round-trip."""

import json
import threading
import time
from unittest.mock import patch

import pytest
import zmq

from interface.text_interface import TextCommandInterface
from interface.zmq_interface import ZMQCommandInterface
from protocol.command_schema import ActionType, MagnitudeLevel, RobotCommand


class TestZMQRoundTrip:
    def test_serialization_round_trip_via_zmq(self):
        """Serialize RobotCommand -> ZMQ send -> receive -> deserialize."""
        port = 5558
        original = RobotCommand(
            action=ActionType.MOVE_UP,
            magnitude=MagnitudeLevel.SMALL,
            confidence=0.95,
            value_mm=2.0,
            raw_text="move up a little",
        )
        received = {}

        def publisher():
            ctx = zmq.Context()
            sock = ctx.socket(zmq.PUB)
            sock.bind(f"tcp://*:{port}")
            time.sleep(0.3)
            sock.send_string(original.to_json_string())
            time.sleep(0.1)
            sock.close()
            ctx.term()

        def subscriber():
            ctx = zmq.Context()
            sock = ctx.socket(zmq.SUB)
            sock.connect(f"tcp://localhost:{port}")
            sock.setsockopt_string(zmq.SUBSCRIBE, "")
            poller = zmq.Poller()
            poller.register(sock, zmq.POLLIN)
            socks = dict(poller.poll(2000))
            if sock in socks:
                msg = sock.recv_string()
                received["cmd"] = RobotCommand.from_json_string(msg)
            sock.close()
            ctx.term()

        t = threading.Thread(target=publisher)
        t.start()
        subscriber()
        t.join()

        assert "cmd" in received
        cmd = received["cmd"]
        assert cmd.action == original.action
        assert cmd.magnitude == original.magnitude
        assert cmd.confidence == original.confidence
        assert cmd.value_mm == original.value_mm
        assert cmd.raw_text == original.raw_text


class TestZMQMalformedMessages:
    def test_invalid_json_returns_none(self):
        """Malformed JSON should return None, not crash."""
        port = 5559
        received = {"result": "not_set"}

        def publisher():
            ctx = zmq.Context()
            sock = ctx.socket(zmq.PUB)
            sock.bind(f"tcp://*:{port}")
            time.sleep(0.3)
            sock.send_string("this is not valid json{{{")
            time.sleep(0.1)
            sock.close()
            ctx.term()

        t = threading.Thread(target=publisher)
        t.start()

        iface = ZMQCommandInterface(f"tcp://localhost:{port}", timeout_ms=2000)
        cmd = iface.get_next_command()
        received["result"] = cmd
        iface.close()
        t.join()

        assert received["result"] is None

    def test_invalid_schema_returns_none(self):
        """Valid JSON but invalid schema should return None, not crash."""
        port = 5560
        received = {"result": "not_set"}

        def publisher():
            ctx = zmq.Context()
            sock = ctx.socket(zmq.PUB)
            sock.bind(f"tcp://*:{port}")
            time.sleep(0.3)
            sock.send_string(json.dumps({"action": "INVALID_ACTION", "magnitude": "SMALL"}))
            time.sleep(0.1)
            sock.close()
            ctx.term()

        t = threading.Thread(target=publisher)
        t.start()

        iface = ZMQCommandInterface(f"tcp://localhost:{port}", timeout_ms=2000)
        cmd = iface.get_next_command()
        received["result"] = cmd
        iface.close()
        t.join()

        assert received["result"] is None


class TestTextCommandInterface:
    def test_parse_up_small(self):
        """'up small' -> MOVE_UP / SMALL."""
        iface = TextCommandInterface()
        with patch("builtins.input", return_value="up small"):
            cmd = iface.get_next_command()
        assert cmd is not None
        assert cmd.action == ActionType.MOVE_UP
        assert cmd.magnitude == MagnitudeLevel.SMALL
        assert cmd.value_mm == 2.0

    def test_parse_forward_big(self):
        """'forward big' -> MOVE_FORWARD / BIG."""
        iface = TextCommandInterface()
        with patch("builtins.input", return_value="forward big"):
            cmd = iface.get_next_command()
        assert cmd is not None
        assert cmd.action == ActionType.MOVE_FORWARD
        assert cmd.magnitude == MagnitudeLevel.BIG
        assert cmd.value_mm == 6.0

    def test_parse_stop(self):
        """'stop' -> STOP."""
        iface = TextCommandInterface()
        with patch("builtins.input", return_value="stop"):
            cmd = iface.get_next_command()
        assert cmd is not None
        assert cmd.action == ActionType.STOP

    def test_parse_default_magnitude(self):
        """'left' with no magnitude -> MID."""
        iface = TextCommandInterface()
        with patch("builtins.input", return_value="left"):
            cmd = iface.get_next_command()
        assert cmd is not None
        assert cmd.action == ActionType.MOVE_LEFT
        assert cmd.magnitude == MagnitudeLevel.MID

    def test_unknown_action_returns_none(self):
        """Unknown action returns None."""
        iface = TextCommandInterface()
        with patch("builtins.input", return_value="jump high"):
            cmd = iface.get_next_command()
        assert cmd is None

    def test_quit_raises_keyboard_interrupt(self):
        """'quit' raises KeyboardInterrupt to exit run_loop."""
        iface = TextCommandInterface()
        with patch("builtins.input", return_value="quit"):
            with pytest.raises(KeyboardInterrupt):
                iface.get_next_command()

    def test_empty_returns_none(self):
        """Empty input returns None."""
        iface = TextCommandInterface()
        with patch("builtins.input", return_value=""):
            cmd = iface.get_next_command()
        assert cmd is None

    def test_rotate_left(self):
        """'rl mid' -> ROTATE_LEFT / MID."""
        iface = TextCommandInterface()
        with patch("builtins.input", return_value="rl mid"):
            cmd = iface.get_next_command()
        assert cmd is not None
        assert cmd.action == ActionType.ROTATE_LEFT
        assert cmd.magnitude == MagnitudeLevel.MID

    def test_shorthand_magnitudes(self):
        """'right s' -> MOVE_RIGHT / SMALL (shorthand)."""
        iface = TextCommandInterface()
        with patch("builtins.input", return_value="right s"):
            cmd = iface.get_next_command()
        assert cmd is not None
        assert cmd.action == ActionType.MOVE_RIGHT
        assert cmd.magnitude == MagnitudeLevel.SMALL
