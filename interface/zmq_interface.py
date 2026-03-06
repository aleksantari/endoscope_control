"""ZMQCommandInterface — ZMQ subscriber receiving commands from voice_control."""

import json
import logging
from typing import Optional

import zmq
from pydantic import ValidationError

from interface.command_interface import CommandInterface
from protocol.command_schema import RobotCommand

logger = logging.getLogger(__name__)


class ZMQCommandInterface(CommandInterface):
    """Subscribes to RobotCommand JSON messages over ZMQ PUB/SUB."""

    def __init__(
        self, address: str = "tcp://localhost:5556", timeout_ms: int = 100
    ) -> None:
        """Connect to ZMQ PUB socket from voice_control pipeline.

        Args:
            address: ZMQ address to connect to.
            timeout_ms: Poll timeout in milliseconds (non-blocking).
        """
        self.context = zmq.Context()
        self.socket = self.context.socket(zmq.SUB)
        self.socket.connect(address)
        self.socket.setsockopt_string(zmq.SUBSCRIBE, "")
        self.timeout_ms = timeout_ms
        self.poller = zmq.Poller()
        self.poller.register(self.socket, zmq.POLLIN)
        logger.info("ZMQ subscriber connected to %s", address)

    def get_next_command(self) -> Optional[RobotCommand]:
        """Non-blocking poll for next command. Returns None if no command available."""
        socks = dict(self.poller.poll(self.timeout_ms))
        if self.socket in socks:
            message = self.socket.recv_string()
            logger.debug("Received ZMQ message: %s", message)
            try:
                return RobotCommand.from_json_string(message)
            except (json.JSONDecodeError, ValidationError) as e:
                logger.warning("Malformed ZMQ message, skipping: %s", e)
                return None
        return None

    def close(self) -> None:
        """Close ZMQ socket and terminate context."""
        self.socket.close()
        self.context.term()
        logger.info("ZMQ subscriber closed")
