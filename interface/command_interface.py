"""CommandInterface — abstract base class for command sources."""

from abc import ABC, abstractmethod
from typing import Optional

from protocol.command_schema import RobotCommand


class CommandInterface(ABC):
    """Abstract base class for all command sources (ZMQ, text, future VLA)."""

    @abstractmethod
    def get_next_command(self) -> Optional[RobotCommand]:
        """Return next command, or None if no command available."""
        pass

    def close(self) -> None:
        """Cleanup resources."""
        pass
