"""TextCommandInterface — typed commands from stdin for testing."""

import logging
from typing import Dict, Optional

from interface.command_interface import CommandInterface
from protocol.command_schema import ActionType, MagnitudeLevel, RobotCommand

logger = logging.getLogger(__name__)


class TextCommandInterface(CommandInterface):
    """Read commands from stdin for testing.

    Accepts shorthand like: 'up small', 'left big', 'forward', 'stop'
    Parses into RobotCommand objects.
    """

    ACTION_MAP: Dict[str, ActionType] = {
        "up": ActionType.MOVE_UP,
        "down": ActionType.MOVE_DOWN,
        "left": ActionType.MOVE_LEFT,
        "right": ActionType.MOVE_RIGHT,
        "forward": ActionType.MOVE_FORWARD,
        "back": ActionType.RETRACT,
        "retract": ActionType.RETRACT,
        "rl": ActionType.ROTATE_LEFT,
        "rr": ActionType.ROTATE_RIGHT,
        "stop": ActionType.STOP,
    }

    MAGNITUDE_MAP: Dict[str, MagnitudeLevel] = {
        "small": MagnitudeLevel.SMALL,
        "s": MagnitudeLevel.SMALL,
        "mid": MagnitudeLevel.MID,
        "m": MagnitudeLevel.MID,
        "big": MagnitudeLevel.BIG,
        "b": MagnitudeLevel.BIG,
    }

    # Default value_mm for each magnitude (translation)
    _DEFAULT_VALUES: Dict[MagnitudeLevel, float] = {
        MagnitudeLevel.SMALL: 2.0,
        MagnitudeLevel.MID: 4.0,
        MagnitudeLevel.BIG: 6.0,
    }

    def get_next_command(self) -> Optional[RobotCommand]:
        """Read and parse a command from stdin.

        Returns:
            RobotCommand if valid input, None on quit/empty/unknown.
        """
        try:
            raw = input("Command> ").strip().lower()
        except (EOFError, KeyboardInterrupt):
            raise KeyboardInterrupt

        if not raw:
            return None
        if raw == "quit":
            raise KeyboardInterrupt

        parts = raw.split()
        action_str = parts[0]
        mag_str = parts[1] if len(parts) > 1 else "mid"

        action = self.ACTION_MAP.get(action_str)
        if action is None:
            print(
                f"Unknown action: {action_str}. "
                f"Options: {list(self.ACTION_MAP.keys())}"
            )
            return None

        magnitude = self.MAGNITUDE_MAP.get(mag_str, MagnitudeLevel.MID)
        value_mm = self._DEFAULT_VALUES[magnitude]

        return RobotCommand(
            action=action,
            magnitude=magnitude,
            value_mm=value_mm,
            raw_text=raw,
        )
