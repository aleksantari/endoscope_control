"""ExecutionMode enum and TriggerPrompt for operator confirmation."""

import logging
from enum import Enum

from protocol.command_schema import RobotCommand

logger = logging.getLogger(__name__)


class ExecutionMode(str, Enum):
    """How commands are executed after passing safety checks."""

    AUTOPILOT = "autopilot"
    TRIGGER = "trigger"


class TriggerPrompt:
    """Handles operator confirmation in trigger mode."""

    @staticmethod
    def prompt_operator(command: RobotCommand, delta_description: str) -> bool:
        """Display the command and intended motion, ask for y/n confirmation.

        Args:
            command: The RobotCommand to display.
            delta_description: Human-readable description of the intended motion.

        Returns:
            True if operator approves, False otherwise.
        """
        mag_str = command.magnitude.value if command.magnitude else "N/A"
        unit = "deg" if command.is_rotation else "mm"
        value = command.value_mm

        print("╔══════════════════════════════════════╗")
        print("║  COMMAND RECEIVED                    ║")
        print("╠══════════════════════════════════════╣")
        print(f"║  Action:     {command.action.value:<23}║")
        print(f"║  Magnitude:  {mag_str} ({value} {unit}){' ' * max(0, 16 - len(f'{mag_str} ({value} {unit})'))}║")
        print(f"║  Confidence: {command.confidence:<23}║")
        print(f"║  Raw text:   \"{command.raw_text}\"{' ' * max(0, 21 - len(command.raw_text))}║")
        print(f"║  Delta:      {delta_description:<23}║")
        print("╚══════════════════════════════════════╝")

        try:
            response = input("Execute? [y/n]: ").strip().lower()
            return response in ("y", "yes")
        except (EOFError, KeyboardInterrupt):
            return False
