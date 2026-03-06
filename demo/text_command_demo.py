"""Text command demo — dry-run REPL for testing without voice or robot.

Usage:
    python -m demo.text_command_demo

Type commands like "up small", "forward big", "stop", "quit".
"""

import logging
import sys

import yaml

from executor.executor import Executor
from executor.modes import ExecutionMode
from interface.text_interface import TextCommandInterface

logger = logging.getLogger(__name__)


def main() -> None:
    config_path = "config/robot_config.yaml"
    with open(config_path) as f:
        config = yaml.safe_load(f)

    mode = config["execution"]["mode"]
    arm = config["robot"]["active_arm"]
    sim = config["robot"]["simulation_mode"]

    print("╔══════════════════════════════════════════╗")
    print("║  Endoscope Control — Text Command Demo   ║")
    print(f"║  Mode: {mode:<8} | Arm: {arm:<5} | Sim: {str(sim):<5} ║")
    print("╚══════════════════════════════════════════╝")
    print()
    print("Commands: up/down/left/right/forward/back/rl/rr/stop")
    print("Magnitude: small/mid/big (default: mid)")
    print('Examples: "up small", "forward big", "stop"')
    print('Type "quit" to exit.')
    print()

    executor = Executor(config_path, controller=None)
    executor.mode = ExecutionMode.AUTOPILOT  # skip trigger prompts in dry-run
    interface = TextCommandInterface()
    executor.run_loop(interface)


if __name__ == "__main__":
    logging.basicConfig(
        level=logging.INFO,
        format="%(name)s — %(message)s",
    )
    main()
