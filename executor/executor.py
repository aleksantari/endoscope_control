"""Executor — main orchestrator for the endoscope control pipeline."""

import logging

import numpy as np
import yaml

from bridge.action_bridge import ActionBridge
from bridge.calibration_loader import CalibrationLoader
from executor.modes import ExecutionMode, TriggerPrompt
from executor.safety import SafetyModule
from interface.command_interface import CommandInterface
from protocol.command_schema import RobotCommand

logger = logging.getLogger(__name__)


class Executor:
    """Main orchestrator: receives commands, checks safety, executes motion.

    Pipeline:
        command → ActionBridge (SE3 delta) → SafetyModule → [TriggerPrompt]
        → SingleArmController → robot
    """

    def __init__(
        self,
        config_path: str = "config/robot_config.yaml",
        controller=None,
    ) -> None:
        """Initialize executor from config file.

        Args:
            config_path: Path to robot_config.yaml.
            controller: SingleArmController instance. None → dry-run mode.
        """
        with open(config_path) as f:
            self.config = yaml.safe_load(f)

        self.calibration = CalibrationLoader(
            self.config["calibration"]["transform_path"]
        )
        self.bridge = ActionBridge(self.calibration, self.config)
        self.safety = SafetyModule(self.config)
        self.mode = ExecutionMode(self.config["execution"]["mode"])
        self.controller = controller

    def process_command(self, command: RobotCommand) -> dict:
        """Process a single command through the full pipeline.

        Args:
            command: Validated RobotCommand to execute.

        Returns:
            Dict with keys: executed (bool), and additional context keys
            (action, reason, delta, success, target_pose, solved_q, error).
        """
        # STOP: bypass bridge, immediately freeze
        if command.is_stop:
            if self.controller is not None:
                self.controller.freeze()
            return {"executed": True, "action": "FREEZE", "reason": "STOP command"}

        # Step 1: Convert to SE3 delta in EE frame
        delta_se3 = self.bridge.command_to_delta(command)

        # Step 2: Safety check on delta magnitude
        safety_result = self.safety.check_delta(delta_se3)
        if not safety_result.safe:
            logger.warning("Safety rejected delta: %s", safety_result.reason)
            return {"executed": False, "reason": safety_result.reason}

        # Step 3: Trigger mode — prompt operator
        if self.mode == ExecutionMode.TRIGGER:
            delta_desc = self._describe_delta(delta_se3)
            approved = TriggerPrompt.prompt_operator(command, delta_desc)
            if not approved:
                return {"executed": False, "reason": "operator_rejected"}

        # Step 4: Dry-run if no controller
        if self.controller is None:
            t_mm = delta_se3[:3, 3] * 1000.0
            R = delta_se3[:3, :3]
            skew = R - R.T
            angle_deg = np.degrees(
                np.arctan2(
                    np.sqrt(skew[0, 1] ** 2 + skew[0, 2] ** 2 + skew[1, 2] ** 2) / 2,
                    (np.trace(R) - 1) / 2,
                )
            )
            if skew[1, 0] < 0:
                angle_deg = -angle_deg
            matrix_str = "\n".join(
                f"    [{delta_se3[i, 0]:+.6f}  {delta_se3[i, 1]:+.6f}  {delta_se3[i, 2]:+.6f}  {delta_se3[i, 3]:+.6f}]"
                for i in range(4)
            )
            print(f"\n  [DRY RUN] {command.action.value}")
            print("  SE3 Delta:")
            print(matrix_str)
            print(f"  Translation: dx={t_mm[0]:+.2f}  dy={t_mm[1]:+.2f}  dz={t_mm[2]:+.2f} mm")
            print(f"  Rotation:    {angle_deg:+.2f} deg\n")
            return {
                "executed": False,
                "reason": "dry_run",
                "delta": delta_se3.tolist(),
            }

        # Step 5: Execute
        result = self.controller.execute_delta(delta_se3)
        return {"executed": result["success"], **result}

    def run_loop(self, interface: CommandInterface) -> None:
        """Main control loop: poll interface, process each command.

        Args:
            interface: CommandInterface to poll for commands.
        """
        logger.info("Executor running in %s mode", self.mode.value)
        try:
            while True:
                command = interface.get_next_command()
                if command is None:
                    continue
                logger.info(
                    "Received: %s / %s", command.action.value, command.magnitude
                )
                result = self.process_command(command)
                logger.info("Result: %s", result)
        except KeyboardInterrupt:
            logger.info("Executor stopped by operator.")
            if self.controller is not None:
                self.controller.freeze()
        finally:
            interface.close()

    def _describe_delta(self, delta_se3: np.ndarray) -> str:
        """Return human-readable description of a delta for operator prompt."""
        t_mm = delta_se3[:3, 3] * 1000.0
        return f"[{t_mm[0]:.1f}, {t_mm[1]:.1f}, {t_mm[2]:.1f}] mm"
