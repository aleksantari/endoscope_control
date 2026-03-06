"""ActionBridge — converts RobotCommand to SE3 delta pose in the EE frame."""

import logging
from typing import Dict

import numpy as np

from bridge.calibration_loader import CalibrationLoader
from protocol.command_schema import ActionType, MagnitudeLevel, RobotCommand

logger = logging.getLogger(__name__)

# Camera-frame axis mapping for each action
_TRANSLATION_MAP: Dict[ActionType, np.ndarray] = {
    ActionType.MOVE_FORWARD: np.array([0.0, 0.0, 1.0]),   # +Z
    ActionType.RETRACT:      np.array([0.0, 0.0, -1.0]),   # -Z
    ActionType.MOVE_RIGHT:   np.array([1.0, 0.0, 0.0]),    # +X
    ActionType.MOVE_LEFT:    np.array([-1.0, 0.0, 0.0]),   # -X
    ActionType.MOVE_UP:      np.array([0.0, -1.0, 0.0]),   # -Y (optical: Y-down)
    ActionType.MOVE_DOWN:    np.array([0.0, 1.0, 0.0]),    # +Y
}

_ROTATION_MAP: Dict[ActionType, np.ndarray] = {
    ActionType.ROTATE_LEFT:  np.array([0.0, 0.0, 1.0]),    # +Rz (CCW)
    ActionType.ROTATE_RIGHT: np.array([0.0, 0.0, -1.0]),   # -Rz
}


def _axis_angle_to_rotation_matrix(axis: np.ndarray, angle: float) -> np.ndarray:
    """Convert axis-angle to 3x3 rotation matrix via Rodrigues' formula."""
    axis = axis / np.linalg.norm(axis)
    K = np.array([
        [0.0, -axis[2], axis[1]],
        [axis[2], 0.0, -axis[0]],
        [-axis[1], axis[0], 0.0],
    ])
    return np.eye(3) + np.sin(angle) * K + (1.0 - np.cos(angle)) * (K @ K)


class ActionBridge:
    """Converts RobotCommand to a 4x4 SE3 delta pose in the EE frame."""

    def __init__(self, calibration: CalibrationLoader, config: dict) -> None:
        """Initialize with calibration and magnitude config.

        Args:
            calibration: CalibrationLoader with T_ee_cam loaded.
            config: Full robot config dict (must contain 'magnitudes' key).
        """
        self.calibration = calibration
        mag_config = config["magnitudes"]
        self.translation_magnitudes = {
            MagnitudeLevel(k): v for k, v in mag_config["translation"].items()
        }
        self.rotation_magnitudes = {
            MagnitudeLevel(k): v for k, v in mag_config["rotation"].items()
        }

    def command_to_delta(self, command: RobotCommand) -> np.ndarray:
        """Convert a RobotCommand to a 4x4 SE3 delta pose in the EE frame.

        Args:
            command: The incoming RobotCommand.

        Returns:
            4x4 numpy array representing the SE3 delta in EE frame.
        """
        if command.is_stop:
            return np.eye(4)

        if command.is_translation:
            return self._translation_delta(command)

        if command.is_rotation:
            return self._rotation_delta(command)

        # Should not reach here given the enum, but be safe
        logger.warning("Unknown action type: %s, returning identity", command.action)
        return np.eye(4)

    def _translation_delta(self, command: RobotCommand) -> np.ndarray:
        """Build translation delta for a movement command."""
        magnitude_m = self._get_magnitude_meters(command)
        direction_cam = _TRANSLATION_MAP[command.action]
        delta_cam = direction_cam * magnitude_m

        # Transform to EE frame
        delta_ee = self.calibration.camera_to_ee_translation(delta_cam)

        T = np.eye(4)
        T[:3, 3] = delta_ee
        return T

    def _rotation_delta(self, command: RobotCommand) -> np.ndarray:
        """Build rotation delta for a rotation command.

        Uses the full similarity transform: delta_ee = T @ delta_cam @ T^(-1).
        This correctly handles T_ee_cam with a translation component — a pure
        rotation in camera frame produces both rotation and translation in EE
        frame when the camera is offset from the EE origin.
        """
        angle_rad = self._get_magnitude_radians(command)
        axis_cam = _ROTATION_MAP[command.action]

        # Build full 4x4 rotation delta in camera frame
        R_cam = _axis_angle_to_rotation_matrix(axis_cam, angle_rad)
        delta_cam = np.eye(4)
        delta_cam[:3, :3] = R_cam

        # Full similarity transform to EE frame
        T = self.calibration.get_T_ee_cam()
        T_inv = np.linalg.inv(T)
        return T @ delta_cam @ T_inv

    def _get_magnitude_meters(self, command: RobotCommand) -> float:
        """Return the translation magnitude in meters.

        Uses config lookup by magnitude level, but prefers command.value_mm
        when explicitly set (override from voice pipeline or VLA).
        """
        if "value_mm" in command.model_fields_set:
            return command.value_mm / 1000.0
        magnitude = command.magnitude or MagnitudeLevel.MID
        return self.translation_magnitudes[magnitude] / 1000.0

    def _get_magnitude_radians(self, command: RobotCommand) -> float:
        """Return the rotation magnitude in radians.

        For rotation commands, value_mm is repurposed as value_deg
        (per shared protocol). Prefers explicit value when set.
        """
        if "value_mm" in command.model_fields_set:
            return np.deg2rad(command.value_mm)
        magnitude = command.magnitude or MagnitudeLevel.MID
        return np.deg2rad(self.rotation_magnitudes[magnitude])
