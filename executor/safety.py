"""SafetyModule — workspace bounds, joint limits, step size, and heartbeat checks."""

import logging
from dataclasses import dataclass

import numpy as np

logger = logging.getLogger(__name__)


@dataclass
class SafetyResult:
    """Result of a safety check."""

    safe: bool
    reason: str = ""


class SafetyModule:
    """Validates deltas, target poses, and IK solutions against safety limits."""

    def __init__(self, config: dict) -> None:
        """Initialize from the full robot config dict.

        Extracts safety and control sections:
          - workspace_bounds (x/y/z min/max, already in meters)
          - max_translation_step (mm in config -> converted to meters)
          - max_rotation_step (degrees in config -> converted to radians)
        """
        safety = config["safety"]
        control = config["control"]

        bounds = safety["workspace_bounds"]
        self.x_min = bounds["x_min"]
        self.x_max = bounds["x_max"]
        self.y_min = bounds["y_min"]
        self.y_max = bounds["y_max"]
        self.z_min = bounds["z_min"]
        self.z_max = bounds["z_max"]

        self.max_translation_step = control["max_translation_step"] / 1000.0  # mm -> m
        self.max_rotation_step = np.deg2rad(control["max_rotation_step"])  # deg -> rad
        self.heartbeat_timeout_ms = safety.get("heartbeat_timeout_ms", 200)

    def check_delta(self, delta_se3: np.ndarray) -> SafetyResult:
        """Check if a proposed SE3 delta is within safe step limits.

        Checks:
            1. Translation magnitude within max_translation_step
            2. Rotation magnitude within max_rotation_step
        """
        # Translation check
        translation = delta_se3[:3, 3]
        t_mag = np.linalg.norm(translation)
        if t_mag > self.max_translation_step:
            return SafetyResult(
                safe=False,
                reason=f"Translation {t_mag*1000:.1f}mm exceeds limit "
                       f"{self.max_translation_step*1000:.1f}mm",
            )

        # Rotation check — extract angle from rotation matrix
        R = delta_se3[:3, :3]
        cos_angle = (np.trace(R) - 1.0) / 2.0
        cos_angle = np.clip(cos_angle, -1.0, 1.0)
        angle = np.abs(np.arccos(cos_angle))
        if angle > self.max_rotation_step:
            return SafetyResult(
                safe=False,
                reason=f"Rotation {np.rad2deg(angle):.1f}deg exceeds limit "
                       f"{np.rad2deg(self.max_rotation_step):.1f}deg",
            )

        return SafetyResult(safe=True)

    def check_target_pose(self, target_position: np.ndarray) -> SafetyResult:
        """Check if a target EE position is within workspace bounds.

        Args:
            target_position: shape (3,) xyz in the robot base frame (meters).
        """
        x, y, z = target_position
        if not (self.x_min <= x <= self.x_max):
            return SafetyResult(
                safe=False,
                reason=f"X={x:.3f}m outside bounds [{self.x_min}, {self.x_max}]",
            )
        if not (self.y_min <= y <= self.y_max):
            return SafetyResult(
                safe=False,
                reason=f"Y={y:.3f}m outside bounds [{self.y_min}, {self.y_max}]",
            )
        if not (self.z_min <= z <= self.z_max):
            return SafetyResult(
                safe=False,
                reason=f"Z={z:.3f}m outside bounds [{self.z_min}, {self.z_max}]",
            )
        return SafetyResult(safe=True)

    def check_ik_result(
        self,
        converged: bool,
        joint_angles: np.ndarray,
        lower_limits: np.ndarray,
        upper_limits: np.ndarray,
    ) -> SafetyResult:
        """Check IK solution validity.

        Args:
            converged: Whether the IK solver converged.
            joint_angles: Solved joint angles.
            lower_limits: Per-joint lower position limits.
            upper_limits: Per-joint upper position limits.
        """
        if not converged:
            return SafetyResult(safe=False, reason="IK solver did not converge")

        tolerance = np.deg2rad(1.0)
        below = joint_angles < (lower_limits - tolerance)
        above = joint_angles > (upper_limits + tolerance)
        if np.any(below) or np.any(above):
            violations = np.where(below | above)[0]
            return SafetyResult(
                safe=False,
                reason=f"Joint limit violation at indices {violations.tolist()}",
            )

        return SafetyResult(safe=True)

    def check_all(
        self,
        delta_se3: np.ndarray,
        target_position: np.ndarray,
        ik_converged: bool,
        joint_angles: np.ndarray,
        lower_limits: np.ndarray,
        upper_limits: np.ndarray,
    ) -> SafetyResult:
        """Run all safety checks. Return first failure, or safe=True."""
        result = self.check_delta(delta_se3)
        if not result.safe:
            return result

        result = self.check_target_pose(target_position)
        if not result.safe:
            return result

        result = self.check_ik_result(ik_converged, joint_angles, lower_limits, upper_limits)
        if not result.safe:
            return result

        return SafetyResult(safe=True)
