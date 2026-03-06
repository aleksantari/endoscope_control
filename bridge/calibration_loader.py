"""CalibrationLoader — loads hand-eye calibration transform T_ee_cam from YAML."""

import logging
from typing import Tuple

import numpy as np
import yaml

logger = logging.getLogger(__name__)


class CalibrationLoader:
    """Loads and validates the 4x4 hand-eye calibration transform T_ee_cam."""

    def __init__(self, config_path: str) -> None:
        """Load T_ee_cam from a YAML file.

        The file should contain a 4x4 matrix under key 'T_ee_cam'.

        Args:
            config_path: Path to the YAML file containing the calibration matrix.

        Raises:
            FileNotFoundError: If the file does not exist.
            ValueError: If the matrix is malformed or not a valid SE3 transform.
        """
        with open(config_path) as f:
            data = yaml.safe_load(f)

        if "T_ee_cam" not in data:
            raise ValueError(f"Key 'T_ee_cam' not found in {config_path}")

        matrix = np.array(data["T_ee_cam"], dtype=np.float64)
        self._validate(matrix, config_path)
        self.T_ee_cam = matrix
        logger.info("Loaded T_ee_cam from %s", config_path)

    @staticmethod
    def _validate(matrix: np.ndarray, path: str) -> None:
        """Validate that the matrix is a proper 4x4 homogeneous transform."""
        if matrix.shape != (4, 4):
            raise ValueError(
                f"T_ee_cam in {path} must be 4x4, got {matrix.shape}"
            )

        expected_last_row = np.array([0.0, 0.0, 0.0, 1.0])
        if not np.allclose(matrix[3, :], expected_last_row):
            raise ValueError(
                f"T_ee_cam last row must be [0,0,0,1], got {matrix[3, :]}"
            )

        R = matrix[:3, :3]
        if not np.allclose(R.T @ R, np.eye(3), atol=1e-6):
            raise ValueError(
                f"T_ee_cam rotation part is not orthogonal in {path}"
            )

    def get_T_ee_cam(self) -> np.ndarray:
        """Return the 4x4 homogeneous transform from EE frame to camera frame."""
        return self.T_ee_cam.copy()

    def camera_to_ee_translation(self, delta_cam: np.ndarray) -> np.ndarray:
        """Transform a 3D translation vector from camera frame to EE frame.

        Uses only the rotation part of T_ee_cam (not the translation offset).

        Args:
            delta_cam: shape (3,) translation in camera frame.

        Returns:
            shape (3,) translation in EE frame.
        """
        R = self.T_ee_cam[:3, :3]
        return R @ delta_cam

    def camera_to_ee_rotation(
        self, axis_cam: np.ndarray, angle_rad: float
    ) -> Tuple[np.ndarray, float]:
        """Transform a rotation axis from camera frame to EE frame.

        Args:
            axis_cam: shape (3,) unit rotation axis in camera frame.
            angle_rad: rotation angle in radians.

        Returns:
            (axis_ee, angle_rad) — axis in EE frame, angle unchanged.
        """
        R = self.T_ee_cam[:3, :3]
        axis_ee = R @ axis_cam
        return axis_ee / np.linalg.norm(axis_ee), angle_rad
