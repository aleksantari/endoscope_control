"""FKSolver — forward kinematics using Pinocchio reduced model from IK solver."""

import logging
from typing import Tuple

import numpy as np

logger = logging.getLogger(__name__)


class FKSolver:
    """Forward kinematics using the same Pinocchio model as the IK solver.

    Shares the reduced model (14 arm DOFs) with G1_29_ArmIK to avoid
    duplicate URDF loading.
    """

    def __init__(self, reduced_robot_model) -> None:
        """Initialize FK solver with the reduced Pinocchio model.

        Args:
            reduced_robot_model: Pinocchio Model (the reduced model from G1_29_ArmIK
                                 with legs/waist/hands locked, only 14 arm DOFs).
        """
        try:
            import pinocchio as pin
            self._pin = pin
        except ImportError:
            raise ImportError("pinocchio is required for FKSolver")

        self.model = reduced_robot_model
        self.data = self.model.createData()
        self.L_ee_id = self.model.getFrameId("L_ee")
        self.R_ee_id = self.model.getFrameId("R_ee")

    def compute_ee_poses(self, q: np.ndarray) -> Tuple[np.ndarray, np.ndarray]:
        """Compute left and right end-effector poses from joint angles.

        Args:
            q: shape (14,) — joint angles for both arms.

        Returns:
            (L_pose, R_pose) — each a 4x4 homogeneous transform.
        """
        self._pin.framesForwardKinematics(self.model, self.data, q)
        L_pose = self.data.oMf[self.L_ee_id].homogeneous
        R_pose = self.data.oMf[self.R_ee_id].homogeneous
        return L_pose.copy(), R_pose.copy()

    def compute_single_ee_pose(self, q: np.ndarray, arm: str = "left") -> np.ndarray:
        """Compute a single arm's end-effector pose.

        Args:
            q: shape (14,) — joint angles for both arms.
            arm: "left" or "right".

        Returns:
            4x4 homogeneous transform for the selected arm.
        """
        L_pose, R_pose = self.compute_ee_poses(q)
        return L_pose if arm == "left" else R_pose
