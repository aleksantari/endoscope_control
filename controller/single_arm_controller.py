"""SingleArmController — wraps dual-arm G1_29 controller for single-arm endoscope use."""

import logging
from typing import Dict, Any

import numpy as np

from controller.fk_utils import FKSolver

logger = logging.getLogger(__name__)


class SingleArmController:
    """Wraps dual-arm controller + IK for single-arm endoscope control.

    Holds the inactive arm at its current position while moving only
    the active arm. Supports left or right arm selection.
    """

    def __init__(
        self,
        arm_controller,
        ik_solver,
        fk_solver: FKSolver,
        active_arm: str = "left",
    ) -> None:
        """Initialize controller.

        Args:
            arm_controller: G1_29_ArmController instance.
            ik_solver: G1_29_ArmIK instance.
            fk_solver: FKSolver instance (uses ik_solver's reduced model).
            active_arm: "left" or "right".
        """
        self.arm = arm_controller
        self.ik = ik_solver
        self.fk = fk_solver
        self.active_arm = active_arm

    @property
    def current_joint_q(self) -> np.ndarray:
        """Current dual-arm joint positions (14,)."""
        return self.arm.get_current_dual_arm_q()

    @property
    def current_ee_pose(self) -> np.ndarray:
        """Current end-effector pose for the active arm (4x4)."""
        q = self.current_joint_q
        return self.fk.compute_single_ee_pose(q, self.active_arm)

    def execute_delta(self, delta_se3: np.ndarray) -> Dict[str, Any]:
        """Apply an SE3 delta to the active arm's end-effector.

        Steps:
            1. Read current dual-arm joints
            2. FK both arms
            3. Apply delta to active arm, hold inactive arm at current pose
            4. IK solve for new joints
            5. Send to arm controller

        Args:
            delta_se3: 4x4 SE3 delta in EE frame.

        Returns:
            Dict with keys: success, target_pose, solved_q (on success),
            or success=False, error, target_pose (on failure).
        """
        current_q = self.current_joint_q
        current_dq = self.arm.get_current_dual_arm_dq()
        L_current, R_current = self.fk.compute_ee_poses(current_q)

        if self.active_arm == "left":
            L_target = L_current @ delta_se3
            R_target = R_current
        else:
            L_target = L_current
            R_target = R_current @ delta_se3

        try:
            sol_q, sol_tau = self.ik.solve_ik(L_target, R_target, current_q, current_dq)
            # Optional: force inactive arm joints to stay exactly at current values.
            # The IK solver should keep them still (target = current pose), but this
            # guarantees zero drift from regularization/smoothing costs.
            # if self.active_arm == "left":
            #     sol_q[7:] = current_q[7:]
            # else:
            #     sol_q[:7] = current_q[:7]
            self.arm.ctrl_dual_arm(sol_q, sol_tau)
            return {
                "success": True,
                "target_pose": L_target if self.active_arm == "left" else R_target,
                "solved_q": sol_q,
            }
        except Exception as e:
            logger.error("IK solve failed: %s", e)
            # FREEZE: hold current position, zero velocity
            self.arm.ctrl_dual_arm(current_q, np.zeros_like(current_q))
            return {
                "success": False,
                "error": str(e),
                "target_pose": L_target if self.active_arm == "left" else R_target,
            }

    def freeze(self) -> None:
        """Immediately hold current position (STOP/FREEZE behavior)."""
        current_q = self.current_joint_q
        self.arm.ctrl_dual_arm(current_q, np.zeros_like(current_q))
