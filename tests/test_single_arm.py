"""Tests for controller/single_arm_controller.py and controller/fk_utils.py."""

from unittest.mock import MagicMock
import numpy as np
import pytest

from controller.single_arm_controller import SingleArmController


def _make_mock_fk(L_pose=None, R_pose=None):
    """Build a mock FKSolver that returns given EE poses."""
    if L_pose is None:
        L_pose = np.eye(4)
    if R_pose is None:
        R_pose = np.eye(4)
    fk = MagicMock()
    fk.compute_ee_poses.return_value = (L_pose.copy(), R_pose.copy())
    fk.compute_single_ee_pose.return_value = L_pose.copy()
    return fk


def _make_mock_arm(q=None, dq=None):
    """Build a mock arm controller with given joint state."""
    if q is None:
        q = np.zeros(14)
    if dq is None:
        dq = np.zeros(14)
    arm = MagicMock()
    arm.get_current_dual_arm_q.return_value = q.copy()
    arm.get_current_dual_arm_dq.return_value = dq.copy()
    return arm


def _make_mock_ik(sol_q=None, sol_tau=None):
    """Build a mock IK solver that returns given solution."""
    if sol_q is None:
        sol_q = np.ones(14) * 0.1
    if sol_tau is None:
        sol_tau = np.zeros(14)
    ik = MagicMock()
    ik.solve_ik.return_value = (sol_q.copy(), sol_tau.copy())
    return ik


class TestSingleArmControllerLeft:
    def test_execute_delta_calls_fk_and_ik(self):
        """execute_delta should call FK, then IK, then ctrl_dual_arm."""
        arm = _make_mock_arm()
        ik = _make_mock_ik()
        fk = _make_mock_fk()
        ctrl = SingleArmController(arm, ik, fk, active_arm="left")

        delta = np.eye(4)
        delta[0, 3] = 0.001
        result = ctrl.execute_delta(delta)

        assert result["success"] is True
        fk.compute_ee_poses.assert_called_once()
        ik.solve_ik.assert_called_once()
        arm.ctrl_dual_arm.assert_called_once()

    def test_execute_delta_applies_to_left_only(self):
        """Active=left: L_target = L_current @ delta, R_target = R_current."""
        L_pose = np.eye(4)
        L_pose[0, 3] = 0.3  # distinct position
        R_pose = np.eye(4)
        R_pose[1, 3] = 0.5

        arm = _make_mock_arm()
        ik = _make_mock_ik()
        fk = _make_mock_fk(L_pose=L_pose, R_pose=R_pose)
        ctrl = SingleArmController(arm, ik, fk, active_arm="left")

        delta = np.eye(4)
        delta[0, 3] = 0.01
        ctrl.execute_delta(delta)

        L_expected = L_pose @ delta
        call_args = ik.solve_ik.call_args[0]
        np.testing.assert_allclose(call_args[0], L_expected)  # L_target
        np.testing.assert_allclose(call_args[1], R_pose)      # R_target unchanged

    def test_execute_delta_applies_to_right_only(self):
        """Active=right: R_target = R_current @ delta, L_target = L_current."""
        L_pose = np.eye(4)
        R_pose = np.eye(4)
        R_pose[1, 3] = 0.5

        arm = _make_mock_arm()
        ik = _make_mock_ik()
        fk = _make_mock_fk(L_pose=L_pose, R_pose=R_pose)
        ctrl = SingleArmController(arm, ik, fk, active_arm="right")

        delta = np.eye(4)
        delta[0, 3] = 0.01
        ctrl.execute_delta(delta)

        R_expected = R_pose @ delta
        call_args = ik.solve_ik.call_args[0]
        np.testing.assert_allclose(call_args[0], L_pose)      # L_target unchanged
        np.testing.assert_allclose(call_args[1], R_expected)  # R_target

    def test_execute_delta_freezes_on_ik_failure(self):
        """If IK raises an exception, freeze (send current q with zero velocity)."""
        q = np.ones(14) * 0.5
        arm = _make_mock_arm(q=q)
        ik = MagicMock()
        ik.solve_ik.side_effect = RuntimeError("IK did not converge")
        fk = _make_mock_fk()
        ctrl = SingleArmController(arm, ik, fk, active_arm="left")

        result = ctrl.execute_delta(np.eye(4))

        assert result["success"] is False
        assert "IK did not converge" in result["error"]
        # ctrl_dual_arm should be called with current q and zeros
        call_args = arm.ctrl_dual_arm.call_args[0]
        np.testing.assert_allclose(call_args[0], q)
        np.testing.assert_allclose(call_args[1], np.zeros(14))

    def test_freeze_sends_current_q_with_zero_velocity(self):
        """freeze() should read current q and send it with zero velocity."""
        q = np.array([0.1] * 14)
        arm = _make_mock_arm(q=q)
        ik = _make_mock_ik()
        fk = _make_mock_fk()
        ctrl = SingleArmController(arm, ik, fk)

        ctrl.freeze()

        arm.ctrl_dual_arm.assert_called_once()
        call_args = arm.ctrl_dual_arm.call_args[0]
        np.testing.assert_allclose(call_args[0], q)
        np.testing.assert_allclose(call_args[1], np.zeros(14))

    def test_execute_delta_returns_solved_q(self):
        """execute_delta should return solved_q in result on success."""
        sol_q = np.linspace(0, 1, 14)
        arm = _make_mock_arm()
        ik = _make_mock_ik(sol_q=sol_q)
        fk = _make_mock_fk()
        ctrl = SingleArmController(arm, ik, fk)

        result = ctrl.execute_delta(np.eye(4))

        assert result["success"] is True
        np.testing.assert_allclose(result["solved_q"], sol_q)


@pytest.mark.skip(reason="requires DDS and physical/simulated robot")
class TestSingleArmControllerDDS:
    def test_real_initialization(self):
        """Requires DDS — instantiate with real G1_29_ArmController."""
        pass
