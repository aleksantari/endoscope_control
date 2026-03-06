"""Tests for executor/safety.py — workspace bounds, step limits, IK checks."""

import numpy as np
import pytest
import yaml

from executor.safety import SafetyModule, SafetyResult


@pytest.fixture
def safety():
    """SafetyModule with default config."""
    config = yaml.safe_load(open("config/robot_config.yaml"))
    return SafetyModule(config)


class TestCheckDelta:
    def test_small_translation_passes(self, safety):
        """1mm translation should pass (limit is 10mm)."""
        delta = np.eye(4)
        delta[0, 3] = 0.001  # 1mm in X
        result = safety.check_delta(delta)
        assert result.safe

    def test_huge_translation_fails(self, safety):
        """100mm translation should fail."""
        delta = np.eye(4)
        delta[2, 3] = 0.1  # 100mm in Z
        result = safety.check_delta(delta)
        assert not result.safe
        assert "exceeds limit" in result.reason

    def test_at_limit_translation_passes(self, safety):
        """Exactly at the limit should pass."""
        delta = np.eye(4)
        delta[0, 3] = 0.01  # 10mm = max_translation_step
        result = safety.check_delta(delta)
        assert result.safe

    def test_identity_delta_passes(self, safety):
        """Identity (no motion) should always pass."""
        result = safety.check_delta(np.eye(4))
        assert result.safe

    def test_large_rotation_fails(self, safety):
        """10 degree rotation should fail (limit is 5 degrees)."""
        angle = np.deg2rad(10.0)
        R = np.array([
            [np.cos(angle), -np.sin(angle), 0.0],
            [np.sin(angle),  np.cos(angle), 0.0],
            [0.0, 0.0, 1.0],
        ])
        delta = np.eye(4)
        delta[:3, :3] = R
        result = safety.check_delta(delta)
        assert not result.safe
        assert "Rotation" in result.reason

    def test_small_rotation_passes(self, safety):
        """2 degree rotation should pass (limit is 5 degrees)."""
        angle = np.deg2rad(2.0)
        R = np.array([
            [np.cos(angle), -np.sin(angle), 0.0],
            [np.sin(angle),  np.cos(angle), 0.0],
            [0.0, 0.0, 1.0],
        ])
        delta = np.eye(4)
        delta[:3, :3] = R
        result = safety.check_delta(delta)
        assert result.safe


class TestCheckTargetPose:
    def test_inside_bounds_passes(self, safety):
        """Position well inside workspace bounds."""
        pos = np.array([0.1, 0.1, 0.2])
        result = safety.check_target_pose(pos)
        assert result.safe

    def test_outside_x_fails(self, safety):
        """Position outside X bounds."""
        pos = np.array([1.0, 0.1, 0.2])  # x_max is 0.5
        result = safety.check_target_pose(pos)
        assert not result.safe
        assert "X=" in result.reason

    def test_outside_y_fails(self, safety):
        """Position outside Y bounds."""
        pos = np.array([0.1, -0.5, 0.2])  # y_min is -0.3
        result = safety.check_target_pose(pos)
        assert not result.safe
        assert "Y=" in result.reason

    def test_outside_z_fails(self, safety):
        """Position outside Z bounds."""
        pos = np.array([0.1, 0.1, 1.0])  # z_max is 0.6
        result = safety.check_target_pose(pos)
        assert not result.safe
        assert "Z=" in result.reason

    def test_at_boundary_passes(self, safety):
        """Exactly at the boundary should pass."""
        pos = np.array([0.5, 0.5, 0.6])  # at max bounds
        result = safety.check_target_pose(pos)
        assert result.safe


class TestCheckIKResult:
    def test_non_convergence_fails(self, safety):
        """IK non-convergence should fail."""
        q = np.zeros(7)
        lower = -np.ones(7)
        upper = np.ones(7)
        result = safety.check_ik_result(False, q, lower, upper)
        assert not result.safe
        assert "converge" in result.reason

    def test_converged_within_limits_passes(self, safety):
        """Converged solution within limits should pass."""
        q = np.zeros(7)
        lower = -np.ones(7)
        upper = np.ones(7)
        result = safety.check_ik_result(True, q, lower, upper)
        assert result.safe

    def test_joint_outside_limits_fails(self, safety):
        """Joint angles outside limits should fail."""
        q = np.array([0, 0, 0, 0, 0, 0, 2.0])  # last joint > upper
        lower = -np.ones(7)
        upper = np.ones(7)
        result = safety.check_ik_result(True, q, lower, upper)
        assert not result.safe
        assert "Joint limit" in result.reason

    def test_joint_slightly_outside_within_tolerance_passes(self, safety):
        """Joints slightly outside limits (within 1deg tolerance) should pass."""
        q = np.array([0, 0, 0, 0, 0, 0, 1.01])  # ~0.6deg over
        lower = -np.ones(7)
        upper = np.ones(7)
        result = safety.check_ik_result(True, q, lower, upper)
        assert result.safe


class TestCheckAll:
    def test_all_pass(self, safety):
        """All checks pass -> safe=True."""
        delta = np.eye(4)
        delta[0, 3] = 0.001
        pos = np.array([0.1, 0.1, 0.2])
        q = np.zeros(7)
        lower = -np.ones(7)
        upper = np.ones(7)
        result = safety.check_all(delta, pos, True, q, lower, upper)
        assert result.safe

    def test_first_failure_returned(self, safety):
        """Delta fails first -> that reason is returned."""
        delta = np.eye(4)
        delta[0, 3] = 0.1  # 100mm, will fail
        pos = np.array([10.0, 10.0, 10.0])  # also outside bounds
        q = np.zeros(7)
        lower = -np.ones(7)
        upper = np.ones(7)
        result = safety.check_all(delta, pos, False, q, lower, upper)
        assert not result.safe
        assert "Translation" in result.reason  # delta check fails first
