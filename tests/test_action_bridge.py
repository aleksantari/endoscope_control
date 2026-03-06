"""Tests for bridge/action_bridge.py — RobotCommand to SE3 delta conversion."""

import os
import tempfile

import numpy as np
import pytest
import yaml

from bridge.action_bridge import ActionBridge
from bridge.calibration_loader import CalibrationLoader
from protocol.command_schema import ActionType, MagnitudeLevel, RobotCommand


@pytest.fixture
def identity_bridge():
    """ActionBridge with identity calibration (camera == EE frame)."""
    cal = CalibrationLoader("config/T_ee_cam.yaml")
    config = yaml.safe_load(open("config/robot_config.yaml"))
    return ActionBridge(cal, config)


@pytest.fixture
def rotated_bridge():
    """ActionBridge with 90deg Z rotation calibration."""
    T = np.eye(4)
    T[:3, :3] = np.array([
        [0.0, -1.0, 0.0],
        [1.0,  0.0, 0.0],
        [0.0,  0.0, 1.0],
    ])
    with tempfile.NamedTemporaryFile(mode="w", suffix=".yaml", delete=False) as f:
        yaml.dump({"T_ee_cam": T.tolist()}, f)
        path = f.name
    cal = CalibrationLoader(path)
    config = yaml.safe_load(open("config/robot_config.yaml"))
    bridge = ActionBridge(cal, config)
    yield bridge
    os.unlink(path)


class TestTranslations:
    def test_move_up_small(self, identity_bridge):
        """MOVE_UP SMALL -> -Y direction, 2mm = 0.002m."""
        cmd = RobotCommand(action=ActionType.MOVE_UP, magnitude=MagnitudeLevel.SMALL)
        delta = identity_bridge.command_to_delta(cmd)
        t = delta[:3, 3]
        np.testing.assert_allclose(t, [0.0, -0.002, 0.0], atol=1e-9)
        # Rotation part should be identity
        np.testing.assert_allclose(delta[:3, :3], np.eye(3), atol=1e-9)

    def test_move_down_mid(self, identity_bridge):
        """MOVE_DOWN MID -> +Y direction, 4mm = 0.004m."""
        cmd = RobotCommand(action=ActionType.MOVE_DOWN, magnitude=MagnitudeLevel.MID)
        delta = identity_bridge.command_to_delta(cmd)
        t = delta[:3, 3]
        np.testing.assert_allclose(t, [0.0, 0.004, 0.0], atol=1e-9)

    def test_move_forward_mid(self, identity_bridge):
        """MOVE_FORWARD MID -> +Z direction, 4mm = 0.004m."""
        cmd = RobotCommand(action=ActionType.MOVE_FORWARD, magnitude=MagnitudeLevel.MID)
        delta = identity_bridge.command_to_delta(cmd)
        t = delta[:3, 3]
        np.testing.assert_allclose(t, [0.0, 0.0, 0.004], atol=1e-9)

    def test_retract_big(self, identity_bridge):
        """RETRACT BIG -> -Z direction, 6mm = 0.006m."""
        cmd = RobotCommand(action=ActionType.RETRACT, magnitude=MagnitudeLevel.BIG)
        delta = identity_bridge.command_to_delta(cmd)
        t = delta[:3, 3]
        np.testing.assert_allclose(t, [0.0, 0.0, -0.006], atol=1e-9)

    def test_move_right_small(self, identity_bridge):
        """MOVE_RIGHT SMALL -> +X direction, 2mm = 0.002m."""
        cmd = RobotCommand(action=ActionType.MOVE_RIGHT, magnitude=MagnitudeLevel.SMALL)
        delta = identity_bridge.command_to_delta(cmd)
        t = delta[:3, 3]
        np.testing.assert_allclose(t, [0.002, 0.0, 0.0], atol=1e-9)

    def test_move_left_mid(self, identity_bridge):
        """MOVE_LEFT MID -> -X direction, 4mm = 0.004m."""
        cmd = RobotCommand(action=ActionType.MOVE_LEFT, magnitude=MagnitudeLevel.MID)
        delta = identity_bridge.command_to_delta(cmd)
        t = delta[:3, 3]
        np.testing.assert_allclose(t, [-0.004, 0.0, 0.0], atol=1e-9)


class TestRotations:
    def test_rotate_left_big(self, identity_bridge):
        """ROTATE_LEFT BIG -> rotation about +Z, 3 degrees."""
        cmd = RobotCommand(action=ActionType.ROTATE_LEFT, magnitude=MagnitudeLevel.BIG)
        delta = identity_bridge.command_to_delta(cmd)
        # Translation should be zero
        np.testing.assert_allclose(delta[:3, 3], [0.0, 0.0, 0.0], atol=1e-9)
        # Rotation should be ~3 degrees about Z
        angle_rad = np.deg2rad(3.0)
        expected_R = np.array([
            [np.cos(angle_rad), -np.sin(angle_rad), 0.0],
            [np.sin(angle_rad),  np.cos(angle_rad), 0.0],
            [0.0, 0.0, 1.0],
        ])
        np.testing.assert_allclose(delta[:3, :3], expected_R, atol=1e-9)

    def test_rotate_right_small(self, identity_bridge):
        """ROTATE_RIGHT SMALL -> rotation about -Z, 1 degree."""
        cmd = RobotCommand(action=ActionType.ROTATE_RIGHT, magnitude=MagnitudeLevel.SMALL)
        delta = identity_bridge.command_to_delta(cmd)
        angle_rad = np.deg2rad(-1.0)
        expected_R = np.array([
            [np.cos(angle_rad), -np.sin(angle_rad), 0.0],
            [np.sin(angle_rad),  np.cos(angle_rad), 0.0],
            [0.0, 0.0, 1.0],
        ])
        np.testing.assert_allclose(delta[:3, :3], expected_R, atol=1e-9)


class TestStop:
    def test_stop_returns_identity(self, identity_bridge):
        """STOP -> identity matrix (no motion)."""
        cmd = RobotCommand(action=ActionType.STOP)
        delta = identity_bridge.command_to_delta(cmd)
        np.testing.assert_allclose(delta, np.eye(4), atol=1e-9)


class TestAllActions:
    def test_all_actions_produce_valid_se3(self, identity_bridge):
        """Every action should produce a valid 4x4 SE3 matrix."""
        for action in ActionType:
            cmd = RobotCommand(action=action)
            delta = identity_bridge.command_to_delta(cmd)
            assert delta.shape == (4, 4)
            np.testing.assert_allclose(delta[3, :], [0, 0, 0, 1], atol=1e-9)

    def test_non_stop_actions_produce_nonzero_delta(self, identity_bridge):
        """All actions except STOP should produce a non-identity delta."""
        for action in ActionType:
            if action == ActionType.STOP:
                continue
            cmd = RobotCommand(action=action)
            delta = identity_bridge.command_to_delta(cmd)
            assert not np.allclose(delta, np.eye(4)), f"{action} produced identity"


class TestNonIdentityCalibration:
    def test_move_right_with_90deg_z_rotation(self, rotated_bridge):
        """MOVE_RIGHT with 90deg Z rotation in T_ee_cam.

        Camera +X -> R @ [1,0,0] = [0,1,0] in EE frame.
        """
        cmd = RobotCommand(action=ActionType.MOVE_RIGHT, magnitude=MagnitudeLevel.MID)
        delta = rotated_bridge.command_to_delta(cmd)
        t = delta[:3, 3]
        np.testing.assert_allclose(t, [0.0, 0.004, 0.0], atol=1e-9)

    def test_move_forward_with_90deg_z_rotation(self, rotated_bridge):
        """MOVE_FORWARD with 90deg Z rotation.

        Camera +Z stays +Z in EE frame (rotation is about Z).
        """
        cmd = RobotCommand(action=ActionType.MOVE_FORWARD, magnitude=MagnitudeLevel.SMALL)
        delta = rotated_bridge.command_to_delta(cmd)
        t = delta[:3, 3]
        np.testing.assert_allclose(t, [0.0, 0.0, 0.002], atol=1e-9)
