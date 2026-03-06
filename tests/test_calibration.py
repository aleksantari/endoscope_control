"""Tests for bridge/calibration_loader.py — T_ee_cam loading and transforms."""

import os
import tempfile

import numpy as np
import pytest
import yaml

from bridge.calibration_loader import CalibrationLoader


class TestCalibrationLoaderLoad:
    def test_load_identity(self):
        """Load the placeholder identity T_ee_cam.yaml."""
        loader = CalibrationLoader("config/T_ee_cam.yaml")
        T = loader.get_T_ee_cam()
        np.testing.assert_array_almost_equal(T, np.eye(4))

    def test_nonexistent_file_raises(self):
        with pytest.raises(FileNotFoundError):
            CalibrationLoader("config/does_not_exist.yaml")

    def test_missing_key_raises(self):
        """YAML without 'T_ee_cam' key should raise ValueError."""
        with tempfile.NamedTemporaryFile(mode="w", suffix=".yaml", delete=False) as f:
            yaml.dump({"wrong_key": [[1, 0], [0, 1]]}, f)
            path = f.name
        try:
            with pytest.raises(ValueError, match="Key 'T_ee_cam' not found"):
                CalibrationLoader(path)
        finally:
            os.unlink(path)

    def test_wrong_shape_raises(self):
        """A 3x3 matrix should raise ValueError."""
        with tempfile.NamedTemporaryFile(mode="w", suffix=".yaml", delete=False) as f:
            yaml.dump({"T_ee_cam": [[1, 0, 0], [0, 1, 0], [0, 0, 1]]}, f)
            path = f.name
        try:
            with pytest.raises(ValueError, match="must be 4x4"):
                CalibrationLoader(path)
        finally:
            os.unlink(path)

    def test_bad_last_row_raises(self):
        """Last row not [0,0,0,1] should raise ValueError."""
        bad_matrix = np.eye(4).tolist()
        bad_matrix[3] = [0, 0, 0, 2]
        with tempfile.NamedTemporaryFile(mode="w", suffix=".yaml", delete=False) as f:
            yaml.dump({"T_ee_cam": bad_matrix}, f)
            path = f.name
        try:
            with pytest.raises(ValueError, match="last row"):
                CalibrationLoader(path)
        finally:
            os.unlink(path)

    def test_non_orthogonal_rotation_raises(self):
        """Non-orthogonal rotation part should raise ValueError."""
        bad_matrix = np.eye(4).tolist()
        bad_matrix[0] = [2.0, 0.0, 0.0, 0.0]  # not unit length
        with tempfile.NamedTemporaryFile(mode="w", suffix=".yaml", delete=False) as f:
            yaml.dump({"T_ee_cam": bad_matrix}, f)
            path = f.name
        try:
            with pytest.raises(ValueError, match="not orthogonal"):
                CalibrationLoader(path)
        finally:
            os.unlink(path)


class TestCalibrationTransforms:
    def test_identity_translation_passthrough(self):
        """With identity T_ee_cam, camera delta passes through unchanged."""
        loader = CalibrationLoader("config/T_ee_cam.yaml")
        delta_cam = np.array([1.0, 2.0, 3.0])
        delta_ee = loader.camera_to_ee_translation(delta_cam)
        np.testing.assert_array_almost_equal(delta_ee, delta_cam)

    def test_identity_rotation_passthrough(self):
        """With identity T_ee_cam, rotation axis passes through unchanged."""
        loader = CalibrationLoader("config/T_ee_cam.yaml")
        axis_cam = np.array([0.0, 0.0, 1.0])
        axis_ee, angle = loader.camera_to_ee_rotation(axis_cam, 0.5)
        np.testing.assert_array_almost_equal(axis_ee, axis_cam)
        assert angle == 0.5

    def test_90deg_z_rotation_translation(self):
        """90deg rotation about Z: camera X -> EE Y, camera Y -> EE -X."""
        # Build a 90deg CCW rotation about Z
        T = np.eye(4)
        T[:3, :3] = np.array([
            [0.0, -1.0, 0.0],
            [1.0,  0.0, 0.0],
            [0.0,  0.0, 1.0],
        ])
        with tempfile.NamedTemporaryFile(mode="w", suffix=".yaml", delete=False) as f:
            yaml.dump({"T_ee_cam": T.tolist()}, f)
            path = f.name
        try:
            loader = CalibrationLoader(path)

            # Camera +X should map to EE -Y... wait, R @ [1,0,0]:
            # R = [[0,-1,0],[1,0,0],[0,0,1]], so R @ [1,0,0] = [0,1,0]
            delta_cam_x = np.array([1.0, 0.0, 0.0])
            delta_ee = loader.camera_to_ee_translation(delta_cam_x)
            np.testing.assert_array_almost_equal(delta_ee, [0.0, 1.0, 0.0])

            # Camera +Y should map to EE -X
            delta_cam_y = np.array([0.0, 1.0, 0.0])
            delta_ee = loader.camera_to_ee_translation(delta_cam_y)
            np.testing.assert_array_almost_equal(delta_ee, [-1.0, 0.0, 0.0])
        finally:
            os.unlink(path)

    def test_90deg_z_rotation_axis(self):
        """Rotation axis in camera Z should stay Z after 90deg Z rotation of T_ee_cam."""
        T = np.eye(4)
        T[:3, :3] = np.array([
            [0.0, -1.0, 0.0],
            [1.0,  0.0, 0.0],
            [0.0,  0.0, 1.0],
        ])
        with tempfile.NamedTemporaryFile(mode="w", suffix=".yaml", delete=False) as f:
            yaml.dump({"T_ee_cam": T.tolist()}, f)
            path = f.name
        try:
            loader = CalibrationLoader(path)
            axis_cam = np.array([0.0, 0.0, 1.0])
            axis_ee, angle = loader.camera_to_ee_rotation(axis_cam, 1.0)
            np.testing.assert_array_almost_equal(axis_ee, [0.0, 0.0, 1.0])
            assert angle == 1.0
        finally:
            os.unlink(path)
