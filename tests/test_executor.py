"""Tests for executor/executor.py — command processing pipeline."""

from unittest.mock import MagicMock, patch

import numpy as np
import pytest

from executor.executor import Executor
from executor.modes import ExecutionMode
from executor.safety import SafetyResult
from protocol.command_schema import ActionType, MagnitudeLevel, RobotCommand


def _make_command(action=ActionType.MOVE_UP, magnitude=MagnitudeLevel.SMALL):
    return RobotCommand(
        action=action,
        magnitude=magnitude,
        value_mm=2.0,
        confidence=1.0,
        raw_text="move up small",
    )


def _make_stop():
    return RobotCommand(
        action=ActionType.STOP,
        magnitude=None,
        value_mm=0.0,
        confidence=1.0,
        raw_text="stop",
    )


def _make_executor(controller=None, mode=None):
    exc = Executor("config/robot_config.yaml", controller=controller)
    if mode is not None:
        exc.mode = mode
    return exc


class TestExecutorConstruction:
    def test_construction_from_config(self):
        exc = Executor("config/robot_config.yaml")
        assert exc.bridge is not None
        assert exc.safety is not None
        assert exc.controller is None


class TestStopCommand:
    def test_stop_calls_freeze(self):
        ctrl = MagicMock()
        exc = _make_executor(controller=ctrl)
        result = exc.process_command(_make_stop())
        ctrl.freeze.assert_called_once()
        assert result["executed"] is True
        assert result["action"] == "FREEZE"

    def test_stop_dry_run_no_error(self):
        exc = _make_executor(controller=None)
        result = exc.process_command(_make_stop())
        assert result["executed"] is True
        assert result["action"] == "FREEZE"


class TestDryRun:
    def test_dry_run_move_returns_dry_run(self):
        exc = _make_executor(controller=None, mode=ExecutionMode.AUTOPILOT)
        result = exc.process_command(_make_command())
        assert result["reason"] == "dry_run"
        assert "delta" in result
        assert result["executed"] is False


class TestSafetyRejection:
    def test_safety_rejection_propagates(self):
        exc = _make_executor(controller=None, mode=ExecutionMode.AUTOPILOT)
        exc.safety.check_delta = MagicMock(
            return_value=SafetyResult(safe=False, reason="too large")
        )
        result = exc.process_command(_make_command())
        assert result["executed"] is False
        assert result["reason"] == "too large"


class TestAutopilotMode:
    def test_autopilot_calls_controller(self):
        ctrl = MagicMock()
        ctrl.execute_delta.return_value = {
            "success": True,
            "target_pose": np.eye(4),
            "solved_q": np.zeros(14),
        }
        exc = _make_executor(controller=ctrl, mode=ExecutionMode.AUTOPILOT)
        result = exc.process_command(_make_command())
        ctrl.execute_delta.assert_called_once()
        assert result["executed"] is True


class TestTriggerMode:
    def test_trigger_approved_calls_controller(self):
        ctrl = MagicMock()
        ctrl.execute_delta.return_value = {
            "success": True,
            "target_pose": np.eye(4),
            "solved_q": np.zeros(14),
        }
        exc = _make_executor(controller=ctrl, mode=ExecutionMode.TRIGGER)
        with patch("executor.modes.TriggerPrompt.prompt_operator", return_value=True):
            result = exc.process_command(_make_command())
        ctrl.execute_delta.assert_called_once()
        assert result["executed"] is True

    def test_trigger_rejected_skips_controller(self):
        ctrl = MagicMock()
        exc = _make_executor(controller=ctrl, mode=ExecutionMode.TRIGGER)
        with patch("executor.modes.TriggerPrompt.prompt_operator", return_value=False):
            result = exc.process_command(_make_command())
        ctrl.execute_delta.assert_not_called()
        assert result["executed"] is False
        assert result["reason"] == "operator_rejected"
