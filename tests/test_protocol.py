"""Tests for protocol/command_schema.py — RobotCommand model."""

import json

import pytest

from protocol.command_schema import ActionType, MagnitudeLevel, RobotCommand


class TestActionType:
    def test_all_nine_actions_present(self):
        expected = {
            "MOVE_FORWARD", "RETRACT", "MOVE_LEFT", "MOVE_RIGHT",
            "MOVE_UP", "MOVE_DOWN", "ROTATE_LEFT", "ROTATE_RIGHT", "STOP",
        }
        assert {a.value for a in ActionType} == expected

    def test_action_count(self):
        assert len(ActionType) == 9


class TestMagnitudeLevel:
    def test_all_magnitudes_present(self):
        assert {m.value for m in MagnitudeLevel} == {"SMALL", "MID", "BIG"}


class TestRobotCommand:
    def test_create_with_all_fields(self):
        cmd = RobotCommand(
            action=ActionType.MOVE_UP,
            magnitude=MagnitudeLevel.SMALL,
            frame="CAMERA",
            confidence=0.95,
            value_mm=2.0,
            raw_text="move the camera up a little",
            timestamp="2026-03-02T18:30:00.000Z",
        )
        assert cmd.action == ActionType.MOVE_UP
        assert cmd.magnitude == MagnitudeLevel.SMALL
        assert cmd.frame == "CAMERA"
        assert cmd.confidence == 0.95
        assert cmd.value_mm == 2.0
        assert cmd.raw_text == "move the camera up a little"
        assert cmd.timestamp == "2026-03-02T18:30:00.000Z"

    def test_defaults(self):
        cmd = RobotCommand(action=ActionType.STOP)
        assert cmd.magnitude == MagnitudeLevel.MID
        assert cmd.frame == "CAMERA"
        assert cmd.confidence == 0.9
        assert cmd.value_mm == 4.0
        assert cmd.raw_text == ""
        assert cmd.timestamp is None

    def test_is_translation(self):
        translation_actions = [
            ActionType.MOVE_FORWARD, ActionType.RETRACT,
            ActionType.MOVE_LEFT, ActionType.MOVE_RIGHT,
            ActionType.MOVE_UP, ActionType.MOVE_DOWN,
        ]
        for action in translation_actions:
            cmd = RobotCommand(action=action)
            assert cmd.is_translation, f"{action} should be translation"
            assert not cmd.is_rotation
            assert not cmd.is_stop

    def test_is_rotation(self):
        for action in [ActionType.ROTATE_LEFT, ActionType.ROTATE_RIGHT]:
            cmd = RobotCommand(action=action)
            assert cmd.is_rotation, f"{action} should be rotation"
            assert not cmd.is_translation
            assert not cmd.is_stop

    def test_is_stop(self):
        cmd = RobotCommand(action=ActionType.STOP)
        assert cmd.is_stop
        assert not cmd.is_translation
        assert not cmd.is_rotation

    def test_from_json_string_round_trip(self):
        original = RobotCommand(
            action=ActionType.MOVE_RIGHT,
            magnitude=MagnitudeLevel.BIG,
            confidence=0.85,
            value_mm=6.0,
            raw_text="move right a lot",
        )
        json_str = original.to_json_string()
        restored = RobotCommand.from_json_string(json_str)
        assert restored.action == original.action
        assert restored.magnitude == original.magnitude
        assert restored.confidence == original.confidence
        assert restored.value_mm == original.value_mm
        assert restored.raw_text == original.raw_text

    def test_deserialize_voice_control_json(self):
        """Test that the exact JSON format from voice_control deserializes correctly."""
        voice_json = json.dumps({
            "action": "MOVE_UP",
            "magnitude": "SMALL",
            "frame": "CAMERA",
            "confidence": 0.95,
            "value_mm": 2.0,
            "raw_text": "move the camera up a little",
        })
        cmd = RobotCommand.from_json_string(voice_json)
        assert cmd.action == ActionType.MOVE_UP
        assert cmd.magnitude == MagnitudeLevel.SMALL
        assert cmd.confidence == 0.95
        assert cmd.value_mm == 2.0

    def test_confidence_bounds(self):
        with pytest.raises(Exception):
            RobotCommand(action=ActionType.STOP, confidence=1.5)
        with pytest.raises(Exception):
            RobotCommand(action=ActionType.STOP, confidence=-0.1)

    def test_every_action_classifies_exactly_once(self):
        """Each action should be exactly one of: translation, rotation, or stop."""
        for action in ActionType:
            cmd = RobotCommand(action=action)
            categories = [cmd.is_translation, cmd.is_rotation, cmd.is_stop]
            assert sum(categories) == 1, f"{action} classified into {sum(categories)} categories"
