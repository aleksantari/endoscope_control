"""RobotCommand Pydantic model — shared protocol between voice_control and endoscope_control."""

import json
from enum import Enum
from typing import Optional

from pydantic import BaseModel, Field


class ActionType(str, Enum):
    """All supported endoscope actions."""

    MOVE_FORWARD = "MOVE_FORWARD"
    RETRACT = "RETRACT"
    MOVE_LEFT = "MOVE_LEFT"
    MOVE_RIGHT = "MOVE_RIGHT"
    MOVE_UP = "MOVE_UP"
    MOVE_DOWN = "MOVE_DOWN"
    ROTATE_LEFT = "ROTATE_LEFT"
    ROTATE_RIGHT = "ROTATE_RIGHT"
    STOP = "STOP"


class MagnitudeLevel(str, Enum):
    """Motion magnitude levels."""

    SMALL = "SMALL"
    MID = "MID"
    BIG = "BIG"


class RobotCommand(BaseModel):
    """Command sent from voice_control to endoscope_control over ZMQ."""

    action: ActionType
    magnitude: Optional[MagnitudeLevel] = MagnitudeLevel.MID
    frame: str = "CAMERA"
    confidence: float = Field(ge=0.0, le=1.0, default=0.9)
    value_mm: float = Field(default=4.0)
    raw_text: str = ""
    timestamp: Optional[str] = None

    @property
    def is_translation(self) -> bool:
        """True if this is a translational movement command."""
        return self.action in {
            ActionType.MOVE_FORWARD,
            ActionType.RETRACT,
            ActionType.MOVE_LEFT,
            ActionType.MOVE_RIGHT,
            ActionType.MOVE_UP,
            ActionType.MOVE_DOWN,
        }

    @property
    def is_rotation(self) -> bool:
        """True if this is a rotational movement command."""
        return self.action in {ActionType.ROTATE_LEFT, ActionType.ROTATE_RIGHT}

    @property
    def is_stop(self) -> bool:
        """True if this is a STOP/FREEZE command."""
        return self.action == ActionType.STOP

    @classmethod
    def from_json_string(cls, json_str: str) -> "RobotCommand":
        """Deserialize from a JSON string (as received over ZMQ)."""
        data = json.loads(json_str)
        return cls(**data)

    def to_json_string(self) -> str:
        """Serialize to a JSON string (for ZMQ transport)."""
        return self.model_dump_json()
