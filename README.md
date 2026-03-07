# Endoscope Control — Language-to-Motion Pipeline for G1 Humanoid

Stage 3 of the **Language-Based Control for Humanoid Endoscope Assistant** project at the [ARCADE Lab](https://arcade.cs.jhu.edu/), Johns Hopkins University. This package receives voice commands over ZeroMQ from the `language_control` pipeline and translates them into precise end-effector motion on a Unitree G1 29-DOF humanoid robot via inverse kinematics.



## Architecture

```
voice_control (ZMQ PUB) ──→ endoscope_control (ZMQ SUB)
                                    │
                              ActionBridge (command → SE3 delta)
                                    │
                              SafetyModule (bounds check)
                                    │
                              Executor (autopilot/trigger mode)
                                    │
                              SingleArmController
                              (FK → delta → IK → DDS publish)
                                    │
                              G1 Robot (sim or real)
```

The two packages communicate via ZeroMQ PUB/SUB on `tcp://localhost:5556`. The shared contract is the `RobotCommand` JSON schema defined in `protocol/command_schema.py`.

### Pipeline Flow

1. **Voice command** arrives as JSON over ZMQ (e.g., `{"action": "MOVE_UP", "magnitude": "SMALL", ...}`)
2. **ZMQCommandInterface** validates it against the Pydantic `RobotCommand` model — malformed messages are logged and skipped
3. **ActionBridge** maps the action + magnitude to an SE3 delta in camera frame, then transforms to EE frame via `T_ee_cam`
4. **SafetyModule** checks delta magnitude, target workspace bounds, and joint limits
5. **Executor** applies the delta (autopilot) or prompts the operator for confirmation (trigger mode)
6. **SingleArmController** gets current pose via FK, applies the delta, solves IK, and publishes joint targets over DDS
7. If IK fails to converge, the controller falls back to the current position — never sends invalid joints

## Quick Start

### 1. Create the conda environment

```bash
conda create -n endoscope_control python=3.11 pinocchio=3.1.0 -c conda-forge
conda activate endoscope_control
pip install -r requirements.txt
```

### 2. Copy robot arm files from xr_teleoperate

Copy `robot_arm.py` and `robot_arm_ik.py` from [xr_teleoperate](https://github.com/unitreerobotics/xr_teleoperate) into `controller/`:

```bash
cp /path/to/xr_teleoperate/teleop/robot_arm/robot_arm.py controller/
cp /path/to/xr_teleoperate/teleop/robot_arm/robot_arm_ik.py controller/
```

### 3. Run tests

```bash
conda activate endoscope_control
cd ~/endoscope_control
python -m pytest tests/ -v
```

Expected: 82 passed, 1 skipped (DDS test requires robot connection).

### 4. Test without a robot (dry-run mode)

```bash
python -m demo.text_command_demo
```

Type commands like `up small`, `forward big`, `rl mid`, `stop`, or `quit`. You'll see the formatted SE3 delta matrix, translation in mm, and signed rotation in degrees.

## Full Pipeline Usage

### Dry-run with ZMQ (no robot needed)

Test the full ZMQ link without robot hardware:

**Terminal 1** — Voice side (language_control env):

```bash
cd ~/voice_control
conda activate voice_control
python zmq_publisher.py --mode text
```

**Terminal 2** — Robot side (endoscope_control env, dry-run):

```bash
cd ~/endoscope_control
conda activate endoscope_control
python -c "
import logging
logging.basicConfig(level=logging.INFO, format='%(name)s — %(message)s')
from executor.executor import Executor
from executor.modes import ExecutionMode
from interface.zmq_interface import ZMQCommandInterface
e = Executor('config/robot_config.yaml', controller=None)
e.mode = ExecutionMode.AUTOPILOT
iface = ZMQCommandInterface('tcp://localhost:5556', timeout_ms=100)
print('Listening on tcp://localhost:5556 ... (Ctrl+C to stop)')
e.run_loop(iface)
"
```

### Full pipeline with robot (simulation)

**Terminal 1** — Voice side:

```bash
cd ~/language_control
conda activate language_control
python zmq_publisher.py --mode mic
```

**Terminal 2** — Robot side:

```bash
cd ~/endoscope_control
conda activate endoscope_control
python -m demo.run_controller
```

### CLI options for `run_controller`

```
--arm {left,right}           Active arm (default: from config)
--mode {autopilot,trigger}   Execution mode (default: from config)
--real                       Use real robot (sets simulation_mode=false, dds_domain_id=0)
--config PATH                Path to robot_config.yaml
```

Examples:

```bash
python -m demo.run_controller                          # sim, left arm, trigger mode
python -m demo.run_controller --arm right --mode autopilot
python -m demo.run_controller --real                   # real robot
```

## ZMQ Protocol Contract

Both packages must agree on this JSON schema. The subscriber validates with Pydantic — invalid messages are rejected.

```json
{
  "action": "MOVE_UP",           // REQUIRED: one of 9 ActionType values
  "magnitude": "SMALL",          // SMALL / MID / BIG (default: MID)
  "frame": "CAMERA",             // default: CAMERA
  "confidence": 0.95,            // 0.0–1.0 (default: 0.9)
  "value_mm": 2.0,               // magnitude in mm (default: 4.0)
  "raw_text": "move up a little",// original spoken text (default: "")
  "timestamp": null              // optional ISO timestamp
}
```

**Valid actions**: `MOVE_FORWARD`, `RETRACT`, `MOVE_LEFT`, `MOVE_RIGHT`, `MOVE_UP`, `MOVE_DOWN`, `ROTATE_LEFT`, `ROTATE_RIGHT`, `STOP`

All string values are **case-sensitive and uppercase**.

## Configuration

All settings live in `config/robot_config.yaml`:

| Section | Description |
|---------|-------------|
| `robot` | Robot type, active arm, simulation mode, DDS domain ID |
| `control` | Velocity limit, max translation/rotation step per command |
| `safety` | Workspace bounds (meters), heartbeat timeout, IK iterations |
| `execution` | Mode: `autopilot` (immediate) or `trigger` (operator confirms) |
| `zmq` | Subscribe/publish addresses, poll timeout |
| `calibration` | Path to hand-eye transform `T_ee_cam.yaml` |
| `magnitudes` | Translation (mm) and rotation (deg) per magnitude level |

## Command Reference

Camera frame uses **optical convention**: X = right, Y = down, Z = forward.

| Voice Command | Axis | Sign | Type |
|---------------|------|------|------|
| MOVE_FORWARD  | Z    | +    | translation |
| RETRACT       | Z    | -    | translation |
| MOVE_RIGHT    | X    | +    | translation |
| MOVE_LEFT     | X    | -    | translation |
| MOVE_UP       | Y    | -    | translation |
| MOVE_DOWN     | Y    | +    | translation |
| ROTATE_LEFT   | Z    | +    | rotation |
| ROTATE_RIGHT  | Z    | -    | rotation |
| STOP          | --   | --   | freeze |

Note: "move up" maps to **negative Y** because the camera frame uses optical convention (Y points down).

### Magnitude Levels

| Level | Translation | Rotation |
|-------|-------------|----------|
| SMALL | 2 mm        | 1 deg    |
| MID   | 4 mm        | 2 deg    |
| BIG   | 6 mm        | 3 deg    |

## Repository Structure

```
endoscope_control/
├── config/
│   ├── robot_config.yaml       # all configuration
│   └── T_ee_cam.yaml           # hand-eye calibration transform
├── protocol/
│   └── command_schema.py       # RobotCommand Pydantic model
├── bridge/
│   ├── action_bridge.py        # RobotCommand → SE3 delta in EE frame
│   └── calibration_loader.py   # loads T_ee_cam from YAML
├── controller/
│   ├── single_arm_controller.py  # wraps dual-arm for single-arm use
│   ├── fk_utils.py             # Pinocchio FK helper
│   ├── robot_arm.py            # from xr_teleoperate (DO NOT MODIFY)
│   └── robot_arm_ik.py         # from xr_teleoperate (DO NOT MODIFY)
├── executor/
│   ├── executor.py             # main orchestrator
│   ├── safety.py               # workspace bounds, limits, heartbeat
│   └── modes.py                # ExecutionMode enum, TriggerPrompt
├── interface/
│   ├── command_interface.py    # abstract base class
│   ├── zmq_interface.py        # ZMQ subscriber
│   └── text_interface.py       # typed commands for testing
├── demo/
│   ├── run_controller.py       # full pipeline entry point
│   ├── text_command_demo.py    # dry-run REPL (no robot needed)
│   └── zmq_echo_test.py        # ZMQ round-trip test
├── scripts/
│   └── zmq_publisher.py        # voice side publisher (copy to language_control)
└── tests/                      # 82 pytest tests
```

## VLA Integration

The `CommandInterface` abstract base class in `interface/command_interface.py` makes the command source swappable. To integrate a Vision-Language-Action model:

```python
from interface.command_interface import CommandInterface
from protocol.command_schema import RobotCommand

class VLAInterface(CommandInterface):
    def __init__(self, model, camera_feed):
        self.model = model
        self.camera = camera_feed

    def get_next_command(self):
        frame = self.camera.grab()
        prediction = self.model.predict(frame)
        return RobotCommand(
            action=prediction.action,
            magnitude=prediction.magnitude,
            confidence=prediction.confidence,
            raw_text="vla_prediction",
        )

    def close(self):
        self.camera.release()
```

Then pass the VLA interface to the executor:

```python
executor = Executor(config_path, controller=single_arm)
executor.run_loop(VLAInterface(model, camera))
```

## Related Repositories

- [xr_teleoperate](https://github.com/unitreerobotics/xr_teleoperate) — source of `robot_arm.py` and `robot_arm_ik.py`
- [unitree_sim_isaaclab](https://github.com/unitreerobotics/unitree_sim_isaaclab) — Isaac Lab simulation bridge
- [unitree_sdk2](https://github.com/unitreerobotics/unitree_sdk2) — DDS communication layer
- [G1_hand_eye_calibration_sinus](https://github.com/HanZhang206/G1_hand_eye_calibration_sinus) — Stage 2 hand-eye calibration

## License

Internal project — ARCADE Lab, Johns Hopkins University.
