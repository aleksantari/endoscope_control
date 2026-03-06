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

### 3. Test without a robot (dry-run mode)

```bash
python -m demo.text_command_demo
```

Type commands like `up small`, `forward big`, `stop`, or `quit`.

## Full Pipeline Usage

**Terminal 1** — Voice side (language_control env):

```bash
cd ~/language_control
conda activate language_control
python zmq_publisher.py --mode text
```

**Terminal 2** — Robot side (endoscope_control env):

```bash
cd ~/endoscope_control
conda activate endoscope_control
python -m demo.run_controller
```

### CLI options for `run_controller`

```
--arm {left,right}           Active arm (default: from config)
--mode {autopilot,trigger}   Execution mode (default: from config)
--real                       Use real robot (default: simulation)
--config PATH                Path to robot_config.yaml
```

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
│   └── zmq_publisher.py        # voice side publisher
└── tests/                      # pytest test suite
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
