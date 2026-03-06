"""Full endoscope control pipeline — ZMQ subscriber + robot execution.

Usage:
    python -m demo.run_controller [--arm {left,right}] [--mode {autopilot,trigger}]
                                  [--real] [--config PATH]

Examples:
    python -m demo.run_controller                        # sim, left arm, trigger mode
    python -m demo.run_controller --arm right --mode autopilot
    python -m demo.run_controller --real                 # real robot (domain_id=0)
"""

import argparse
import logging
import sys

import yaml

logger = logging.getLogger(__name__)


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Endoscope control pipeline — ZMQ subscriber + robot execution.",
    )
    parser.add_argument(
        "--arm", choices=["left", "right"], default=None,
        help="Active arm (default: from config)",
    )
    parser.add_argument(
        "--mode", choices=["autopilot", "trigger"], default=None,
        help="Execution mode (default: from config)",
    )
    parser.add_argument(
        "--real", action="store_true",
        help="Use real robot (default: simulation)",
    )
    parser.add_argument(
        "--config", default="config/robot_config.yaml",
        help="Path to robot_config.yaml",
    )
    return parser.parse_args()


def main() -> None:
    args = parse_args()

    # Import robot components — these require DDS, pinocchio, casadi
    try:
        from controller.robot_arm import G1_29_ArmController
        from controller.robot_arm_ik import G1_29_ArmIK
    except ImportError as e:
        print(f"ERROR: Could not import robot components: {e}")
        print("This demo requires unitree_sdk2py, pinocchio, and casadi.")
        print("Make sure you are in the correct conda environment.")
        sys.exit(1)

    from controller.fk_utils import FKSolver
    from controller.single_arm_controller import SingleArmController
    from executor.executor import Executor
    from executor.modes import ExecutionMode
    from interface.zmq_interface import ZMQCommandInterface

    # Load and override config
    with open(args.config) as f:
        config = yaml.safe_load(f)

    if args.arm is not None:
        config["robot"]["active_arm"] = args.arm
    if args.mode is not None:
        config["execution"]["mode"] = args.mode
    if args.real:
        config["robot"]["simulation_mode"] = False
        config["robot"]["dds_domain_id"] = 0

    active_arm = config["robot"]["active_arm"]
    sim_mode = config["robot"]["simulation_mode"]
    exec_mode = config["execution"]["mode"]
    zmq_address = config["zmq"]["subscribe_address"]

    print("╔══════════════════════════════════════════╗")
    print("║  Endoscope Control — Full Pipeline       ║")
    print(f"║  Mode: {exec_mode:<8} | Arm: {active_arm:<5} | Sim: {str(sim_mode):<5} ║")
    print("╚══════════════════════════════════════════╝")
    print(f"ZMQ subscribing on: {zmq_address}")
    print()

    # Initialize controller stack
    arm_controller = G1_29_ArmController(
        simulation=sim_mode,
        domain_id=config["robot"]["dds_domain_id"],
    )
    ik_solver = G1_29_ArmIK()
    fk_solver = FKSolver(ik_solver.model)
    controller = SingleArmController(
        arm_controller, ik_solver, fk_solver, active_arm=active_arm,
    )

    # Initialize executor and interface
    executor = Executor(args.config, controller=controller)
    if args.mode is not None:
        executor.mode = ExecutionMode(args.mode)

    interface = ZMQCommandInterface(
        address=zmq_address,
        timeout_ms=config["zmq"]["timeout_ms"],
    )

    print("Waiting for commands...")
    executor.run_loop(interface)


if __name__ == "__main__":
    logging.basicConfig(
        level=logging.INFO,
        format="%(asctime)s %(name)s — %(message)s",
        datefmt="%H:%M:%S",
    )
    main()
