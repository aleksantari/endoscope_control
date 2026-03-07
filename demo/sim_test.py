"""Standalone simulation test — verify FK → IK → DDS with a single delta.

Usage:
    python -m demo.sim_test [--arm {left,right}]

Requires Isaac Lab simulation running with DDS domain_id=1.
"""

import argparse
import logging
import sys
import time

import numpy as np


def main() -> None:
    parser = argparse.ArgumentParser(description="Standalone sim test")
    parser.add_argument("--arm", choices=["left", "right"], default="left")
    args = parser.parse_args()

    # Step 1: Import robot components
    print("[1/7] Importing robot components...")
    try:
        from unitree_sdk2py.core.channel import ChannelFactoryInitialize
        from controller.robot_arm import G1_29_ArmController
        from controller.robot_arm_ik import G1_29_ArmIK
        from controller.fk_utils import FKSolver
        from controller.single_arm_controller import SingleArmController
    except ImportError as e:
        print(f"  FAILED: {e}")
        print("  Make sure you are in the endoscope_control conda env.")
        sys.exit(1)
    print("  OK")

    # Step 2: Initialize DDS
    print("[2/7] Initializing DDS (domain_id=1, simulation)...")
    ChannelFactoryInitialize(1)
    print("  OK")

    # Step 3: Initialize arm controller (connects to DDS, waits for state)
    print("[3/7] Initializing G1_29_ArmController (simulation_mode=True)...")
    print("  Waiting for DDS connection to sim...")
    arm_controller = G1_29_ArmController(simulation_mode=True)
    print("  OK — DDS connected")

    # Step 4: Initialize IK and FK solvers
    print("[4/7] Initializing IK solver (loading URDF + CasADi)...")
    ik_solver = G1_29_ArmIK()
    print("  OK")

    print("  Initializing FK solver (reusing IK model)...")
    fk_solver = FKSolver(ik_solver.reduced_robot.model)
    print("  OK")

    # Step 5: Create single-arm controller
    print(f"[5/7] Creating SingleArmController (arm={args.arm})...")
    controller = SingleArmController(
        arm_controller, ik_solver, fk_solver, active_arm=args.arm,
    )
    print("  OK")

    # Step 6: Read current state and print EE pose
    print("[6/7] Reading current joint state and computing FK...")
    current_q = arm_controller.get_current_dual_arm_q()
    print(f"  Current q (14 joints): {np.round(current_q, 4)}")

    L_pose, R_pose = fk_solver.compute_ee_poses(current_q)
    active_pose = L_pose if args.arm == "left" else R_pose
    print(f"  Current {args.arm} EE pose:")
    for i in range(4):
        print(f"    [{active_pose[i, 0]:+.4f}  {active_pose[i, 1]:+.4f}  "
              f"{active_pose[i, 2]:+.4f}  {active_pose[i, 3]:+.4f}]")

    # Step 7: Apply a small delta and execute
    print("[7/7] Applying test delta: +2mm in Z (forward in EE frame)...")
    delta = np.eye(4)
    delta[2, 3] = -0.05  # +2mm Z

    try:
        result = controller.execute_delta(delta)
        if result["success"]:
            print("  SUCCESS")
            print(f"  Solved q: {np.round(result['solved_q'], 4)}")
            target = result["target_pose"]
            print(f"  Target EE pose:")
            for i in range(4):
                print(f"    [{target[i, 0]:+.4f}  {target[i, 1]:+.4f}  "
                      f"{target[i, 2]:+.4f}  {target[i, 3]:+.4f}]")
        else:
            print(f"  FAILED: {result.get('error', 'unknown')}")
    except Exception as e:
        print(f"  EXCEPTION: {e}")

    # Monitor velocity while holding position
    print("\nHolding new position for 3 seconds — monitoring joint velocity...")
    print(f"  {'Time':>5s}  {'Max |dq| (deg/s)':>16s}  {'dq (deg/s)':s}")
    try:
        for i in range(30):  # 30 samples over 3 seconds
            dq = arm_controller.get_current_dual_arm_dq()
            dq_deg = np.degrees(dq)
            max_dq = np.max(np.abs(dq_deg))
            print(f"  {i*0.1:5.1f}s  {max_dq:16.2f}  {np.array2string(dq_deg, precision=1, suppress_small=True, max_line_width=120)}")
            time.sleep(0.1)
    except KeyboardInterrupt:
        pass
    print("Done. Exiting.")


if __name__ == "__main__":
    logging.basicConfig(
        level=logging.INFO,
        format="%(asctime)s %(name)s — %(message)s",
        datefmt="%H:%M:%S",
    )
    main()
