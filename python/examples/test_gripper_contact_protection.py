import argparse
import os
import sys
import time

import numpy as np


ROOT_DIR = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, ROOT_DIR)
os.chdir(ROOT_DIR)

import arx5_interface as arx5


def make_hold_cmd(arx5_module, dof, arm_pos, gripper_pos):
    cmd = arx5_module.JointState(dof)
    cmd.pos()[:] = arm_pos
    cmd.gripper_pos = float(gripper_pos)
    cmd.gripper_vel = 0.0
    cmd.gripper_torque = 0.0
    return cmd


def main():
    parser = argparse.ArgumentParser(
        description="Slowly close the ARX gripper with SDK contact protection enabled."
    )
    parser.add_argument("model", help="ARX arm model, e.g. X5 or L5")
    parser.add_argument("interface", help="CAN interface name, e.g. can0")
    parser.add_argument("--rate", type=float, default=50.0, help="Policy command rate in Hz")
    parser.add_argument("--close-speed", type=float, default=0.004, help="Target closing speed in m/s")
    parser.add_argument("--hold-time", type=float, default=5.0, help="Seconds to hold after target reaches 0")
    parser.add_argument("--max-time", type=float, default=30.0, help="Hard timeout in seconds")
    parser.add_argument(
        "--contact-ratio",
        type=float,
        default=0.35,
        help="Contact threshold as a ratio of gripper_torque_max if --contact-threshold is unset",
    )
    parser.add_argument(
        "--contact-threshold",
        type=float,
        default=None,
        help="Absolute contact torque threshold in SDK torque units",
    )
    parser.add_argument("--unload-margin", type=float, default=0.0005, help="Unload margin in meters")
    parser.add_argument("--kp-scale", type=float, default=0.2, help="KP scale in the protected band")
    parser.add_argument("--filter-alpha", type=float, default=0.2, help="Torque filter alpha")
    parser.add_argument(
        "--gripper-torque-max",
        type=float,
        default=None,
        help="Override emergency torque threshold; leave unset for SDK default",
    )
    parser.add_argument(
        "--gripper-open-readout",
        type=float,
        default=-3.2,
        help="Fully-open gripper motor readout used by this robot",
    )
    parser.add_argument("--no-confirm", action="store_true", help="Skip interactive safety prompt")
    args = parser.parse_args()

    if not args.no_confirm:
        print("This test will enable gripper position control and slowly close the gripper.")
        print("Keep a human at the robot, keep e-stop ready, and place the test object only when safe.")
        input("Press Enter to continue, or Ctrl-C to abort.")

    robot_config = arx5.RobotConfigFactory.get_instance().get_config(args.model)
    robot_config.gripper_open_readout = float(args.gripper_open_readout)
    if args.gripper_torque_max is not None:
        robot_config.gripper_torque_max = float(args.gripper_torque_max)
    robot_config.gripper_contact_protection = True
    if args.contact_threshold is None:
        robot_config.gripper_contact_torque_threshold = (
            float(args.contact_ratio) * robot_config.gripper_torque_max
        )
    else:
        robot_config.gripper_contact_torque_threshold = float(args.contact_threshold)
    robot_config.gripper_contact_unload_margin = float(args.unload_margin)
    robot_config.gripper_contact_kp_scale = float(args.kp_scale)
    robot_config.gripper_contact_torque_filter_alpha = float(args.filter_alpha)

    controller_config = arx5.ControllerConfigFactory.get_instance().get_config(
        "joint_controller", robot_config.joint_dof
    )
    controller_config.background_send_recv = True

    controller = None
    try:
        controller = arx5.Arx5JointController(robot_config, controller_config, args.interface)
        state = controller.get_joint_state()
        arm_hold_pos = state.pos().copy()
        target_gripper_pos = float(np.clip(state.gripper_pos, 0.0, robot_config.gripper_width))

        controller.set_joint_cmd(
            make_hold_cmd(arx5, robot_config.joint_dof, arm_hold_pos, target_gripper_pos)
        )
        gain = arx5.Gain(robot_config.joint_dof)
        gain.kp()[:] = controller_config.default_kp
        gain.kd()[:] = controller_config.default_kd
        gain.gripper_kp = controller_config.default_gripper_kp
        gain.gripper_kd = controller_config.default_gripper_kd
        controller.set_gain(gain)

        dt = 1.0 / args.rate
        start_time = time.monotonic()
        hold_start = None
        print(
            "t,policy_gripper_pos,actual_gripper_pos,actual_gripper_vel,"
            "gripper_torque,contact_threshold,emergency_threshold"
        )
        while time.monotonic() - start_time < args.max_time:
            loop_start = time.monotonic()
            if target_gripper_pos > 0.0:
                target_gripper_pos = max(0.0, target_gripper_pos - args.close_speed * dt)
            elif hold_start is None:
                hold_start = loop_start
            elif loop_start - hold_start >= args.hold_time:
                break

            controller.set_joint_cmd(
                make_hold_cmd(arx5, robot_config.joint_dof, arm_hold_pos, target_gripper_pos)
            )
            state = controller.get_joint_state()
            print(
                f"{loop_start - start_time:.3f},"
                f"{target_gripper_pos:.6f},"
                f"{state.gripper_pos:.6f},"
                f"{state.gripper_vel:.6f},"
                f"{state.gripper_torque:.6f},"
                f"{robot_config.gripper_contact_torque_threshold:.6f},"
                f"{robot_config.gripper_torque_max:.6f}",
                flush=True,
            )

            elapsed = time.monotonic() - loop_start
            if elapsed < dt:
                time.sleep(dt - elapsed)
    finally:
        if controller is not None:
            controller.set_to_damping()


if __name__ == "__main__":
    main()
