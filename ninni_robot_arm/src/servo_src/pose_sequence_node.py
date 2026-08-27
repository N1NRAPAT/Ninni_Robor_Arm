#!/usr/bin/env python3
"""Replay a sequence of joint poses via FollowJointTrajectory.
Angles are entered by the user in DEGREES and converted to radians before use.
Optionally recenters to pose 'A' before every move."""

import time
import math
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from control_msgs.action import FollowJointTrajectory
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from builtin_interfaces.msg import Duration

JOINT_NAMES = ['Servor_1_to_waist', 'left_1_arm', 'left_2_arm', 'left_3_arm']

SEQUENCE = ['B', 'C']          # poses you actually want to visit (A is the recenter pose)
RECENTER_BEFORE_EACH = True    # if True, inserts pose 'A' before every non-A move
SECONDS_PER_MOVE = 6.0         # increase for slower/smoother motion
LOOP_FOREVER = True
PAUSE_BETWEEN_CYCLES = 3.0


def prompt_pose(label):
    """Ask the user for 4 joint angles (degrees) for pose `label`, convert to radians."""
    while True:
        raw = input(
            f"Enter angles for pose {label} "
            f"[{', '.join(JOINT_NAMES)}] (degrees, space-separated): "
        ).strip()
        parts = raw.split()
        if len(parts) != len(JOINT_NAMES):
            print(f"  need {len(JOINT_NAMES)} values, got {len(parts)} -- try again")
            continue
        try:
            degrees = [float(p) for p in parts]
        except ValueError:
            print("  couldn't parse one of those as a number -- try again")
            continue

        radians = [math.radians(d) for d in degrees]
        print(f"  -> radians: {[round(r, 4) for r in radians]}")
        return radians


def prompt_all_poses():
    poses = {}
    for label in ['A', 'B', 'C']:
        poses[label] = prompt_pose(label)
    return poses


class PoseSequence(Node):
    def __init__(self, poses):
        super().__init__('pose_sequence_node')
        self.poses = poses
        self.client = ActionClient(
            self, FollowJointTrajectory,
            '/ninni_arm_controller/follow_joint_trajectory')

    def run(self):
        self.get_logger().info('waiting for action server...')
        self.client.wait_for_server()

        cycle = 0
        while rclpy.ok():
            cycle += 1
            self.get_logger().info(f'--- cycle {cycle} ---')
            for name in SEQUENCE:
                if RECENTER_BEFORE_EACH and name != 'A':
                    self.get_logger().info(f'recentering to A before {name}')
                    self.send_pose(self.poses['A'])
                    if not rclpy.ok():
                        return

                self.get_logger().info(f'moving to pose {name}: {self.poses[name]}')
                self.send_pose(self.poses[name])
                if not rclpy.ok():
                    return

            if not LOOP_FOREVER:
                break
            time.sleep(PAUSE_BETWEEN_CYCLES)

        self.get_logger().info('sequence complete')

    def send_pose(self, positions):
        traj = JointTrajectory()
        traj.joint_names = JOINT_NAMES

        pt = JointTrajectoryPoint()
        pt.positions = [float(p) for p in positions]
        pt.time_from_start = Duration(sec=int(SECONDS_PER_MOVE))
        traj.points = [pt]

        goal = FollowJointTrajectory.Goal()
        goal.trajectory = traj

        future = self.client.send_goal_async(goal)
        rclpy.spin_until_future_complete(self, future)

        handle = future.result()
        if not handle.accepted:
            self.get_logger().error('goal rejected')
            return

        result_future = handle.get_result_async()
        rclpy.spin_until_future_complete(self, result_future)


def main():
    poses = prompt_all_poses()

    rclpy.init()
    node = PoseSequence(poses)
    try:
        node.run()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()