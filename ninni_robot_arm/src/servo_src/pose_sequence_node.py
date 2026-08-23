#!/usr/bin/env python3
"""Replay a fixed sequence of joint poses via FollowJointTrajectory."""

import time
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from control_msgs.action import FollowJointTrajectory
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from builtin_interfaces.msg import Duration

JOINT_NAMES = ['Servor_1_to_waist', 'left_1_arm', 'left_2_arm', 'left_3_arm']

# EDIT THESE -- radians, in the order above
POSES = {
    'A': [0.0,  0.0,  0.0,  0.0],
    'B': [0.8, -0.5,  0.6, -0.3],
    'C': [-0.8, 0.4, -0.6,  0.3],
}

SEQUENCE = ['A', 'B', 'A', 'C', 'A']
SECONDS_PER_MOVE = 3.0
LOOP_FOREVER = True
PAUSE_BETWEEN_CYCLES = 1.0


class PoseSequence(Node):
    def __init__(self):
        super().__init__('pose_sequence_node')
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
                self.get_logger().info(f'moving to pose {name}')
                self.send_pose(POSES[name])
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
    rclpy.init()
    node = PoseSequence()
    try:
        node.run()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
