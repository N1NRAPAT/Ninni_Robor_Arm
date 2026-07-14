#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
import time
import math
rclpy.init()
node = Node('servo_test')
pub = node.create_publisher(JointState, '/joint_states', 10)
time.sleep(0.5)
STEP_DEG = 45
TOTAL_DEG = 360 * 3
DELAY = 1.0
angle = 0
step = 0
while angle <= TOTAL_DEG:
    rad = math.radians(angle % 360) - math.pi
    msg = JointState()
    msg.name = ['joint1', 'joint2', 'joint3', 'joint4', 'joint5']
    msg.position = [rad, rad, rad, rad, rad]
    pub.publish(msg)
    step += 1
    print(f'Step {step}: {angle % 360}° ({rad:.2f} rad)')
    angle += STEP_DEG
    time.sleep(DELAY)
print('Done!')
node.destroy_node()
rclpy.shutdown()
