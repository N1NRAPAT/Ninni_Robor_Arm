#!/usr/bin/env python3
"""
servo_bridge_node.py
ROS2 node: subscribes to /joint_states (radians) -> converts to STS3215 ticks -> writes to servo bus.

Pipeline:
  IK solver -> /joint_states -> [this node] -> /dev/ttyACM0 -> Waveshare adapter -> STS3215 servos
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from scservo_sdk import sms_sts, PortHandler
import math


class ServoBridgeNode(Node):
    def __init__(self):
        super().__init__('servo_bridge_node')

        self.declare_parameter('serial_port', '/dev/ttyACM0')
        self.declare_parameter('baudrate', 1000000)
        self.declare_parameter('move_speed', 2000)   # 0-4095ish range, higher = faster
        self.declare_parameter('move_acc', 50)       # 0-255 range, acceleration

        serial_port = self.get_parameter('serial_port').value
        baudrate = self.get_parameter('baudrate').value

        self.joint_to_servo = {
            'joint1': 1,
            'joint2': 2,
            'joint3': 3,
        }

        self.joint_config = {
            'joint1': {'offset_rad': math.pi, 'direction': 1, 'min_tick': 0, 'max_tick': 4095},
            'joint2': {'offset_rad': math.pi, 'direction': 1, 'min_tick': 0, 'max_tick': 4095},
            'joint3': {'offset_rad': math.pi, 'direction': 1, 'min_tick': 0, 'max_tick': 4095},
        }

        self.port_handler = PortHandler(serial_port)
        self.servo = sms_sts(self.port_handler)

        if not self.port_handler.openPort():
            self.get_logger().error(f'Failed to open serial port: {serial_port}')
            raise RuntimeError(f'Cannot open {serial_port}')

        if not self.port_handler.setBaudRate(baudrate):
            self.get_logger().error(f'Failed to set baud rate: {baudrate}')
            raise RuntimeError(f'Cannot set baudrate {baudrate}')

        self.get_logger().info(f'Servo bus opened on {serial_port} @ {baudrate}')

        for joint_name, servo_id in self.joint_to_servo.items():
            _, result, _ = self.servo.ping(servo_id)
            if result == 0:
                self.get_logger().info(f'  OK {joint_name} -> servo ID {servo_id}')
            else:
                self.get_logger().warn(f'  MISSING {joint_name} -> servo ID {servo_id}')

        self.subscription = self.create_subscription(
            JointState,
            '/joint_states',
            self.joint_states_callback,
            10
        )

        self.get_logger().info('Servo bridge ready - listening on /joint_states')

    def rad_to_tick(self, angle_rad, joint_name):
        config = self.joint_config.get(joint_name, {})
        offset = config.get('offset_rad', math.pi)
        direction = config.get('direction', 1)
        min_tick = config.get('min_tick', 0)
        max_tick = config.get('max_tick', 4095)

        adjusted = direction * angle_rad + offset
        tick = int((adjusted / (2.0 * math.pi)) * 4096)
        tick = max(min_tick, min(max_tick, tick))
        return tick

    def joint_states_callback(self, msg):
        # WritePosEx signature is (id, position, speed, acc) - NOT (id, position, time, speed).
        # speed: 0-4095ish range (higher = faster movement)
        # acc: 0-255 range ONLY - this is a single byte, must stay small
        move_speed = int(self.get_parameter('move_speed').value)
        move_acc = int(self.get_parameter('move_acc').value)
        move_acc = max(0, min(255, move_acc))  # hard clamp - this is what overflowed before

        for i, joint_name in enumerate(msg.name):
            if joint_name not in self.joint_to_servo:
                continue
            if i >= len(msg.position):
                continue

            angle_rad = msg.position[i]
            servo_id = self.joint_to_servo[joint_name]
            tick = self.rad_to_tick(angle_rad, joint_name)

            self.get_logger().info(
                f'DEBUG servo_id={servo_id} tick={tick} '
                f'move_speed={move_speed} move_acc={move_acc}'
            )

            self.servo.WritePosEx(servo_id, tick, move_speed, move_acc)
            self.get_logger().debug(
                f'{joint_name} (ID {servo_id}): {angle_rad:.3f} rad -> tick {tick}'
            )

    def destroy_node(self):
        self.get_logger().info('Shutting down - closing serial port')
        self.port_handler.closePort()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = ServoBridgeNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()