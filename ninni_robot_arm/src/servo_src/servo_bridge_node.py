#!/usr/bin/env python3


import glob
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from scservo_sdk import sms_sts, PortHandler # STS3215 servo driver
import math


"""AUTO-DETECT SERVO PORT

This fuction is set to auto-detect the serial port that the servo is connected to. 
It scans through /dev/ttyACM* and /dev/ttyUSB* and pings each one to see if a servo responds. 
If a responsive servo is found, it returns the corresponding port. If no responsive servo is 
found, it returns None.

"""

def find_servo_port(baudrate, ping_id=1, logger=None):
    """Scan /dev/ttyACM* and /dev/ttyUSB*, return first port that responds to a ping."""
    candidates = sorted(glob.glob('/dev/ttyACM*') + glob.glob('/dev/ttyUSB*')) # sort for consistent order
    if logger:
        logger.info(f'Auto-detecting servo port, candidates: {candidates}')

    for port in candidates:
        ph = PortHandler(port)
        try:
            if not ph.openPort():
                continue
            if not ph.setBaudRate(baudrate):
                ph.closePort()
                continue
        # If the port cannot be opened or the baud rate cannot be set, skip to the next candidate.
            servo = sms_sts(ph)
            _, result, _ = servo.ping(ping_id)
            ph.closePort()
            if result == 0: # mean the servo responded successfully
                if logger:
                    logger.info(f'  Found responsive servo on {port}')
                return port
        except Exception: # fail safely and continue to the next candidate
            try:
                ph.closePort()
            except Exception:
                pass
            continue

    return None


class ServoBridgeNode(Node):
    def __init__(self):
        super().__init__('servo_bridge_node')

        self.declare_parameter('serial_port', 'auto') 
        self.declare_parameter('baudrate', 1000000)
        self.declare_parameter('move_speed', 500) # speed of servo in range 0 - 4095
        self.declare_parameter('move_acc', 50) # acceleration of servo in range 0 - 255

        requested_port = self.get_parameter('serial_port').value
        baudrate = self.get_parameter('baudrate').value

        if requested_port == 'auto': # true statement
            serial_port = find_servo_port(baudrate, ping_id=1, logger=self.get_logger())
            if serial_port is None:
                self.get_logger().error(
                    'Auto-detect found no responsive servo on any /dev/ttyACM*/ttyUSB* port. '
                    'Check usbipd attach + 12V power, or pass serial_port explicitly.'
                    'Check wire connections and power supply to the servo. Ensure that the servo is powered and connected properly.'
                ) # red signal error message
                raise RuntimeError('No servo port found') 
        else:
            serial_port = requested_port

        self.joint_to_servo = {
            'joint1': 1,
            'joint2': 2,
            'joint3': 3,
        }

        self.joint_config = {
            'joint1': {'offset_rad': math.pi, 'direction': 1, 'min_tick': 0, 'max_tick': 4095},
            'joint2': {'offset_rad': math.pi, 'direction': 1, 'min_tick': 0, 'max_tick': 4095},
            'joint3': {'offset_rad': math.pi, 'direction': 1, 'min_tick': 0, 'max_tick': 4095},
            # initial configuration for each joint, including offset, direction, and tick limits
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

 """ Converts radians

    Uses the joint configuration to apply offset and direction, then scales to the servo's tick range (0-4095).

"""
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
        move_speed = int(self.get_parameter('move_speed').value)
        move_acc = int(self.get_parameter('move_acc').value)
        move_acc = max(0, min(255, move_acc))

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