#!/usr/bin/env python3
"""
gesture_control_node.py
Subscribes to /detected_class (std_msgs/String) from yolo_detect_node.
Drives all 3 servos in continuous wheel (velocity) mode:
  - "person" detected  -> spin clockwise
  - "bottle" detected  -> spin counter-clockwise
  - nothing detected   -> stop
Type 'q' + Enter in this terminal to stop all servos, restore position
mode, and exit cleanly.

Auto-detects the correct /dev/ttyACM* port by trying each candidate and
pinging servo ID 1 - no more manually figuring out ACM0 vs ACM1 each session.
Set serial_port param to a specific path (e.g. /dev/ttyACM1) to skip auto-detect.

NOTE: wheel-mode register addresses below follow the standard Waveshare
ST3215 control table (MODE=33, TORQUE_ENABLE=40, ACC=41, GOAL_SPEED=46-47,
direction = sign+magnitude with bit15 as the direction flag). Test at low
spin_speed first - if direction is flipped or nothing moves, this is the
first place to check against your SDK's actual register map.
"""

import glob
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from scservo_sdk import sms_sts, PortHandler
import threading
import sys

ADDR_TORQUE_ENABLE = 40
ADDR_ACC = 41
ADDR_MODE = 33
ADDR_GOAL_SPEED = 46

WHEEL_MODE = 1
POSITION_MODE = 0
DIRECTION_BIT = 0x8000


def find_servo_port(baudrate, ping_id=1, logger=None):
    """Scan /dev/ttyACM* and /dev/ttyUSB*, return first port that responds to a ping."""
    candidates = sorted(glob.glob('/dev/ttyACM*') + glob.glob('/dev/ttyUSB*'))
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
            servo = sms_sts(ph)
            _, result, _ = servo.ping(ping_id)
            ph.closePort()
            if result == 0:
                if logger:
                    logger.info(f'  Found responsive servo on {port}')
                return port
        except Exception:
            try:
                ph.closePort()
            except Exception:
                pass
            continue

    return None


class GestureControlNode(Node):
    def __init__(self):
        super().__init__('gesture_control_node')

        self.declare_parameter('serial_port', 'auto')
        self.declare_parameter('baudrate', 1000000)
        self.declare_parameter('spin_speed', 500)
        self.declare_parameter('spin_acc', 50)

        requested_port = self.get_parameter('serial_port').value
        baudrate = self.get_parameter('baudrate').value
        self.spin_speed = int(self.get_parameter('spin_speed').value)
        self.spin_acc = max(0, min(255, int(self.get_parameter('spin_acc').value)))

        if requested_port == 'auto':
            serial_port = find_servo_port(baudrate, ping_id=1, logger=self.get_logger())
            if serial_port is None:
                self.get_logger().error(
                    'Auto-detect found no responsive servo on any /dev/ttyACM*/ttyUSB* port. '
                    'Check usbipd attach + 12V power, or pass serial_port explicitly.'
                )
                raise RuntimeError('No servo port found')
        else:
            serial_port = requested_port

        self.servo_ids = [1, 2, 3]

        self.port_handler = PortHandler(serial_port)
        self.servo = sms_sts(self.port_handler)

        if not self.port_handler.openPort():
            self.get_logger().error(f'Failed to open serial port: {serial_port}')
            raise RuntimeError(f'Cannot open {serial_port}')
        if not self.port_handler.setBaudRate(baudrate):
            self.get_logger().error(f'Failed to set baud rate: {baudrate}')
            raise RuntimeError(f'Cannot set baudrate {baudrate}')

        self.get_logger().info(f'Servo bus opened on {serial_port} @ {baudrate}')

        self._enter_wheel_mode()
        self.current_state = None

        self.subscription = self.create_subscription(
            String, '/detected_class', self.class_callback, 10)

        self.last_msg_time = self.get_clock().now()
        self.watchdog_timer = self.create_timer(0.5, self._watchdog_check)

        self.stop_flag = threading.Event()
        self._key_thread = threading.Thread(target=self._keyboard_listener, daemon=True)
        self._key_thread.start()

        self.get_logger().info(
            "Gesture control ready. person=CW, bottle=CCW. Type 'q' + Enter to stop."
        )

    def _enter_wheel_mode(self):
        for sid in self.servo_ids:
            self.servo.write1ByteTxRx(sid, ADDR_TORQUE_ENABLE, 0)
            self.servo.write1ByteTxRx(sid, ADDR_MODE, WHEEL_MODE)
            self.servo.write1ByteTxRx(sid, ADDR_ACC, self.spin_acc)
            self.servo.write1ByteTxRx(sid, ADDR_TORQUE_ENABLE, 1)
        self.get_logger().info('Servos switched to wheel mode')

    def _exit_wheel_mode(self):
        for sid in self.servo_ids:
            self._write_wheel_speed(sid, 0)
            self.servo.write1ByteTxRx(sid, ADDR_TORQUE_ENABLE, 0)
            self.servo.write1ByteTxRx(sid, ADDR_MODE, POSITION_MODE)
        self.get_logger().info('Servos returned to position mode')

    def _write_wheel_speed(self, servo_id, speed_signed):
        magnitude = min(abs(int(speed_signed)), 3400)
        raw = magnitude | DIRECTION_BIT if speed_signed < 0 else magnitude
        self.servo.write2ByteTxRx(servo_id, ADDR_GOAL_SPEED, raw)

    def _set_all(self, speed_signed):
        for sid in self.servo_ids:
            self._write_wheel_speed(sid, speed_signed)

    def class_callback(self, msg):
        self.last_msg_time = self.get_clock().now()
        label = msg.data.strip().lower()

        if label == 'person' and self.current_state != 'cw':
            self.get_logger().info('Person detected -> spinning CW')
            self._set_all(self.spin_speed)
            self.current_state = 'cw'
        elif label == 'bottle' and self.current_state != 'ccw':
            self.get_logger().info('Bottle detected -> spinning CCW')
            self._set_all(-self.spin_speed)
            self.current_state = 'ccw'
        elif label not in ('person', 'bottle') and self.current_state != 'stop':
            self._set_all(0)
            self.current_state = 'stop'

    def _watchdog_check(self):
        elapsed = (self.get_clock().now() - self.last_msg_time).nanoseconds / 1e9
        if elapsed > 1.0 and self.current_state != 'stop':
            self.get_logger().info('No recent detections -> stopping')
            self._set_all(0)
            self.current_state = 'stop'

    def _keyboard_listener(self):
        while True:
            line = sys.stdin.readline()
            if line.strip().lower() == 'q':
                self.stop_flag.set()
                break

    def shutdown_cleanly(self):
        self._exit_wheel_mode()
        self.port_handler.closePort()


def main(args=None):
    rclpy.init(args=args)
    node = GestureControlNode()
    try:
        while rclpy.ok() and not node.stop_flag.is_set():
            rclpy.spin_once(node, timeout_sec=0.1)
    except KeyboardInterrupt:
        pass
    finally:
        node.shutdown_cleanly()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()