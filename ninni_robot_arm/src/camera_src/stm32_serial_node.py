#!/usr/bin/env python3
#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import serial

class STM32SerialNode(Node):
    def __init__(self):
        super().__init__('stm32_serial_node')
        self.publisher_ = self.create_publisher(String, '/stm32_cmd', 10)
        self.serial_ = serial.Serial('/dev/ttyACM0', 115200, timeout=1.0)
        self.timer_ = self.create_timer(0.01, self.read_serial)  # 100Hz polling
        self.get_logger().info('STM32 Serial Node started — listening on /dev/ttyACM0')

    def read_serial(self):
        if self.serial_.in_waiting > 0:
            line = self.serial_.readline().decode('utf-8', errors='ignore').strip()
            if line:
                self.get_logger().info(f'Received: {line}')
                msg = String()
                msg.data = line
                self.publisher_.publish(msg)

    def destroy_node(self):
        self.serial_.close()
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    node = STM32SerialNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
