#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2

# Windows host MJPEG stream (ffmpeg serving the C920)
STREAM_URL = 'http://172.31.208.1:8080'

class CameraPublisher(Node):
    def __init__(self):
        super().__init__('camera_publisher')
        self.publisher = self.create_publisher(Image, '/image_raw', 10)
        self.timer = self.create_timer(1/30.0, self.timer_callback)
        self.bridge = CvBridge()
        self.cap = cv2.VideoCapture(STREAM_URL)
        self.count = 0
        if not self.cap.isOpened():
            self.get_logger().error(f'Failed to open stream: {STREAM_URL}')
        else:
            self.get_logger().info(f'Camera publisher started! Stream: {STREAM_URL}')

    def timer_callback(self):
        ret, frame = self.cap.read()
        if ret:
            msg = self.bridge.cv2_to_imgmsg(frame, 'bgr8')
            self.publisher.publish(msg)
            self.count += 1
            if self.count % 30 == 0:
                self.get_logger().info(f'Published {self.count} frames')
        else:
            self.get_logger().warn('Failed to grab frame!')

def main():
    rclpy.init()
    node = CameraPublisher()
    rclpy.spin(node)
    node.cap.release()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
