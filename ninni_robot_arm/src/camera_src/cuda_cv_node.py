#!/usr/bin/env python3
import sys
sys.path.insert(0, '/usr/local/lib/python3.10/dist-packages')

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import String
from cv_bridge import CvBridge
import cv2
import time

class CudaCVNode(Node):
    def __init__(self):
        super().__init__('cuda_cv_node')
        self.subscription = self.create_subscription(
            Image, '/image_raw', self.image_callback, 10)
        self.publisher = self.create_publisher(Image, '/image_processed', 10)
        self.stm32_sub = self.create_subscription(
            String, '/stm32_cmd', self.stm32_callback, 10)
        self.bridge = CvBridge()
        self.frame_count = 0
        self.latest_frame = None
        self.get_logger().info(f'CUDA CV Node started — GPU count: {cv2.cuda.getCudaEnabledDeviceCount()}')

    def stm32_callback(self, msg):
        if msg.data.strip() == 'SNAP':
            if self.latest_frame is not None:
                filename = f'snapshot_{int(time.time())}.png'
                cv2.imwrite(filename, self.latest_frame)
                self.get_logger().info(f'📸 Snapshot saved: {filename}')
            else:
                self.get_logger().warn('SNAP received but no frame available yet!')

    def image_callback(self, msg):
        t0 = time.perf_counter()

        frame = self.bridge.imgmsg_to_cv2(msg, 'bgr8')
        self.latest_frame = frame

        gpu_frame = cv2.cuda_GpuMat()
        gpu_frame.upload(frame)

        gpu_gray = cv2.cuda.cvtColor(gpu_frame, cv2.COLOR_BGR2GRAY)

        gaussian = cv2.cuda.createGaussianFilter(
            cv2.CV_8UC1, cv2.CV_8UC1, (15, 15), 0)
        gpu_blurred = gaussian.apply(gpu_gray)

        canny = cv2.cuda.createCannyEdgeDetector(50, 150)
        gpu_edges = canny.detect(gpu_blurred)

        edges = gpu_edges.download()

        t1 = time.perf_counter()
        process_ms = (t1 - t0) * 1000

        self.frame_count += 1
        if self.frame_count % 30 == 0:
            self.get_logger().info(f'Frame {self.frame_count} | CUDA process time: {process_ms:.2f}ms')

        edges_color = cv2.cvtColor(edges, cv2.COLOR_GRAY2BGR)
        combined = cv2.hconcat([frame, edges_color])
        cv2.putText(combined, f'CUDA: {process_ms:.1f}ms', (10, 30),
                    cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2)
        cv2.imshow('Ninni Robot Arm - CUDA CV', combined)
        cv2.waitKey(1)

        out_msg = self.bridge.cv2_to_imgmsg(edges, 'mono8')
        self.publisher.publish(out_msg)

def main():
    rclpy.init()
    node = CudaCVNode()
    rclpy.spin(node)
    cv2.destroyAllWindows()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
