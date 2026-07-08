#!/usr/bin/env python3
"""
yolo_detect_node.py
Subscribes to /image_raw, runs YOLOv8 on GPU, detects 'person' and 'bottle'.
Publishes:
  /detected_object_pose  (geometry_msgs/Point)  - pixel-ish centroid of best detection
  /detected_class        (std_msgs/String)      - 'person', 'bottle', or 'none'
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from geometry_msgs.msg import Point
from std_msgs.msg import String
from cv_bridge import CvBridge
import cv2
import time
from ultralytics import YOLO

# Classes we care about for the gesture-control demo
TARGET_CLASSES = ['person', 'bottle']

# Rough depth estimate (meters) — placeholder until real depth estimation exists
ASSUMED_DEPTH_Z = 0.30

# Camera intrinsics for C920 at 720p (approximate)
FX = 600.0
FY = 600.0
CX = 640.0
CY = 360.0


class YoloDetectNode(Node):
    def __init__(self):
        super().__init__('yolo_detect_node')

        self.model = YOLO('yolov8n.pt')
        self.model.to('cuda')
        self.get_logger().info(f'YOLOv8 loaded on CUDA | targets: {TARGET_CLASSES}')

        self.subscription = self.create_subscription(
            Image, '/image_raw', self.image_callback, 10)

        self.pose_pub = self.create_publisher(Point, '/detected_object_pose', 10)
        self.class_pub = self.create_publisher(String, '/detected_class', 10)

        self.bridge = CvBridge()
        self.frame_count = 0

    def image_callback(self, msg):
        t0 = time.perf_counter()

        frame = self.bridge.imgmsg_to_cv2(msg, 'bgr8')
        results = self.model(frame, device='cuda', verbose=False)[0]

        best_box = None
        best_conf = 0.0
        best_label = 'none'

        for box in results.boxes:
            cls_id = int(box.cls[0])
            label = self.model.names[cls_id]
            conf = float(box.conf[0])

            if label in TARGET_CLASSES and conf > best_conf:
                best_conf = conf
                best_box = box.xyxy[0].cpu().numpy()  # [x1, y1, x2, y2]
                best_label = label

        t1 = time.perf_counter()
        infer_ms = (t1 - t0) * 1000
        self.frame_count += 1

        class_msg = String()
        class_msg.data = best_label
        self.class_pub.publish(class_msg)

        if best_box is not None:
            x1, y1, x2, y2 = best_box
            cx_px = (x1 + x2) / 2.0
            cy_px = (y1 + y2) / 2.0

            # Rough pixel -> approximate world offset using pinhole model
            x_world = (cx_px - CX) * ASSUMED_DEPTH_Z / FX
            y_world = (cy_px - CY) * ASSUMED_DEPTH_Z / FY

            pt = Point()
            pt.x = float(x_world)
            pt.y = float(y_world)
            pt.z = float(ASSUMED_DEPTH_Z)
            self.pose_pub.publish(pt)

            if self.frame_count % 15 == 0:
                self.get_logger().info(
                    f'{best_label} conf={best_conf:.2f} infer={infer_ms:.1f}ms'
                )
        else:
            if self.frame_count % 30 == 0:
                self.get_logger().debug(f'No target detected | infer={infer_ms:.1f}ms')


def main(args=None):
    rclpy.init(args=args)
    node = YoloDetectNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()