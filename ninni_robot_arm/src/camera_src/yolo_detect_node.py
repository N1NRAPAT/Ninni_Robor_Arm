import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from geometry_msgs.msg import Point
from cv_bridge import CvBridge
import cv2
import numpy as np
import time
import threading
from ultralytics import YOLO

TARGET_CLASS = 'bottle'
ASSUMED_DEPTH_Z = 0.30
FX = 600.0
FY = 600.0
CX = 320.0
CY = 240.0

class YoloDetectNode(Node):
    def __init__(self):
        super().__init__('yolo_detect_node')
        self.model = YOLO('yolov8n.pt')
        self.model.to('cuda')
        self.get_logger().info(f'YOLOv8 loaded on CUDA | target: [{TARGET_CLASS}]')

        self.subscription = self.create_subscription(
            Image, '/image_raw', self.image_callback, 1)
        self.pose_pub = self.create_publisher(Point, '/detected_object_pose', 10)
        self.bridge = CvBridge()

        # Shared state between threads
        self.latest_frame = None       # latest raw frame for display (30fps)
        self.yolo_frame = None         # frame queued for YOLO (throttled)
        self.last_boxes = []           # last known YOLO boxes to overlay
        self.last_infer_ms = 0.0
        self.frame_count = 0
        self.lock = threading.Lock()

        # YOLO runs in a background thread so it never blocks display
        self.yolo_thread = threading.Thread(target=self.yolo_worker, daemon=True)
        self.yolo_thread.start()

        # Display timer at full 30fps
        self.timer = self.create_timer(1/30.0, self.display_frame)

    def image_callback(self, msg):
        try:
            frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        except Exception as e:
            self.get_logger().warn(f'Bridge error: {e}')
            return

        with self.lock:
            self.latest_frame = frame.copy()
            # Only queue a new YOLO frame if worker is free
            if self.yolo_frame is None:
                self.yolo_frame = frame.copy()

    def yolo_worker(self):
        """Runs in background thread — processes frames as fast as GPU allows"""
        while True:
            frame = None
            with self.lock:
                if self.yolo_frame is not None:
                    frame = self.yolo_frame
                    self.yolo_frame = None

            if frame is None:
                time.sleep(0.001)
                continue

            t0 = time.perf_counter()
            results = self.model(frame, device='cuda', verbose=False)[0]
            infer_ms = (time.perf_counter() - t0) * 1000

            boxes = []
            for box in results.boxes:
                cls_id = int(box.cls[0])
                label = self.model.names[cls_id]
                conf = float(box.conf[0])
                if label == TARGET_CLASS and conf > 0.3:
                    xyxy = box.xyxy[0].cpu().numpy()
                    boxes.append((xyxy, conf))

                    # Publish pose for best detection
                    x1, y1, x2, y2 = xyxy
                    cx_px = (x1 + x2) / 2.0
                    cy_px = (y1 + y2) / 2.0
                    X = (cx_px - CX) * ASSUMED_DEPTH_Z / FX
                    Y = (cy_px - CY) * ASSUMED_DEPTH_Z / FY
                    pose_msg = Point()
                    pose_msg.x = float(X)
                    pose_msg.y = float(Y)
                    pose_msg.z = float(ASSUMED_DEPTH_Z)
                    self.pose_pub.publish(pose_msg)

            with self.lock:
                self.last_boxes = boxes
                self.last_infer_ms = infer_ms

            self.frame_count += 1
            if self.frame_count % 30 == 0:
                self.get_logger().info(
                    f'YOLO infer={infer_ms:.1f}ms | detections={len(boxes)}')

    def display_frame(self):
        """Runs at 30fps — shows latest camera frame with last known YOLO boxes"""
        with self.lock:
            if self.latest_frame is None:
                return
            frame = self.latest_frame.copy()
            boxes = list(self.last_boxes)
            infer_ms = self.last_infer_ms

        # Overlay last known boxes
        for (xyxy, conf) in boxes:
            x1, y1, x2, y2 = xyxy
            cx_px = int((x1 + x2) / 2)
            cy_px = int((y1 + y2) / 2)
            X = (cx_px - CX) * ASSUMED_DEPTH_Z / FX
            Y = (cy_px - CY) * ASSUMED_DEPTH_Z / FY
            cv2.rectangle(frame, (int(x1), int(y1)), (int(x2), int(y2)), (0, 255, 0), 2)
            cv2.circle(frame, (cx_px, cy_px), 6, (0, 0, 255), -1)
            cv2.putText(frame, f'{TARGET_CLASS} {conf:.2f}',
                        (int(x1), int(y1) - 10),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
            cv2.putText(frame, f'X:{X:.2f}m Y:{Y:.2f}m Z:{ASSUMED_DEPTH_Z:.2f}m',
                        (int(x1), int(y2) + 25),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 0), 2)

        cv2.putText(frame, f'YOLO infer: {infer_ms:.1f}ms', (10, 30),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 200, 255), 2)
        cv2.imshow('Ninni Robot Arm - YOLO Detection', frame)
        cv2.waitKey(1)


def main():
    rclpy.init()
    node = YoloDetectNode()
    rclpy.spin(node)
    cv2.destroyAllWindows()
    rclpy.shutdown()

if __name__ == '__main__':
    main()