#!/usr/bin/env python3
"""
Camera Logger
COMPSYS732

Run with:
    ~/ros2_venv/bin/python3 camera_logger.py --namespace /TXX --mode view
    ~/ros2_venv/bin/python3 camera_logger.py --namespace /TXX --mode detect

Modes:
  view    — live camera window only
  detect  — live window with red detection overlay
             saves snapshot + pixel count to CSV on each detection
"""

import argparse, csv, os, time
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import CompressedImage
import cv2
import numpy as np

# Minimum red pixels to count as a detection
DETECTION_THRESHOLD = 500

# HSV range for red (two ranges needed as red wraps around 0/180)
RED_LOW1,  RED_HIGH1  = np.array([0,   120, 70]), np.array([10,  255, 255])
RED_LOW2,  RED_HIGH2  = np.array([170, 120, 70]), np.array([180, 255, 255])


class CameraLogger(Node):
    def __init__(self, namespace, mode):
        super().__init__('camera_logger')
        self.mode = mode
        self.namespace = namespace.rstrip('/')
        self.detection_count = 0

        topic = f"{self.namespace}/oakd/rgb/image_raw/compressed"
        self.create_subscription(CompressedImage, topic, self.callback, 10)
        print(f"\n  Camera Logger — Mode: {mode.upper()}")
        print(f"  Topic: {topic}")
        print(f"  Press Ctrl+C to stop\n")

    def callback(self, msg):
        arr = np.frombuffer(msg.data, np.uint8)
        img = cv2.imdecode(arr, cv2.IMREAD_COLOR)
        if img is None:
            return

        if self.mode == 'view':
            cv2.imshow('Camera', img)
            cv2.waitKey(1)

        elif self.mode == 'detect':
            hsv  = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
            mask = cv2.inRange(hsv, RED_LOW1, RED_HIGH1) | cv2.inRange(hsv, RED_LOW2, RED_HIGH2)
            pixels = int(cv2.countNonZero(mask))

            # Highlight detected pixels in the display
            overlay = img.copy()
            overlay[mask > 0] = [0, 0, 255]
            cv2.putText(overlay, f"Red pixels: {pixels}", (10, 30),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 255, 0), 2)

            if pixels > DETECTION_THRESHOLD:
                cv2.putText(overlay, "DETECTED", (10, 65),
                            cv2.FONT_HERSHEY_SIMPLEX, 1.0, (0, 0, 255), 3)
                self.save_detection(img, pixels)

            cv2.imshow('Camera — Red Detection', overlay)
            cv2.waitKey(1)

    def save_detection(self, img, pixels):
        self.detection_count += 1
        snap_path = os.path.expanduser(f'~/detection_{self.detection_count}.png')
        csv_path  = os.path.expanduser('~/detections.csv')

        cv2.imwrite(snap_path, img)

        write_header = not os.path.exists(csv_path)
        with open(csv_path, 'a', newline='') as f:
            writer = csv.DictWriter(f, fieldnames=['detection', 'pixels', 'snapshot', 'time'])
            if write_header:
                writer.writeheader()
            writer.writerow({
                'detection': self.detection_count,
                'pixels':    pixels,
                'snapshot':  snap_path,
                'time':      round(time.time(), 2),
            })

        print(f"  Detection {self.detection_count} — {pixels} red pixels — saved: {snap_path}")


def main():
    parser = argparse.ArgumentParser(description='Lab 4D — Camera Logger')
    parser.add_argument('--namespace', required=True, help='Robot namespace e.g. /T10')
    parser.add_argument('--mode', choices=['view', 'detect'], default='view')
    args = parser.parse_args()

    rclpy.init()
    node = CameraLogger(args.namespace, args.mode)
    try:
        while rclpy.ok():
            rclpy.spin_once(node, timeout_sec=0.05)
    except KeyboardInterrupt:
        print("\n  Stopped.")
    finally:
        cv2.destroyAllWindows()
        try:
            node.destroy_node()
            rclpy.shutdown()
        except Exception:
            pass


if __name__ == '__main__':
    main()
