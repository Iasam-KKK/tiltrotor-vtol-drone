#!/usr/bin/env python3
"""
Record the tri-tiltrotor's Gazebo cameras straight to MP4.

Why this exists rather than Gazebo's own <save> element: on this machine
<save enabled="true"> writes no files and the run dumps core, while the camera
sensors themselves render perfectly and publish live image topics. So frames
are taken off the topic instead.

Why ROS 2 rather than gz-transport directly: ros_gz_image bridges the Gazebo
image topic to a standard sensor_msgs/Image, which cv_bridge turns into a
numpy array. That path is already installed and verified on this box, and it
doubles as evidence the aircraft is drivable from ROS 2 -- which is the whole
positioning of this project, not an incidental detail.

No ffmpeg on this machine, so OpenCV does the encoding.

Usage:
    ros2 run ros_gz_image image_bridge /world/capture/.../image &
    python3 record_video.py --topic /wide --out media/master_16x9.mp4
"""

from __future__ import annotations

import argparse
import sys
import time

import cv2
import numpy as np
import rclpy
from cv_bridge import CvBridge
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from sensor_msgs.msg import Image


class Recorder(Node):
    def __init__(self, topic: str, out_path: str, fps: float, timeout: float):
        super().__init__("tritilt_recorder")
        self.out_path = out_path
        self.fps = fps
        self.timeout = timeout
        self.bridge = CvBridge()
        self.writer: cv2.VideoWriter | None = None
        self.frames = 0
        self.last_rx = time.time()
        self.started = time.time()

        # Sensor data is best-effort; a RELIABLE subscriber silently receives
        # nothing from a BEST_EFFORT publisher, which looks identical to "the
        # camera is not publishing".
        qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
        )
        self.sub = self.create_subscription(Image, topic, self.on_image, qos)
        self.create_timer(1.0, self.on_tick)
        self.get_logger().info(f"listening on {topic} -> {out_path}")

    def on_image(self, msg: Image) -> None:
        try:
            frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
        except Exception as exc:                       # noqa: BLE001
            self.get_logger().error(f"cv_bridge failed: {exc}")
            return

        if self.writer is None:
            h, w = frame.shape[:2]
            fourcc = cv2.VideoWriter_fourcc(*"mp4v")
            self.writer = cv2.VideoWriter(self.out_path, fourcc, self.fps, (w, h))
            if not self.writer.isOpened():
                self.get_logger().error(f"could not open writer for {self.out_path}")
                rclpy.shutdown()
                return
            self.get_logger().info(f"writing {w}x{h} @ {self.fps} fps")

        self.writer.write(frame)
        self.frames += 1
        self.last_rx = time.time()

    def on_tick(self) -> None:
        now = time.time()
        if self.frames and now - self.last_rx > 6.0:
            self.get_logger().info("stream went quiet, stopping")
            self.finish()
        elif not self.frames and now - self.started > self.timeout:
            self.get_logger().error("no frames ever arrived")
            self.finish(fail=True)

    def finish(self, fail: bool = False) -> None:
        if self.writer is not None:
            self.writer.release()
        self.get_logger().info(f"wrote {self.frames} frames to {self.out_path}")
        rclpy.shutdown()
        sys.exit(1 if fail else 0)


def main() -> None:
    ap = argparse.ArgumentParser()
    ap.add_argument("--topic", required=True)
    ap.add_argument("--out", required=True)
    ap.add_argument("--fps", type=float, default=30.0)
    ap.add_argument("--timeout", type=float, default=60.0,
                    help="seconds to wait for the first frame")
    args = ap.parse_args()

    rclpy.init()
    node = Recorder(args.topic, args.out, args.fps, args.timeout)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.finish()
    except SystemExit:
        raise


if __name__ == "__main__":
    main()
