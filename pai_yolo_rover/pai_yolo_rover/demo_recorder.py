"""Record a side-by-side demo video: bird's-eye view (left) + YOLO front-camera view (right).

Frames are written on a ROS timer driven by *simulation* time (use_sim_time:=true), so the
resulting video plays back at real simulated speed even if the PC runs Gazebo slower than
real time.
"""
import os

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image
from std_msgs.msg import String

import cv2

from pai_yolo_rover.yolo_search_node import image_to_bgr


class DemoRecorder(Node):
    def __init__(self):
        super().__init__('demo_recorder')
        self.declare_parameter('video_path', os.path.expanduser('~/pai_demo.avi'))
        self.declare_parameter('fps', 10.0)
        self.declare_parameter('overhead_topic', '/overhead/image_raw')
        self.declare_parameter('yolo_topic', '/yolo_search/annotated')
        self.path = self.get_parameter('video_path').value
        self.fps = float(self.get_parameter('fps').value)
        self.over = None
        self.yolo = None
        self.status = 'waiting for YOLO node...'
        self.writer = None
        self.frames = 0
        self.create_subscription(Image, self.get_parameter('overhead_topic').value,
                                 lambda m: setattr(self, 'over', m), qos_profile_sensor_data)
        self.create_subscription(Image, self.get_parameter('yolo_topic').value,
                                 lambda m: setattr(self, 'yolo', m), 10)
        self.create_subscription(String, '/yolo_search/status',
                                 lambda m: setattr(self, 'status', m.data), 10)
        self.create_timer(1.0 / self.fps, self.tick)
        # make sure the file is finalised even when launch stops us with SIGINT
        self.context.on_shutdown(self.close)
        self.get_logger().info(f'recording to {self.path}')

    def tick(self):
        if self.over is None:
            return
        left = cv2.resize(image_to_bgr(self.over), (640, 480))
        if self.yolo is not None:
            right = image_to_bgr(self.yolo)
            right = cv2.resize(right, (640, int(right.shape[0] * 640 / right.shape[1])))
        else:
            right = np.zeros((550, 640, 3), np.uint8)
        h = max(left.shape[0], right.shape[0])
        pad = lambda im: np.vstack([im, np.zeros((h - im.shape[0], 640, 3), np.uint8)])  # noqa: E731
        frame = np.hstack([pad(left), pad(right)])
        t = self.get_clock().now().nanoseconds * 1e-9
        cv2.putText(frame, f'sim t={t:6.1f}s', (10, h - 15), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 2)
        cv2.putText(frame, "bird's-eye view", (10, 25), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 255), 2)
        if self.writer is None:
            # MJPG/AVI stays readable even if the process is killed before release()
            self.writer = cv2.VideoWriter(self.path, cv2.VideoWriter_fourcc(*'MJPG'), self.fps,
                                          (frame.shape[1], frame.shape[0]))
        self.writer.write(frame)
        self.frames += 1

    def close(self):
        if self.writer is not None:
            self.writer.release()
            self.writer = None
            print(f'[demo_recorder] saved {self.frames} frames -> {self.path}', flush=True)


def main(args=None):
    rclpy.init(args=args)
    node = DemoRecorder()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, Exception):  # noqa: BLE001  (also ExternalShutdownException)
        pass
    finally:
        node.close()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
