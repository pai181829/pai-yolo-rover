"""YOLO-guided object search node.

The rover visits a list of target objects in order. For each target it
  SEARCH   : rotates in place (and occasionally moves forward) until YOLO finds the target
  APPROACH : keeps the target in the image centre (P control on bbox centre) and drives forward
  ARRIVED  : stops when the target is large enough in the image (= close enough)
  BACKOFF  : reverses a little, then starts searching for the next target
After the last target it switches to DONE and stops.

AI usage: a COCO-pretrained YOLO model (Ultralytics, inference only) runs on the
robot's front camera image. No training is required.
"""
import math
import time

import numpy as np
import rclpy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image
from std_msgs.msg import String

try:
    import cv2
except ImportError:  # pragma: no cover
    cv2 = None


def image_to_bgr(msg: Image) -> np.ndarray:
    """Convert sensor_msgs/Image to an OpenCV BGR array without cv_bridge.

    Avoiding cv_bridge keeps the node independent from the numpy ABI that
    cv_bridge was compiled against (pip installs of ultralytics may upgrade numpy).
    """
    buf = np.frombuffer(msg.data, dtype=np.uint8)
    if msg.encoding in ('rgb8', 'bgr8'):
        img = buf.reshape(msg.height, msg.step)[:, :msg.width * 3].reshape(msg.height, msg.width, 3)
        return img[:, :, ::-1].copy() if msg.encoding == 'rgb8' else img.copy()
    if msg.encoding in ('rgba8', 'bgra8'):
        img = buf.reshape(msg.height, msg.step)[:, :msg.width * 4].reshape(msg.height, msg.width, 4)
        img = img[:, :, :3]
        return img[:, :, ::-1].copy() if msg.encoding == 'rgba8' else img.copy()
    if msg.encoding == 'mono8':
        img = buf.reshape(msg.height, msg.step)[:, :msg.width]
        return np.repeat(img[:, :, None], 3, axis=2)
    raise ValueError(f'unsupported encoding: {msg.encoding}')


def bgr_to_image(img: np.ndarray, stamp, frame_id: str) -> Image:
    msg = Image()
    msg.header.stamp = stamp
    msg.header.frame_id = frame_id
    msg.height, msg.width = img.shape[:2]
    msg.encoding = 'bgr8'
    msg.step = msg.width * 3
    msg.data = np.ascontiguousarray(img).tobytes()
    return msg


class YoloSearch(Node):
    SEARCH, APPROACH, ARRIVED, BACKOFF, RECOVER, DONE = 'SEARCH', 'APPROACH', 'ARRIVED', 'BACKOFF', 'RECOVER', 'DONE'

    def __init__(self):
        super().__init__('yolo_search')
        p = self.declare_parameter
        p('targets', ['stop sign', 'person', 'fire hydrant'])
        p('model', 'yolov8n.pt')
        p('device', 'cpu')
        p('imgsz', 640)
        p('conf', 0.35)
        p('image_topic', '/camera/image_raw')
        p('cmd_vel_topic', '/cmd_vel')
        p('control_period', 0.2)
        p('search_ang_vel', 0.45)          # rad/s
        p('max_lin_vel', 0.35)             # m/s
        p('k_ang', 1.1)                    # P gain on normalised x-error
        p('arrive_ratio', 0.50)            # bbox height / image height
        p('arrive_overrides', ['stop sign:0.42', 'fire hydrant:0.45', 'person:0.66'])
        p('lost_tolerance', 6)             # control cycles to keep approaching without a detection
        p('hold_time', 2.0)                # s to stay at a target
        p('backoff_time', 1.6)             # s to reverse after arriving
        p('show_window', False)
        p('loop', False)                   # restart mission after DONE
        p('stuck_time', 3.0)               # s of commanded motion without odometry motion -> RECOVER

        g = lambda n: self.get_parameter(n).value  # noqa: E731
        self.targets = list(g('targets'))
        self.conf = float(g('conf'))
        self.imgsz = int(g('imgsz'))
        self.device = str(g('device'))
        self.search_w = float(g('search_ang_vel'))
        self.max_v = float(g('max_lin_vel'))
        self.k_ang = float(g('k_ang'))
        self.arrive_default = float(g('arrive_ratio'))
        self.arrive = {}
        for item in g('arrive_overrides'):
            name, _, val = item.rpartition(':')
            if name:
                self.arrive[name.strip()] = float(val)
        self.lost_tol = int(g('lost_tolerance'))
        self.hold_time = float(g('hold_time'))
        self.backoff_time = float(g('backoff_time'))
        self.show = bool(g('show_window')) and cv2 is not None
        self.loop = bool(g('loop'))
        self.dt = float(g('control_period'))
        self.stuck_time = float(g('stuck_time'))

        from ultralytics import YOLO  # imported lazily so `--help` works without torch
        self.get_logger().info(f'loading YOLO model {g("model")} ...')
        self.model = YOLO(g('model'))
        self.names = self.model.names
        self.get_logger().info(f'targets: {self.targets}')

        self.cmd_pub = self.create_publisher(Twist, g('cmd_vel_topic'), 10)
        self.ann_pub = self.create_publisher(Image, '/yolo_search/annotated', 2)
        self.status_pub = self.create_publisher(String, '/yolo_search/status', 10)
        self.create_subscription(Image, g('image_topic'), self.on_image, qos_profile_sensor_data)
        self.create_subscription(Odometry, '/odom', self.on_odom, 10)
        self.odom_v = 0.0
        self.stuck_since = None

        self.latest = None
        self.idx = 0
        self.state = self.SEARCH
        self.state_t0 = self.now()
        self.search_t0 = self.now()
        self.lost = 0
        self.last_ang = 0.0
        self.infer_ms = 0.0
        self.create_timer(self.dt, self.step)

    # ------------------------------------------------------------------ utils
    def now(self) -> float:
        return self.get_clock().now().nanoseconds * 1e-9

    def set_state(self, s):
        if s != self.state:
            self.get_logger().info(f'[{self.target or "-"}] {self.state} -> {s}')
        self.state = s
        self.state_t0 = self.now()
        if s == self.SEARCH:
            self.search_t0 = self.now()

    @property
    def target(self):
        return self.targets[self.idx] if self.idx < len(self.targets) else None

    def on_image(self, msg: Image):
        self.latest = msg

    def on_odom(self, msg: Odometry):
        self.odom_v = msg.twist.twist.linear.x

    def publish_cmd(self, v, w):
        t = Twist()
        t.linear.x = float(v)
        t.angular.z = float(w)
        self.cmd_pub.publish(t)

    # ------------------------------------------------------------- perception
    def detect(self, img):
        t0 = time.perf_counter()
        res = self.model.predict(img, imgsz=self.imgsz, conf=self.conf, device=self.device, verbose=False)[0]
        self.infer_ms = (time.perf_counter() - t0) * 1000.0
        dets = []
        if res.boxes is not None and len(res.boxes) > 0:
            xyxy = res.boxes.xyxy.cpu().numpy()
            cls = res.boxes.cls.cpu().numpy().astype(int)
            cf = res.boxes.conf.cpu().numpy()
            for b, c, s in zip(xyxy, cls, cf):
                dets.append((self.names[int(c)], float(s), [float(v) for v in b]))
        return dets

    # ---------------------------------------------------------------- control
    def step(self):
        if self.latest is None:
            return
        msg = self.latest
        img = image_to_bgr(msg)
        H, W = img.shape[:2]
        dets = self.detect(img)
        tgt = self.target
        cand = [d for d in dets if d[0] == tgt]
        best = max(cand, key=lambda d: (d[2][2] - d[2][0]) * (d[2][3] - d[2][1])) if cand else None

        v = w = 0.0
        ratio = 0.0
        if best is not None:
            x1, y1, x2, y2 = best[2]
            ratio = max((y2 - y1) / H, 0.6 * (x2 - x1) / W)
        goal = self.arrive.get(tgt, self.arrive_default) if tgt else 0.0
        t_in = self.now() - self.state_t0

        if self.state == self.SEARCH:
            if best is not None:
                self.lost = 0
                self.set_state(self.APPROACH)
            else:
                # rotate in place; after ~1.2 turns without success, drive forward briefly
                turn_period = 2 * math.pi / self.search_w * 1.2
                if (self.now() - self.search_t0) % (turn_period + 2.5) > turn_period:
                    v, w = 0.2, 0.0
                else:
                    w = self.search_w
        if self.state == self.APPROACH:
            if best is not None:
                self.lost = 0
                x1, _, x2, _ = best[2]
                err = ((x1 + x2) / 2 - W / 2) / (W / 2)      # -1 (left) .. +1 (right)
                w = float(np.clip(-self.k_ang * err, -0.8, 0.8))
                slow = float(np.clip((goal - ratio) / goal * 2.0, 0.25, 1.0))
                v = self.max_v * slow * max(0.0, 1.0 - 1.6 * abs(err))
                self.last_ang = w
                if ratio >= goal:
                    v = w = 0.0
                    self.set_state(self.ARRIVED)
            else:
                self.lost += 1
                if self.lost > self.lost_tol:
                    self.set_state(self.SEARCH)
                else:
                    v, w = 0.05, self.last_ang * 0.5
        elif self.state == self.RECOVER:
            # reverse -> turn away -> drive forward, then search again (simple obstacle escape)
            if t_in < 1.5:
                v = -0.2
            elif t_in < 3.5:
                w = 0.7
            elif t_in < 5.5:
                v = 0.25
            else:
                self.set_state(self.SEARCH)
        elif self.state == self.ARRIVED:
            if t_in >= self.hold_time:
                self.set_state(self.BACKOFF)
        elif self.state == self.BACKOFF:
            v = -0.2
            if t_in >= self.backoff_time:
                self.idx += 1
                if self.idx >= len(self.targets):
                    if self.loop:
                        self.idx = 0
                        self.set_state(self.SEARCH)
                    else:
                        self.set_state(self.DONE)
                else:
                    self.set_state(self.SEARCH)
                v = 0.0
        elif self.state == self.DONE:
            v = w = 0.0

        # stuck detection: we ask to move forward but the wheels' odometry says we do not move
        if self.state == self.APPROACH and v > 0.08 and abs(self.odom_v) < 0.02:
            self.stuck_since = self.stuck_since or self.now()
            if self.now() - self.stuck_since > self.stuck_time:
                self.get_logger().warn('stuck (obstacle?) -> RECOVER')
                self.set_state(self.RECOVER)
                v = w = 0.0
        else:
            self.stuck_since = None

        self.publish_cmd(v, w)
        status = (f'state={self.state} target={tgt or "-"} ({min(self.idx + 1, len(self.targets))}/'
                  f'{len(self.targets)}) size={ratio:.2f}/{goal:.2f} v={v:.2f} w={w:.2f} '
                  f'yolo={self.infer_ms:.0f}ms')
        self.status_pub.publish(String(data=status))
        self.publish_annotated(img, dets, best, ratio, goal, v, w, msg.header.stamp)

    # ------------------------------------------------------------ visualising
    def publish_annotated(self, img, dets, best, ratio, goal, v, w, stamp):
        if cv2 is None:
            return
        out = img.copy()
        H, W = out.shape[:2]
        for name, score, (x1, y1, x2, y2) in dets:
            is_t = best is not None and name == best[0] and [x1, y1, x2, y2] == best[2]
            col = (0, 0, 255) if is_t else ((0, 200, 255) if name == self.target else (180, 180, 180))
            cv2.rectangle(out, (int(x1), int(y1)), (int(x2), int(y2)), col, 3 if is_t else 1)
            cv2.putText(out, f'{name} {score:.2f}', (int(x1), max(15, int(y1) - 6)),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.55, col, 2)
        cv2.line(out, (W // 2, H - 30), (W // 2, H - 10), (255, 255, 255), 2)
        bar = np.zeros((70, W, 3), np.uint8)
        state_col = {self.SEARCH: (0, 200, 255), self.APPROACH: (0, 255, 0), self.ARRIVED: (255, 120, 0),
                     self.BACKOFF: (200, 200, 200), self.RECOVER: (0, 0, 255), self.DONE: (255, 0, 255)}[self.state]
        cv2.putText(bar, f'{self.state}', (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.9, state_col, 2)
        tgt = self.target or 'mission complete'
        cv2.putText(bar, f'target: {tgt} ({min(self.idx + 1, len(self.targets))}/{len(self.targets)})',
                    (200, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.65, (255, 255, 255), 2)
        cv2.putText(bar, f'size {ratio:.2f}/{goal:.2f}  v={v:+.2f}m/s  w={w:+.2f}rad/s  YOLO {self.infer_ms:.0f}ms',
                    (10, 60), cv2.FONT_HERSHEY_SIMPLEX, 0.55, (220, 220, 220), 1)
        out = np.vstack([out, bar])
        self.ann_pub.publish(bgr_to_image(out, stamp, 'camera_optical_link'))
        if self.show:
            cv2.imshow('YOLO search (front camera)', out)
            cv2.waitKey(1)


def main(args=None):
    rclpy.init(args=args)
    node = YoloSearch()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        try:
            node.publish_cmd(0.0, 0.0)
        except Exception:  # noqa: BLE001
            pass
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
