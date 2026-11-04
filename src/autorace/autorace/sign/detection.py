'''
Mission sign detection with the YOLO11n mission model (imgsz 320).
Copied from turtlebot-autorace-2026/lane_detection_dl mission_model (f1dcce7). Only change: the model
is found by file name in this package's models/ (it holds the lane model too).

  /camera2 (CompressedImage)        front camera
  /parking, /left, ... (Bool)       one topic per sign, published when its state changes
  /mission_model/debug/compressed   boxes on the front image (publish_debug)

Model classes (8 signs, see label_image.jpg) -> topics (parameter `topics`, same order
as the class ids):
  0 parking    -> /parking      4 stop         -> /stop
  1 left_turn  -> /left         5 construction -> /construction
  2 right_turn -> /right        6 intersection -> /intersection
  3 no_entry   -> /no_entry     7 tunnel       -> /tunnel

A sign counts in a frame when a box of its class is above `conf` and its longer side
is at least `min_box_size` of the image height (sign close enough). Each sign has its
own state: true after `confirm_frames` counting frames in a row, false again after
`release_frames` frames in a row without it. Several signs can be true at once.
All topics start false and are latched (transient local), so a node started later
still gets the current state.
'''
import os
import cv2
import rclpy
import numpy as np
from rclpy.node import Node
from rclpy.qos import QoSProfile, DurabilityPolicy
from std_msgs.msg import Bool
from sensor_msgs.msg import CompressedImage
from ament_index_python.packages import get_package_share_directory
from ultralytics import YOLO

CLASSES = ['parking', 'left_turn', 'right_turn', 'no_entry', 'stop', 'construction', 'intersection', 'tunnel']
TOPICS = ['/parking', '/left', '/right', '/no_entry', '/stop', '/construction', '/intersection', '/tunnel']


class MissionDetection(Node):
    def __init__(self):
        super().__init__('mission_detection_node')
        self.declare_parameter('model_path', '')        # '' = models/yolo11n_mission.pt in this package
        self.declare_parameter('image_topic', '/camera2')
        self.declare_parameter('topics', TOPICS)
        self.declare_parameter('conf', 0.7)
        self.declare_parameter('imgsz', 320)
        self.declare_parameter('device', '')          # '' = auto (cuda if available), 'cpu', '0'
        self.declare_parameter('min_box_size', 0.10)    # longer box side / image height: ignore far signs
        self.declare_parameter('confirm_frames', 3)     # frames in a row with the sign before true
        self.declare_parameter('release_frames', 5)     # frames in a row without the sign before false
        self.declare_parameter('process_hz', 10.0)
        self.declare_parameter('publish_debug', True)   # boxes on /mission_model/debug/compressed
        self.declare_parameter('show', False)           # cv2 debug window
        self.conf = self.get_parameter('conf').value
        self.imgsz = self.get_parameter('imgsz').value
        self.device = self.get_parameter('device').value or None
        self.min_box_size = self.get_parameter('min_box_size').value
        self.confirm_frames = self.get_parameter('confirm_frames').value
        self.release_frames = self.get_parameter('release_frames').value
        self.show = self.get_parameter('show').value

        model_path = self.get_parameter('model_path').value
        if not model_path:
            model_path = os.path.join(get_package_share_directory('autorace'), 'models', 'yolo11n_mission.pt')
            if not os.path.exists(model_path):
                raise ValueError(f'set model_path: {model_path} not found')
        self.model = YOLO(model_path, task='detect')
        if len(self.model.names) != len(CLASSES):
            raise ValueError(f'{model_path} has {len(self.model.names)} classes, expected {len(CLASSES)}: {CLASSES}')
        topics = list(self.get_parameter('topics').value)
        if len(topics) != len(CLASSES):
            raise ValueError(f'topics has {len(topics)} names, expected {len(CLASSES)}')
        self.get_logger().info(f'Mission model loaded: {model_path} '
                               + ', '.join(f'{c}->{t}' for c, t in zip(CLASSES, topics)))

        self.videoSubscriber = self.create_subscription(
            CompressedImage, self.get_parameter('image_topic').value, self.videoSubscriber_callback, 10)
        latched = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.signPublishers = [self.create_publisher(Bool, t, latched) for t in topics]
        self.debugPublisher = None
        if self.get_parameter('publish_debug').value:
            self.debugPublisher = self.create_publisher(CompressedImage, '/mission_model/debug/compressed', 10)

        n = len(CLASSES)
        self.active = [False] * n
        self.seen_count = [0] * n       # frames in a row with the sign
        self.missed_count = [0] * n     # frames in a row without the sign
        for pub in self.signPublishers:
            pub.publish(Bool(data=False))
        # process only the latest frame (inference may be slower than the camera)
        self.latest_msg = None
        self.timer = self.create_timer(1.0 / self.get_parameter('process_hz').value, self.process)

    def videoSubscriber_callback(self, msg):
        self.latest_msg = msg

    def detect(self, src):
        '''YOLO inference -> list of (class id, conf, x1, y1, x2, y2)'''
        result = self.model.predict(src, imgsz=self.imgsz, conf=self.conf, device=self.device, verbose=False)[0]
        if result.boxes is None or len(result.boxes) == 0:
            return []
        cls = result.boxes.cls.cpu().numpy().astype(int)
        confs = result.boxes.conf.cpu().numpy()
        xyxy = result.boxes.xyxy.cpu().numpy()
        return [(int(c), float(p), *map(float, b)) for c, p, b in zip(cls, confs, xyxy)]

    def seen_classes(self, detections, height):
        '''Classes with a close-enough box in this frame.'''
        return {d[0] for d in detections if max(d[4] - d[2], d[5] - d[3]) >= self.min_box_size * height}

    def update(self, seen):
        '''Per-sign debounce; publish the signs whose state changed.'''
        for c in range(len(CLASSES)):
            if c in seen:
                self.seen_count[c] += 1
                self.missed_count[c] = 0
            else:
                self.missed_count[c] += 1
                self.seen_count[c] = 0
            if not self.active[c] and self.seen_count[c] >= self.confirm_frames:
                self.active[c] = True
            elif self.active[c] and self.missed_count[c] >= self.release_frames:
                self.active[c] = False
            else:
                continue
            self.signPublishers[c].publish(Bool(data=self.active[c]))
            self.get_logger().info(f'===>> {self.signPublishers[c].topic_name}: {self.active[c]}')

    def process(self):
        msg, self.latest_msg = self.latest_msg, None
        if msg is None:
            return
        try:
            src = cv2.imdecode(np.frombuffer(msg.data, np.uint8), cv2.IMREAD_COLOR)
        except Exception as e:
            self.get_logger().error(f'Error decoding compressed image: {e}')
            return

        detections = self.detect(src)
        self.update(self.seen_classes(detections, src.shape[0]))

        if self.debugPublisher is not None or self.show:
            img = self.draw(src, detections)
            if self.debugPublisher is not None:
                debug = CompressedImage()
                debug.header = msg.header
                debug.format = 'jpeg'
                debug.data = cv2.imencode('.jpg', img)[1].tobytes()
                self.debugPublisher.publish(debug)
            if self.show:
                cv2.imshow('mission', img)
                cv2.waitKey(1)

    def draw(self, src, detections):
        img = src.copy()
        min_size = self.min_box_size * img.shape[0]
        for c, p, x1, y1, x2, y2 in detections:
            color = (0, 255, 0) if max(x2 - x1, y2 - y1) >= min_size else (0, 160, 255)   # orange: too far to count
            cv2.rectangle(img, (int(x1), int(y1)), (int(x2), int(y2)), color, 2)
            cv2.putText(img, f'{CLASSES[c]} {p:.2f}', (int(x1), max(int(y1) - 5, 12)),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, color, 1)
        active = [pub.topic_name for pub, a in zip(self.signPublishers, self.active) if a]
        cv2.putText(img, 'active: ' + (' '.join(active) or '-'), (8, 24), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 0, 255), 2)
        return img


def main():
    rclpy.init()
    node = MissionDetection()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Keyboard Interrupt (SIGINT)')
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
