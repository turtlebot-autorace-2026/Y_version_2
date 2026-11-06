'''
mission_fsm node: the only place that decides what the robot is doing.

  sub  /parking /left /right /no_entry /stop /construction /intersection /tunnel
                         std_msgs/Bool, latched, from the senior's mission_model (sign_topics param)
  sub  /odom             nav_msgs/Odometry
  sub  /scan             sensor_msgs/LaserScan
  sub  /lane/status      std_msgs/String   lane node mode (both, yellow_only, ..., lost)
  sub  /lane/centre      std_msgs/Float32MultiArray  [lateral m (+ left), heading rad] of the lane centre
  sub  /camera2          sensor_msgs/CompressedImage   front camera: traffic light, barrier
  pub  /lane_state       std_msgs/Bool     lane node on (a reset sends False for one tick)
  pub  /lane/follow_line std_msgs/String   both | yellow | white
  pub  /lane/offset      std_msgs/Float32  m, + = left
  pub  /intersection_state std_msgs/Bool   lane node slow-down (v_max_intersection)
  pub  /mission/drive    std_msgs/String   lane | mission | stop -> cmd_vel_mux
  pub  /cmd_vel_mission  geometry_msgs/Twist   own manoeuvres (parking, tunnel)
  pub  /mission/state    std_msgs/String   status line
  pub  /mission/debug/compressed  front image with the light / bar check (only while used)
'''
import cv2
import numpy as np
import rclpy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, qos_profile_sensor_data
from sensor_msgs.msg import CompressedImage, LaserScan
from std_msgs.msg import Bool, Float32, Float32MultiArray, String

from autorace.mission import vision
from autorace.mission.fsm_core import CLASSES, DEFAULTS, MissionFSM
from autorace.mission.lidar import Scan
from autorace.mission.missions import Ctx, Tunnel
from autorace.mission.gridnav import GRID_DEFAULTS
from autorace.mission.motion import GAP_DEFAULTS, WALL_DEFAULTS
from autorace.mission.signs import SIGN_TOPICS, SIGNS, SignBoard


def yaw_of(q):
    return float(np.arctan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z)))


class MissionFsmNode(Node):
    def __init__(self):
        super().__init__('mission_fsm')
        p = self.declare_parameter
        p('sign_topics', [t for _, t in SIGN_TOPICS])
        p('odom_topic', '/odom')
        p('scan_topic', '/scan')
        p('image_topic', '/camera2')
        p('lidar_x', -0.032)
        p('rate_hz', 10.0)
        p('status_period', 1.0)
        p('publish_debug', True)
        for k, v in DEFAULTS.items():
            p(k, v)
        g = lambda k: self.get_parameter(k).value  # noqa: E731
        mission_params = {}
        for name, cls in CLASSES.items():
            d = dict(cls.DEFAULTS)
            if cls is Tunnel:
                d.update(('wall_' + k, v) for k, v in WALL_DEFAULTS.items())
                d.update(('gap_' + k, v) for k, v in GAP_DEFAULTS.items())
                d.update(('grid_' + k, v) for k, v in GRID_DEFAULTS.items())
            for k, v in d.items():
                p('%s.%s' % (name, k), v)
            mission_params[name] = dict((k, g('%s.%s' % (name, k))) for k in d)
        for k, v in vision.LIGHT_DEFAULTS.items():
            p('light.' + k, v)
        for k, v in vision.BAR_DEFAULTS.items():
            p('bar.' + k, v)
        self.light_p = dict((k, g('light.' + k)) for k in vision.LIGHT_DEFAULTS)
        self.bar_p = dict((k, g('bar.' + k)) for k in vision.BAR_DEFAULTS)
        self.fsm = MissionFSM(dict((k, g(k)) for k in DEFAULTS), mission_params)

        self.signs = SignBoard()
        self.travel, self.last_xy, self.pose = 0.0, None, (0.0, 0.0, 0.0)
        self.scan, self.lane_status, self.lane_seq = None, '', 0
        self.lane_centre, self.lane_centre_t = None, -1e9
        self.image, self.image_seq, self.done_seq = None, 0, 0
        self.light, self.bar, self.vis_seq, self.vis_info = None, None, 0, None
        self.lidar_x = g('lidar_x')

        topics = list(g('sign_topics'))
        if len(topics) != len(SIGNS):
            raise ValueError('sign_topics needs %d names in the order %s' % (len(SIGNS), list(SIGNS)))
        latched = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        for name, topic in zip(SIGNS, topics):
            self.create_subscription(Bool, topic, lambda m, n=name: self.on_sign(n, m), latched)
        self.create_subscription(Odometry, g('odom_topic'), self.on_odom, 10)
        self.create_subscription(LaserScan, g('scan_topic'), self.on_scan, qos_profile_sensor_data)
        self.create_subscription(String, '/lane/status', self.on_lane_status, 10)
        self.create_subscription(Float32MultiArray, '/lane/centre', self.on_lane_centre, 10)
        self.create_subscription(CompressedImage, g('image_topic'), self.on_image, qos_profile_sensor_data)

        self.pub_lane_state = self.create_publisher(Bool, '/lane_state', 10)
        self.pub_follow = self.create_publisher(String, '/lane/follow_line', 10)
        self.pub_offset = self.create_publisher(Float32, '/lane/offset', 10)
        self.pub_slow = self.create_publisher(Bool, '/intersection_state', 10)
        self.pub_drive = self.create_publisher(String, '/mission/drive', 10)
        self.pub_cmd = self.create_publisher(Twist, '/cmd_vel_mission', 10)
        self.pub_state = self.create_publisher(String, '/mission/state', 10)
        self.pub_debug = self.create_publisher(CompressedImage, '/mission/debug/compressed', 2) \
            if g('publish_debug') else None

        self.last_status_t, self.last_debug_t = -1e9, -1e9
        self.status_period = g('status_period')
        self.create_timer(1.0 / g('rate_hz'), self.tick)
        self.get_logger().info('mission_fsm: start_mode=%s missions=%s signs=%s'
                               % (g('start_mode'), list(g('missions')), dict(zip(SIGNS, topics))))

    def now(self):
        return self.get_clock().now().nanoseconds * 1e-9

    # ---- inputs ----------------------------------------------------------------------

    def on_sign(self, name, msg):
        self.signs.set(name, msg.data, self.now(), self.travel)
        self.get_logger().info('sign %s %s' % (name, msg.data))

    def on_odom(self, msg):
        pos = msg.pose.pose.position
        if self.last_xy is not None:
            self.travel += float(np.hypot(pos.x - self.last_xy[0], pos.y - self.last_xy[1]))
        self.last_xy = (pos.x, pos.y)
        self.pose = (pos.x, pos.y, yaw_of(msg.pose.pose.orientation))

    def on_scan(self, msg):
        self.scan = Scan(msg.ranges, msg.angle_min, msg.angle_increment, msg.range_min, msg.range_max,
                         self.lidar_x)

    def on_lane_status(self, msg):
        self.lane_status, self.lane_seq = msg.data, self.lane_seq + 1

    def on_lane_centre(self, msg):
        d = list(msg.data)
        ok = len(d) >= 2 and all(np.isfinite(d[:2]))
        self.lane_centre = (float(d[0]), float(d[1])) if ok else None
        self.lane_centre_t = self.now()

    def on_image(self, msg):
        self.image, self.image_seq = msg, self.image_seq + 1

    def run_vision(self, kind, t):
        '''Classify the newest camera frame once (only for the check the active mission needs).'''
        if kind is None or self.image is None or self.image_seq == self.done_seq:
            return
        self.done_seq = self.image_seq
        img = cv2.imdecode(np.frombuffer(self.image.data, np.uint8), cv2.IMREAD_COLOR)
        if img is None:
            return
        self.vis_seq += 1
        if kind == 'light':
            self.light, info = vision.classify_light(img, self.light_p)
            view = (lambda: vision.draw(img, light=self.light, light_info=info))
        else:
            self.bar, info = vision.bar_state(img, self.bar_p)
            view = (lambda: vision.draw(img, bar=self.bar, bar_info=info))
        if self.pub_debug is not None and t - self.last_debug_t >= 0.2:
            self.last_debug_t = t
            out = CompressedImage()
            out.header = self.image.header
            out.format = 'jpeg'
            out.data = cv2.imencode('.jpg', view())[1].tobytes()
            self.pub_debug.publish(out)

    # ---- main ------------------------------------------------------------------------

    def tick(self):
        t = self.now()
        self.signs.touch(self.travel)
        kind = self.fsm.vision()
        self.run_vision(kind, t)
        x, y, yaw = self.pose
        centre = self.lane_centre if t - self.lane_centre_t < 0.3 else None
        ctx = Ctx(t=t, x=x, y=y, yaw=yaw, travel=self.travel, signs=self.signs,
                  lane_status=self.lane_status, lane_seq=self.lane_seq, scan=self.scan,
                  light=self.light if kind == 'light' else None, light_seq=self.vis_seq,
                  bar=self.bar if kind == 'bar' else None, bar_seq=self.vis_seq, lane_centre=centre)
        out = self.fsm.step(ctx)
        for text in self.fsm.events:
            self.get_logger().info(text)
        self.fsm.events = []

        # reset: /lane_state False on this tick, True again on the next (the lane node acts on edges
        # and builds a fresh estimator on True)
        self.pub_lane_state.publish(Bool(data=not out.reset))
        self.pub_follow.publish(String(data=out.follow_line))
        self.pub_offset.publish(Float32(data=float(out.offset)))
        self.pub_slow.publish(Bool(data=out.slow))
        self.pub_drive.publish(String(data=out.drive))
        if out.drive == 'mission':
            cmd = Twist()
            cmd.linear.x, cmd.angular.z = float(out.cmd[0]), float(out.cmd[1])
            self.pub_cmd.publish(cmd)

        status = '%s | signs %s | lane %s | out %s' % (self.fsm.status(), self.signs.summary(),
                                                      self.lane_status or '-', out.as_dict())
        if kind is not None:
            status += ' | %s=%s' % (kind, self.light if kind == 'light' else self.bar)
        self.pub_state.publish(String(data=status))
        if t - self.last_status_t >= self.status_period:
            self.last_status_t = t
            self.get_logger().info(status)


def main():
    rclpy.init()
    node = MissionFsmNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        try:
            node.pub_drive.publish(String(data='stop'))
            node.destroy_node()
            rclpy.shutdown()
        except Exception:
            pass


if __name__ == '__main__':
    main()
