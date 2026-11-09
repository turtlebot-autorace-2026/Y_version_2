'''
cmd_vel_mux node: only the source chosen by mission_fsm reaches the robot.

  sub  <lane_topic>     geometry_msgs/Twist   lane node   (default /cmd_vel_lane)
  sub  <mission_topic>  geometry_msgs/Twist   FSM's own manoeuvres (default /cmd_vel_mission)
  sub  <mode_topic>     std_msgs/String       lane | mission | stop (default /mission/drive)
  pub  <out_topic>      geometry_msgs/Twist   to the robot (default cmd_vel)

Do not run teleop at the same time: while stopped the mux keeps sending zero commands.
'''
import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node
from std_msgs.msg import String

from autorace.mission.mux_core import CmdVelMux


def twist(v, w):
    msg = Twist()
    msg.linear.x = float(v)
    msg.angular.z = float(w)
    return msg


class CmdVelMuxNode(Node):
    def __init__(self):
        super().__init__('cmd_vel_mux')
        p = self.declare_parameter
        p('lane_topic', '/cmd_vel_lane')
        p('mission_topic', '/cmd_vel_mission')
        p('mode_topic', '/mission/drive')
        p('out_topic', 'cmd_vel')
        p('source_timeout', 0.5)
        p('mode_timeout', 1.0)
        p('tick_hz', 20.0)
        p('accel', 0.3)               # m/s^2 start ramp after any stop (0 = off)
        g = lambda k: self.get_parameter(k).value  # noqa: E731
        self.mux = CmdVelMux(g('source_timeout'), g('mode_timeout'), g('accel'))
        self.pub = self.create_publisher(Twist, g('out_topic'), 10)
        self.create_subscription(Twist, g('lane_topic'), lambda m: self.on_cmd('lane', m), 10)
        self.create_subscription(Twist, g('mission_topic'), lambda m: self.on_cmd('mission', m), 10)
        self.create_subscription(String, g('mode_topic'), self.on_mode, 10)
        self.reason = None
        self.create_timer(1.0 / g('tick_hz'), self.tick)
        self.get_logger().info('cmd_vel_mux: %s / %s -> %s (mode %s)'
                               % (g('lane_topic'), g('mission_topic'), g('out_topic'), g('mode_topic')))

    def now(self):
        return self.get_clock().now().nanoseconds * 1e-9

    def on_mode(self, msg):
        try:
            if self.mux.set_mode(msg.data.strip(), self.now()):
                self.get_logger().info('drive mode -> %s' % self.mux.mode)
        except ValueError as e:
            self.get_logger().warn(str(e))

    def on_cmd(self, source, msg):
        out = self.mux.on_cmd(source, msg.linear.x, msg.angular.z, self.now())
        if out is not None:
            self.pub.publish(twist(*out))

    def tick(self):
        cmd, reason = self.mux.tick(self.now())
        if cmd is not None:
            self.pub.publish(twist(*cmd))
        if reason != self.reason:
            (self.get_logger().info if reason == 'ok' else self.get_logger().warn)('mux: %s' % reason)
            self.reason = reason


def main():
    rclpy.init()
    node = CmdVelMuxNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        for _ in range(3):
            try:
                node.pub.publish(Twist())
            except Exception:
                break
        try:
            node.destroy_node()
            rclpy.shutdown()
        except Exception:
            pass


if __name__ == '__main__':
    main()
