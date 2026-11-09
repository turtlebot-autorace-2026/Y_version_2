'''
Fake rclpy / message packages / ament_index / ultralytics, enough to construct every node of this
package without ROS and record what it declares: parameters (name, default, type), subscriptions
and publishers (topic, message type, QoS), timers. test_wiring.py cross-checks them.

Galactic-faithful where it matters for wiring:
  - a parameter declared with a default of one type and set from YAML with another is rejected
  - a parameter set in YAML but never declared is kept and reported (Galactic drops it silently
    unless allow_undeclared_parameters: the node then runs with its default, which is the bug)
  - ints in YAML for a double parameter are rejected (ROS 2 does not coerce 1 -> 1.0)
'''
import sys
import types


class _Rec(object):
    '''Per-node record.'''

    def __init__(self):
        self.params = {}          # name -> (default, overridden value or None)
        self.subs = []            # (topic, msgtype, qos)
        self.pubs = []            # (topic, msgtype, qos)
        self.timers = []          # period s
        self.errors = []


NODES = {}                        # node name -> _Rec
OVERRIDES = {}                    # node name -> {param: value} (from a YAML file)
SHARE = {}                        # package -> share dir


def _mod(name, **attrs):
    m = types.ModuleType(name)
    for k, v in attrs.items():
        setattr(m, k, v)
    sys.modules[name] = m
    return m


def _msg(pkg, names):
    m = _mod(pkg)
    sub = _mod(pkg + '.msg')
    m.msg = sub
    for n in names:
        def make(name=n, pkg=pkg):
            class M(object):
                __name__ = name
                _type = pkg + '/msg/' + name

                def __init__(self, **kw):
                    self.data = kw.get('data')
                    self.header = types.SimpleNamespace(stamp=types.SimpleNamespace(sec=0, nanosec=0), frame_id='')
                    self.linear = types.SimpleNamespace(x=0.0, y=0.0, z=0.0)
                    self.angular = types.SimpleNamespace(x=0.0, y=0.0, z=0.0)
                    for k, v in kw.items():
                        setattr(self, k, v)
            M.__name__ = name
            M.__qualname__ = name
            return M
        setattr(sub, n, make())
    return sub


def install(overrides=None, share=None):
    OVERRIDES.clear()
    OVERRIDES.update(overrides or {})
    SHARE.clear()
    SHARE.update(share or {})
    NODES.clear()

    # ---- rclpy
    rclpy = _mod('rclpy')
    rclpy.init = lambda *a, **k: None
    rclpy.shutdown = lambda *a, **k: None
    rclpy.ok = lambda: True
    rclpy.spin = lambda node: None

    class Duration(object):
        def __init__(self, seconds=0.0, nanoseconds=0):
            self.nanoseconds = int(seconds * 1e9) + nanoseconds

    class Time(object):
        def __init__(self, nanoseconds=0):
            self.nanoseconds = nanoseconds

        def to_msg(self):
            return types.SimpleNamespace(sec=0, nanosec=0)

    class Clock(object):
        def now(self):
            return Time(0)

    class Param(object):
        def __init__(self, value):
            self.value = value

    class Node(object):
        def __init__(self, name):
            self._name = name
            self.rec = _Rec()
            NODES[name] = self.rec
            self._ov = OVERRIDES.get(name, {})
            self._clock = Clock()

        def get_name(self):
            return self._name

        def declare_parameter(self, name, value=None, descriptor=None):
            if name in self.rec.params:
                self.rec.errors.append('parameter %s declared twice' % name)
            ov = self._ov.get(name, _MISSING)
            if ov is not _MISSING and value is not None:
                if type(ov) is not type(value) and not (isinstance(value, list) and isinstance(ov, list)):
                    self.rec.errors.append('parameter %s: YAML %r (%s) does not match declared default %r (%s)'
                                           % (name, ov, type(ov).__name__, value, type(value).__name__))
                elif isinstance(value, list) and value and ov and type(ov[0]) is not type(value[0]):
                    self.rec.errors.append('parameter %s: YAML list of %s, declared list of %s'
                                           % (name, type(ov[0]).__name__, type(value[0]).__name__))
            self.rec.params[name] = (value, None if ov is _MISSING else ov)
            return Param(value if ov is _MISSING else ov)

        def get_parameter(self, name):
            if name not in self.rec.params:
                raise KeyError('get_parameter(%s) before declare_parameter' % name)
            d, ov = self.rec.params[name]
            return Param(d if ov is None else ov)

        def create_subscription(self, msgtype, topic, cb, qos, **kw):
            self.rec.subs.append((topic, getattr(msgtype, '_type', str(msgtype)), _qos_name(qos)))
            return object()

        def create_publisher(self, msgtype, topic, qos, **kw):
            self.rec.pubs.append((topic, getattr(msgtype, '_type', str(msgtype)), _qos_name(qos)))
            return types.SimpleNamespace(publish=lambda m: None, topic_name=topic)

        def create_timer(self, period, cb, **kw):
            self.rec.timers.append(period)
            return object()

        def get_clock(self):
            return self._clock

        def get_logger(self):
            return types.SimpleNamespace(info=lambda *a, **k: None, warn=lambda *a, **k: None,
                                         warning=lambda *a, **k: None, error=lambda *a, **k: None,
                                         debug=lambda *a, **k: None)

        def destroy_node(self):
            pass

    _mod('rclpy.node', Node=Node)
    _mod('rclpy.duration', Duration=Duration)
    _mod('rclpy.time', Time=Time)
    _mod('rclpy.clock', Clock=Clock)

    class QoSProfile(object):
        def __init__(self, depth=10, durability=None, reliability=None, history=None):
            self.depth, self.durability, self.reliability = depth, durability, reliability

    class DurabilityPolicy(object):
        TRANSIENT_LOCAL = 'transient_local'
        VOLATILE = 'volatile'

    class ReliabilityPolicy(object):
        BEST_EFFORT = 'best_effort'
        RELIABLE = 'reliable'
    sensor = QoSProfile(depth=5, reliability=ReliabilityPolicy.BEST_EFFORT)
    sensor._name = 'sensor_data'
    _mod('rclpy.qos', QoSProfile=QoSProfile, DurabilityPolicy=DurabilityPolicy, ReliabilityPolicy=ReliabilityPolicy,
         qos_profile_sensor_data=sensor)
    _mod('rclpy.action', ActionClient=object)

    class ParameterDescriptor(object):
        def __init__(self, **kw):
            pass
    _mod('rcl_interfaces')
    _mod('rcl_interfaces.msg', ParameterDescriptor=ParameterDescriptor)

    # ---- messages
    _msg('std_msgs', ['Bool', 'String', 'Float32', 'Float32MultiArray', 'Int32'])
    _msg('sensor_msgs', ['CompressedImage', 'Image', 'LaserScan'])
    _msg('geometry_msgs', ['Twist', 'PoseStamped', 'PoseWithCovarianceStamped'])
    _msg('nav_msgs', ['Odometry'])

    # ---- ament_index
    def share_dir(pkg):
        if pkg not in SHARE:
            raise PackageNotFoundError(pkg)
        return SHARE[pkg]

    class PackageNotFoundError(KeyError):
        pass
    _mod('ament_index_python')
    _mod('ament_index_python.packages', get_package_share_directory=share_dir,
         get_package_prefix=share_dir, PackageNotFoundError=PackageNotFoundError)

    # ---- ultralytics
    class YOLO(object):
        def __init__(self, path, task=None):
            import os
            if not os.path.exists(path):
                raise FileNotFoundError(path)
            self.path = path
            self.names = {i: 'c%d' % i for i in range(8)}

        def predict(self, *a, **k):
            return [types.SimpleNamespace(masks=None, boxes=None)]
    _mod('ultralytics', YOLO=YOLO)


_MISSING = object()


def _qos_name(q):
    if isinstance(q, int):
        return 'depth%d' % q
    if getattr(q, '_name', None):
        return q._name
    if getattr(q, 'durability', None) == 'transient_local':
        return 'latched'
    return 'depth%d' % getattr(q, 'depth', 10)
