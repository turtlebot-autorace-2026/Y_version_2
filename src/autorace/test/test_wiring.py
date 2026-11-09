'''
Build every node with the fake ROS (fakeros.py), with the package's YAML parameter files applied as
Galactic would apply them, and check the wiring between them: every topic one node subscribes to is
published by another node of this package or by the robot / Gazebo, message types and QoS match,
YAML parameters all exist and have the declared type, and the launch files only set parameters that
exist. This is what a first Gazebo run would otherwise find.
'''
import importlib
import os
import sys

sys.dont_write_bytecode = True      # modules are re-imported per launch file; a stale .pyc would hide an edit

import pytest
import yaml

sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..'))
sys.path.insert(0, os.path.dirname(__file__))
import fakeros  # noqa: E402

PKG_DIR = os.path.join(os.path.dirname(__file__), '..')
SHARE = {'autorace': os.path.abspath(PKG_DIR)}

# topics that come from outside this package (robot / Gazebo / turtlebot3_bringup / mission_model)
EXTERNAL_PUBS = {
    '/odom': 'nav_msgs/msg/Odometry', '/scan': 'sensor_msgs/msg/LaserScan',
    '/camera/image_raw': 'sensor_msgs/msg/Image', '/front_camera/image_raw': 'sensor_msgs/msg/Image',
}
EXTERNAL_SUBS = {'/cmd_vel': 'geometry_msgs/msg/Twist'}    # the robot base
# the lane node's inputs that the FSM drives; the senior's lane_mode is kept for compatibility, nobody sends it
KNOWN_UNDRIVEN_SUBS = {'/lane_mode', '/lane/section'}


def abs_topic(t):
    '''Relative topic names resolve under the node's namespace; the launch files use the root.'''
    return t if t.startswith('/') else '/' + t


def yaml_params(path):
    with open(path) as f:
        d = yaml.safe_load(f)
    out = {}
    for node, body in d.items():
        flat = {}

        def walk(prefix, v):
            if isinstance(v, dict):
                for k, vv in v.items():
                    walk(prefix + k + '.', vv)
            else:
                flat[prefix[:-1]] = v
        walk('', body['ros__parameters'])
        out[node] = flat
    return out


import ast
import re

# executable -> module, as setup.py's console_scripts
EXECUTABLES = dict(re.findall(r"'(\w+) = (autorace\.[\w.]+):main'", open(os.path.join(PKG_DIR, 'setup.py')).read()))


def launch_nodes(launch_file):
    """(executable, node name, inline parameter dict) of every Node(package=PKG, ...) call in a launch
    file, read from the AST. Literal values are kept; anything computed (arg(...), float(arg(...)),
    show, prefix) becomes '<arg>'."""
    tree = ast.parse(open(os.path.join(PKG_DIR, 'launch', launch_file)).read())
    out = []
    for call in ast.walk(tree):
        if not (isinstance(call, ast.Call) and getattr(call.func, 'id', None) == 'Node'):
            continue
        kw = {k.arg: k.value for k in call.keywords if k.arg}
        if getattr(kw.get('package'), 'id', None) != 'PKG':
            continue
        exe, name = kw['executable'].value, kw['name'].value
        params = {}
        for elt in ast.walk(kw['parameters']):
            if isinstance(elt, ast.Dict):
                for k, v in zip(elt.keys, elt.values):
                    try:
                        params[ast.literal_eval(k)] = ast.literal_eval(v)
                    except Exception:
                        params[ast.literal_eval(k)] = '<arg>'
        out.append((exe, name, params))
    return out


def build_all(launch_file='robot.launch.py'):
    '''Build the nodes the way the launch file starts them: node name from the launch, YAML files
    first, then the launch's inline parameters on top (that is the ROS 2 precedence).'''
    overrides = {}
    for f in ('lane_params.yaml', 'mission_params.yaml', 'camera_params.yaml'):
        overrides.update(yaml_params(os.path.join(PKG_DIR, 'config', f)))
    plan = launch_nodes(launch_file)
    for exe, name, params in plan:
        overrides.setdefault(name, {}).update({k: v for k, v in params.items() if v != '<arg>'})
    fakeros.install(overrides, SHARE)
    for m in [m for m in sys.modules if m.startswith('autorace.')]:
        del sys.modules[m]
    NodeBase = sys.modules['rclpy.node'].Node
    nodes = {}
    for exe, name, params in plan:
        m = importlib.import_module(EXECUTABLES[exe])
        cls = [getattr(m, n) for n in dir(m) if isinstance(getattr(m, n), type)
               and issubclass(getattr(m, n), NodeBase) and getattr(m, n) is not NodeBase][0]
        # the launch's name= replaces the name the node gives itself
        real_init = NodeBase.__init__
        NodeBase.__init__ = lambda self, _n, _name=name: real_init(self, _name)
        try:
            node = cls()
        finally:
            NodeBase.__init__ = real_init
        nodes[name] = node
    return nodes, fakeros.NODES


@pytest.fixture(scope='module', params=['robot.launch.py', 'sim.launch.py'])
def built(request):
    return build_all(request.param)


def test_nodes_construct_and_yaml_types(built):
    nodes, recs = built
    errors = {n: r.errors for n, r in recs.items() if r.errors}
    assert not errors, errors


def test_yaml_parameters_all_declared(built):
    '''A YAML parameter the node never declares is silently ignored in Galactic: the node runs on its
    default and nobody notices.'''
    nodes, recs = built
    undeclared = {}
    for f in ('lane_params.yaml', 'mission_params.yaml', 'camera_params.yaml'):
        for node, ps in yaml_params(os.path.join(PKG_DIR, 'config', f)).items():
            if node not in recs:
                continue                      # not started by this launch (cameras in sim)
            missing = sorted(set(ps) - set(recs[node].params) - {'use_sim_time'})
            if missing:
                undeclared['%s (%s)' % (node, f)] = missing
    assert not undeclared, undeclared


def test_every_subscription_has_a_publisher(built):
    nodes, recs = built
    pubs = dict(EXTERNAL_PUBS)
    for n, r in recs.items():
        for topic, mt, _ in r.pubs:
            pubs[abs_topic(topic)] = mt
    problems = []
    for n, r in recs.items():
        for topic, mt, _ in r.subs:
            topic = abs_topic(topic)
            if topic in KNOWN_UNDRIVEN_SUBS:
                continue
            if topic not in pubs:
                problems.append('%s subscribes %s: nobody publishes it' % (n, topic))
            elif pubs[topic] != mt:
                problems.append('%s subscribes %s as %s, published as %s' % (n, topic, mt, pubs[topic]))
    assert not problems, problems


def test_every_publisher_has_a_reader(built):
    '''Not an error in ROS, but a topic nobody reads is dead code or a typo in a topic name.'''
    nodes, recs = built
    subs = dict(EXTERNAL_SUBS)
    for n, r in recs.items():
        for topic, mt, _ in r.subs:
            subs[abs_topic(topic)] = mt
    debug = ('/debug/', '/mission/state', 'camera_raw', '/lane/section')
    unread = ['%s publishes %s' % (n, t) for n, r in recs.items() for t, _, _ in r.pubs
              if abs_topic(t) not in subs and not any(d in t for d in debug)]
    assert not unread, unread


def test_qos_compatible(built):
    '''A latched (transient local) publisher needs a transient-local subscriber to deliver the last
    value to a late starter; a best-effort sensor publisher cannot be read by a reliable subscriber
    in some RMWs. Check the pairs inside the package.'''
    nodes, recs = built
    pub_qos = {}
    for n, r in recs.items():
        for topic, _, q in r.pubs:
            pub_qos[abs_topic(topic)] = (n, q)
    problems = []
    for n, r in recs.items():
        for topic, _, q in r.subs:
            topic = abs_topic(topic)
            if topic in pub_qos:
                pn, pq = pub_qos[topic]
                if pq == 'latched' and q != 'latched':
                    problems.append('%s publishes %s latched, %s subscribes %s (late start loses the state)' % (pn, topic, n, q))
    assert not problems, problems


def test_cmd_vel_chain(built):
    '''lane -> /cmd_vel_lane, FSM -> /cmd_vel_mission, mux -> cmd_vel; only the mux talks to the robot.'''
    nodes, recs = built
    twist_pubs = [(n, abs_topic(t)) for n, r in recs.items() for t, mt, _ in r.pubs if mt == 'geometry_msgs/msg/Twist']
    to_robot = [(n, t) for n, t in twist_pubs if t == '/cmd_vel']
    assert to_robot == [('cmd_vel_mux', '/cmd_vel')], twist_pubs
    assert ('lane_detection_ad', '/cmd_vel_lane') in twist_pubs, twist_pubs
    assert ('mission_fsm', '/cmd_vel_mission') in twist_pubs, twist_pubs


def test_launch_files_set_only_existing_parameters(built):
    '''Parameters the launch files set inline must exist on the node (else: silently ignored).'''
    nodes, recs = built
    problems = []
    for lf in ('robot.launch.py', 'sim.launch.py'):
        for exe, name, params in launch_nodes(lf):
            if name not in recs:
                continue
            for k in params:
                if k not in recs[name].params and k != 'use_sim_time':
                    problems.append('%s sets %s.%s: not a parameter of that node' % (lf, name, k))
    assert not problems, problems


def test_timers_match_documented_rates(built):
    nodes, recs = built
    assert any(abs(p - 0.1) < 1e-9 for p in recs['mission_fsm'].timers), recs['mission_fsm'].timers
    assert any(abs(p - 1 / 15.0) < 1e-9 for p in recs['lane_detection_ad'].timers), recs['lane_detection_ad'].timers
