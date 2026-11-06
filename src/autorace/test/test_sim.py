'''Closed-loop offline runs on the Gazebo map geometry (autorace_gazebo v014 world file):
tunnel walls/cylinders, parking lot, construction boxes. The lane node is replaced by a pure
pursuit on the lane centre line, so these check the mission logic and manoeuvres, not lane tracking.'''
import math
import os
import sys

import numpy as np
import pytest

sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..'))
sys.path.insert(0, os.path.dirname(__file__))
from autorace.mission.missions import Construction, Ctx, Outputs, Parking, Tunnel  # noqa: E402
from autorace.mission.signs import SignBoard  # noqa: E402
from simlib import World, follow_polyline, integrate  # noqa: E402

DT = 0.1
ROBOT_R = 0.10     # m, footprint radius used for clearance (wheels at +-0.099 m)


def run(mission, world, pose, lane_path, lane_status=lambda pose: 'both', steps=3000, latency_steps=1,
        on_step=None, lane_centre=lambda pose: None, after_steps=0):
    '''Runs one mission. drive=lane -> pure pursuit on lane_path(pose, out); drive=mission -> out.cmd.'''
    signs = SignBoard()
    pose, travel, t, seq = list(pose), 0.0, 0.0, 0
    queue = [(0.0, 0.0)] * latency_steps
    trace, clear = [], []
    mission.start(Ctx(t=t, x=pose[0], y=pose[1], yaw=pose[2], travel=travel, signs=signs, scan=world.scan(pose)))
    for _ in range(steps):
        seq += 1
        ctx = Ctx(t=t, x=pose[0], y=pose[1], yaw=pose[2], travel=travel, signs=signs,
                  lane_status=lane_status(pose), lane_seq=seq, scan=world.scan(pose), lane_centre=lane_centre(pose))
        out = mission.step(ctx) if not mission.done else Outputs()     # FSM: plain lane once done
        if on_step:
            on_step(pose, out, mission)
        if mission.done:
            if after_steps <= 0:
                break
            after_steps -= 1
        if out.drive == 'lane':
            cmd = follow_polyline(pose, lane_path(pose, out, mission))
        elif out.drive == 'mission':
            cmd = out.cmd
        else:
            cmd = (0.0, 0.0)
        queue.append(cmd)
        v, w = queue.pop(0)
        old = pose
        pose = integrate(pose, v, w, DT)
        travel += math.hypot(pose[0] - old[0], pose[1] - old[1])
        t += DT
        trace.append(tuple(pose))
        clear.append(world.clearance(pose[0], pose[1], ROBOT_R))
    return pose, np.array(trace), np.array(clear)


# ---------------------------------------------------------------- tunnel

def tunnel_world(extra=()):
    w = World()
    # tunnel_wall model (-1.105, -1.186) + links, 0.05 m thick, from autorace_v014_full.world
    w.add_box(-2.228, -1.129, 2.06167, 0.05, math.pi / 2)    # Wall_1 (left side)
    w.add_box(-1.105, -2.257, 2.06167, 0.05)                 # Wall_2 (bottom)
    w.add_box(-0.024, -0.976, 1.76043, 0.05, math.pi / 2)    # Wall_3 (right side, exit gap below)
    w.add_box(-0.933, -0.138, 1.80969, 0.05)                 # Wall_4 (top, entrance gap left)
    for c in ((-0.708, -1.533), (-1.564, -1.620), (-1.307, -0.718)) + tuple(extra):
        w.add_circle(c[0], c[1], 0.1)
    return w


def tunnel_lane_status(pose):
    x, y, yaw = pose
    if -2.14 < x < -1.86 and y > 0.12:                       # road before the entrance
        return 'both'
    if x > -0.30 and -2.15 < y < -1.87 and abs(yaw) < 0.6:   # start straight lines in view at the exit
        return 'both'
    return 'lost'


def tunnel_lane_path(pose, out, mission):
    if pose[1] > -1.0:
        return [(-1.995, 0.6), (-1.995, -0.5)]               # entrance road (lines end at y ~0.02)
    return [(-0.4, -2.01), (1.5, -2.01)]                     # start straight


@pytest.mark.parametrize('extra', [(), ((-2.02, -1.05),), ((-1.2, -2.05),)],
                         ids=['map', 'obstacle_on_left_wall_path', 'obstacle_on_bottom_wall_path'])
def test_tunnel_reaches_exit_without_contact(extra):
    world = tunnel_world(extra)
    m = Tunnel()
    pose, trace, clear = run(m, world, (-1.995, 0.35, -math.pi / 2), tunnel_lane_path,
                             tunnel_lane_status, steps=2500)
    assert m.done and m.result == 'ok', (m.phase, pose)
    assert pose[0] > -0.2 and -2.15 < pose[1] < -1.87, pose
    assert clear.min() > 0.0, clear.min()


class TunnelBox(object):
    '''Tunnel box like the Gazebo one at any size: inner faces x ox..ox+W, y oy-H..oy; entrance gap in
    the top wall at the left (robot comes in heading -y), exit gap low in the right wall.'''

    def __init__(self, W, H, ox, oy, entry_off=0.208, gap_in=0.365, exit_off=0.222, gap_out=0.376, t=0.05):
        self.ex, self.ey, self.oy = ox + entry_off, oy - H + exit_off, oy
        self.x1 = ox + W
        self.walls = [((ox - t / 2, oy - H / 2), (t, H + 2 * t)),
                      ((ox + W / 2, oy - H - t / 2), (W + 2 * t, t)),
                      ((ox + W + t / 2, (oy - H + gap_out + oy + t) / 2), (t, H + t - gap_out)),
                      (((ox + gap_in + ox + W + t) / 2, oy + t / 2), (W + t - gap_in, t))]
        self.exit_fwd = (oy + 0.163) - self.ey     # tunnel frame: walls on both sides 0.163 m before the inner face
        self.exit_left = self.x1 - self.ex

    def world(self, obs):
        w = World()
        for (cx, cy), (sx, sy) in self.walls:
            w.add_box(cx, cy, sx, sy)
        for o in obs:
            (w.add_circle if o[0] == 'c' else w.add_box)(*o[1:])
        return w

    def lane_status(self, pose):
        x, y, yaw = pose
        if abs(x - self.ex) < 0.14 and y > self.oy + 0.12:
            return 'both'
        if x > self.x1 - 0.25 and abs(y - self.ey) < 0.14 and abs(yaw) < 0.6:
            return 'both'
        return 'lost'

    def lane_path(self, pose, out, mission):
        if pose[1] > self.oy - 0.5 and pose[0] < self.ex + 0.4:
            return [(self.ex, self.oy + 0.8), (self.ex, self.oy - 0.7)]
        return [(self.x1 - 0.4, self.ey), (self.x1 + 2.0, self.ey)]


GAZEBO_TUNNEL = TunnelBox(2.154, 2.069, -2.203, -0.163)      # = tunnel_world()
RULE_TUNNEL = TunnelBox(1.8, 1.8, 0.0, 0.0)                  # rulebook interior 1.8 x 1.8 m

# Random passable layouts where steering toward the exit at each tick (no map) got stuck in a pocket
# facing the exit. The previous right-hand wall following did not get out of two of them either
# (gazebo_box_before_exit, rule_boxes_both_sides: circling / contact).
POCKETS = {
    'gazebo_pocket_at_entrance': (GAZEBO_TUNNEL, [('c', -1.58, -0.79, 0.17), ('b', -1.34, -0.41, 0.19, 0.18, 1.5),
                                                  ('c', -1.83, -0.94, 0.05), ('b', -0.28, -1.54, 0.23, 0.19, 0.2)]),
    'gazebo_box_before_exit': (GAZEBO_TUNNEL, [('c', -0.84, -1.5, 0.11), ('b', -0.47, -1.82, 0.22, 0.25, 0.45)]),
    'rule_four_cylinders': (RULE_TUNNEL, [('c', 1.0, -0.6, 0.16), ('c', 1.0, -1.2, 0.16), ('c', 1.32, -1.03, 0.09),
                                          ('c', 1.28, -0.38, 0.13)]),
    'rule_boxes_both_sides': (RULE_TUNNEL, [('b', 0.37, -0.46, 0.37, 0.34, 2.1), ('c', 1.23, -0.81, 0.13),
                                            ('b', 0.21, -1.4, 0.11, 0.13, 0.31), ('b', 1.14, -0.43, 0.27, 0.34, 0.11)]),
}


@pytest.mark.parametrize('name', sorted(POCKETS))
def test_tunnel_plans_round_pockets(name):
    tun, obs = POCKETS[name]
    m = Tunnel({'exit_fwd': tun.exit_fwd, 'exit_left': tun.exit_left})
    pose, trace, clear = run(m, tun.world(obs), (tun.ex, tun.oy + 0.5, -math.pi / 2), tun.lane_path,
                             tun.lane_status, steps=1500)
    assert m.done and m.result == 'ok', (m.phase, pose)
    assert pose[0] > tun.x1 - 0.25 and abs(pose[1] - tun.ey) < 0.14, pose
    assert clear.min() > 0.0, clear.min()


def test_tunnel_exit_parameters_off_by_20cm():
    '''The grid finds the opening itself: exit_fwd / exit_left 0.2 m off still gets out.'''
    tun = GAZEBO_TUNNEL
    for df, dl in ((0.2, -0.2), (-0.2, 0.2)):
        m = Tunnel({'exit_fwd': tun.exit_fwd + df, 'exit_left': tun.exit_left + dl})
        pose, trace, clear = run(m, tunnel_world(), (tun.ex, tun.oy + 0.5, -math.pi / 2), tun.lane_path,
                                 tun.lane_status, steps=1500)
        assert m.done and pose[0] > tun.x1 - 0.25 and abs(pose[1] - tun.ey) < 0.14, (df, dl, m.phase, pose)
        assert clear.min() > 0.0, (df, dl, clear.min())


# ---------------------------------------------------------------- parking

LOT = {'corridor_x': 0.577, 'left_x': 0.863, 'right_x': 0.295, 'row_y': 0.864, 'road_y': 2.006}


def parking_world(house):
    w = World()
    if house:
        w.add_box(LOT[house + '_x'], LOT['row_y'], 0.14, 0.18)     # TurtleBot-sized house robot
    return w


def parking_lane_path(pose, out, mission):
    cx, ry = LOT['corridor_x'], LOT['road_y']
    if mission.phase in ('to_spur', 'spur'):    # bottom road heading -x, turn left into the spur
        arc = [(cx + 0.2 - 0.2 * math.sin(a), ry - 0.2 + 0.2 * math.cos(a)) for a in np.linspace(0, math.pi / 2, 8)]
        return [(1.6, ry)] + arc + [(cx, 0.4)]
    arc = [(cx - 0.2 + 0.2 * math.cos(a), ry - 0.2 + 0.2 * math.sin(a)) for a in np.linspace(0, math.pi / 2, 8)]
    return [(cx, 0.9)] + arc + [(-1.0, ry)]   # spur heading +y, turn left onto the road heading -x


@pytest.mark.parametrize('house,expect', [(None, 'left'), ('left', 'right'), ('right', 'left')])
def test_parking_free_stall(house, expect):
    world = parking_world(house)
    m = Parking()
    stall = []

    def watch(pose, out, mission):
        if mission.phase == 'parked':
            stall.append(tuple(pose))
    pose, trace, clear = run(m, world, (1.5, LOT['road_y'], math.pi), parking_lane_path, on_step=watch)
    assert m.done and m.result == 'ok', m.phase
    assert m.side == expect
    sx, sy, syaw = stall[0]
    # fully in the stall: footprint (0.10 m behind / 0.07 m ahead of the axle, +-0.10 m wide) inside the
    # stall lines (Gazebo texture: columns 1.004 | 0.722 | 0.436 | 0.154, rows 0.573 .. 1.155, lines 26 mm)
    assert abs(abs(syaw) - (0.0 if expect == 'left' else math.pi)) < 0.1, syaw
    head = 1.0 if expect == 'left' else -1.0
    xs = sorted((sx - 0.10 * head, sx + 0.07 * head))
    lo, hi = (0.722, 1.004) if expect == 'left' else (0.154, 0.436)
    assert lo + 0.013 < xs[0] and xs[1] < hi - 0.013, (stall[0], xs)
    assert 0.573 + 0.013 < sy - 0.10 and sy + 0.10 < 1.155 - 0.013, stall[0]
    if house:
        assert clear.min() > 0.0, clear.min()
    assert pose[1] > 1.85 and abs(math.atan2(math.sin(pose[2] - math.pi), math.cos(pose[2] - math.pi))) < 0.35, pose


# ---------------------------------------------------------------- construction
# Lines measured on the course texture (scaled to v014): road heading +x, sharp yellow corner /
# wide white arc into the double-width section (heading +y), then sharp yellow corner / wide white
# arc onto the bottom road (heading -x).

LANE_OFF = 0.143


def _arc(cx, cy, r, a0, a1, n=24):
    return [(cx + r * math.cos(a), cy + r * math.sin(a)) for a in np.linspace(a0, a1, n)]


YELLOW = [(0.9, 0.434)] + _arc(1.536, 0.484, 0.05, -math.pi / 2, 0.0) + _arc(1.536, 1.811, 0.05, 0.0, math.pi / 2) \
    + [(0.6, 1.861)]
WHITE = [(0.9, 0.144)] + _arc(1.739, 0.561, 0.417, -math.pi / 2, 0.0) + _arc(1.733, 1.727, 0.420, 0.0, math.pi / 2) \
    + [(0.6, 2.147)]
BOXES = ((1.707, 0.619), (1.993, 1.145), (1.707, 1.677))      # barrier_1..3, 0.25 x 0.1 m


def _dense(poly, step=0.01):
    p = np.asarray(poly, float)
    out = [p[0]]
    for a, b in zip(p[:-1], p[1:]):
        n = max(1, int(np.hypot(*(b - a)) / step))
        out.extend(a + (b - a) * k / n for k in range(1, n + 1))
    return np.array(out)


def _shift(poly, dist):
    '''Polyline shifted to its left (driving direction) by dist (negative = right).'''
    p = _dense(poly)
    t = np.gradient(p, axis=0)
    t /= np.linalg.norm(t, axis=1, keepdims=True)
    return p + dist * np.c_[-t[:, 1], t[:, 0]]


def _poly_dist(poly, x, y):
    p = _dense(poly)
    return float(np.hypot(p[:, 0] - x, p[:, 1] - y).min())


PATHS = {'yellow': _shift(YELLOW, -LANE_OFF), 'white': _shift(WHITE, LANE_OFF)}
PATHS['both'] = PATHS['yellow']


def construction_world():
    w = World()
    for cx, cy in BOXES:
        w.add_box(cx, cy, 0.25, 0.10)
    return w


class LineLane(object):
    '''Stand-in for the lane node: follows the centre path of the commanded line and reports
    /lane/centre for it.'''

    def __init__(self):
        self.follow = 'both'

    def __call__(self, pose, out, mission):
        self.follow = out.follow_line
        return PATHS[self.follow]

    def centre(self, pose):
        c = PATHS[self.follow]
        x, y, yaw = pose
        i = int(np.argmin(np.hypot(c[:, 0] - x, c[:, 1] - y)))
        i = min(i, len(c) - 2)
        th = math.atan2(c[i + 1, 1] - c[i, 1], c[i + 1, 0] - c[i, 0])
        lat = -math.sin(th) * (c[i, 0] - x) + math.cos(th) * (c[i, 1] - y)     # + = centre left of robot
        return lat, math.atan2(math.sin(th - yaw), math.cos(th - yaw))


def test_construction_slalom():
    world = construction_world()
    m = Construction()
    lane = LineLane()
    pose, trace, clear = run(m, world, (1.0, 0.291, 0.0), lane, steps=900, lane_centre=lane.centre, after_steps=60)
    assert m.done and m.result == 'ok', m.phase
    assert m.changes == 2, m.changes                         # outer -> inner (box 2) -> outer (box 3)
    # touching is allowed by the rules, but the plan should clear the boxes
    assert clear.min() > 0.0, clear.min()
    # never across a line (robot edge vs painted line edge), including both corners
    edge = ROBOT_R + 0.013
    dy = min(_poly_dist(YELLOW, x, y) for x, y, _ in trace)
    dw = min(_poly_dist(WHITE, x, y) for x, y, _ in trace)
    assert dy > edge and dw > edge, (dy, dw)
    # back on the normal lane centre of the bottom road, heading -x
    assert abs(pose[1] - 2.004) < 0.03 and pose[0] < 1.4, pose
