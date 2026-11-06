'''Odometry motion primitives and the LiDAR wall follower, no ROS.'''
import math


def wrap(a):
    return math.atan2(math.sin(a), math.cos(a))


def clip(v, lo, hi):
    return max(lo, min(hi, v))


def turn_cmd(yaw, target, k=2.0, w_max=0.8, w_min=0.15, tol_deg=3.0):
    '''In-place turn. Returns ((v, w), done).'''
    e = wrap(target - yaw)
    if abs(e) < math.radians(tol_deg):
        return (0.0, 0.0), True
    w = k * e
    w = math.copysign(clip(abs(w), w_min, w_max), w)
    return (0.0, w), False


def drive_cmd(driven, target, yaw, yaw_hold, v=0.06, k=2.0, w_max=0.6):
    '''Straight drive with heading hold; target > 0 forward, < 0 backward. Returns ((v, w), done).'''
    remaining = abs(target) - abs(driven)
    if remaining <= 0.0:
        return (0.0, 0.0), True
    speed = clip(0.6 * remaining + 0.02, 0.02, abs(v))
    w = clip(k * wrap(yaw_hold - yaw), -w_max, w_max)
    return (math.copysign(speed, target), w), False


def line_pursuit(pose, p0, phi, v=0.05, lookahead=0.12):
    '''Pure pursuit onto the straight line through p0 with heading phi (odometry frame).
    Returns ((v, w), lateral error m (+ = robot left of the line), heading error rad).'''
    x, y, yaw = pose
    ux, uy = math.cos(phi), math.sin(phi)
    e_lat = (x - p0[0]) * -uy + (y - p0[1]) * ux
    fx, fy = x + e_lat * uy, y - e_lat * ux                 # foot point on the line
    tx, ty = fx + lookahead * ux, fy + lookahead * uy
    a = wrap(math.atan2(ty - y, tx - x) - yaw)
    w = 2.0 * v * math.sin(a) / lookahead
    return (v, w), e_lat, wrap(yaw - phi)


WALL_DEFAULTS = {
    'side': 'right',          # wall kept on this side
    'd_wall': 0.20,           # m
    'v': 0.08,                # m/s
    'k_d': 4.0,               # rad/s per m of distance error
    'k_a': 1.5,               # rad/s per rad of wall angle
    'front_slow': 0.40,       # m, slow down
    'front_turn': 0.30,       # m, turn away from the wall side
    'front_stop': 0.16,       # m, turn in place
    'w_turn': 0.9,            # rad/s, turning away from an obstacle / corner
    'w_seek': 0.45,           # rad/s, turning toward the wall side when it is lost
    'gap_straight_m': 0.30,   # m driven straight after losing the wall before seeking it
    'w_max': 1.0,
}


GAP_DEFAULTS = {
    'half_width': 0.11,       # m, corridor half width checked along a direction (robot half width ~0.10)
    'look': 0.9,              # m, free distance counted up to this
    'min_free': 0.25,         # m free needed to take a direction (less when the target is closer)
    'slow_free': 0.50,        # m, full speed only with this much free
    'v': 0.08,                # m/s
    'k_w': 1.6,               # rad/s per rad of direction
    'w_max': 1.0,
    'turn_deg': 50.0,         # direction further off than this: turn in place first
    'step_deg': 5.0,          # candidate directions -90..90 deg in these steps
    'keep': 0.3,              # cost per rad of change from the last direction (no left/right flapping)
}


def free_distance(scan, theta, half_width, look):
    '''Free length of a straight corridor (half_width) from the robot centre in direction theta (rad).'''
    c, s = math.cos(theta), math.sin(theta)
    along = scan.x * c + scan.y * s
    perp = -scan.x * s + scan.y * c
    sel = (along > 0.0) & (abs(perp) < half_width)
    if not sel.any():
        return look
    return min(look, float(along[sel].min()))


class GapPlanner(object):
    '''Goal-directed obstacle avoidance on one LiDAR scan: of the directions whose corridor is free,
    take the one closest to the target bearing. Not trapped by a free-standing obstacle the way a
    wall follower is (it leaves the obstacle as soon as the target side opens).'''

    def __init__(self, p=None):
        self.p = dict(GAP_DEFAULTS, **(p or {}))
        self.prev = 0.0
        self.mode = 'init'

    def step(self, scan, bearing, dist, need=None):
        '''bearing: target direction (rad, robot frame, + left); dist: m to the target (speed);
        need: m free required along a direction (default min(min_free, dist)). Returns (v, w).'''
        p = self.p
        need = min(p['min_free'], max(dist if need is None else need, 0.12))
        best, best_cost, best_free = None, None, 0.0
        n = int(round(90.0 / p['step_deg']))
        for i in range(-n, n + 1):
            th = math.radians(i * p['step_deg'])
            f = free_distance(scan, th, p['half_width'], p['look'])
            if f < need:
                continue
            cost = abs(wrap(th - bearing)) + p['keep'] * abs(th - self.prev)
            if best_cost is None or cost < best_cost:
                best, best_cost, best_free = th, cost, f
        if best is None:
            # boxed in ahead: turn in place toward the side with more room
            left = free_distance(scan, math.radians(70.0), p['half_width'], p['look'])
            right = free_distance(scan, math.radians(-70.0), p['half_width'], p['look'])
            self.mode = 'blocked'
            self.prev = 0.0
            return 0.0, (0.6 if left >= right else -0.6) * p['w_max']
        self.prev = best
        w = clip(p['k_w'] * best, -p['w_max'], p['w_max'])
        if abs(best) > math.radians(p['turn_deg']):
            self.mode = 'turn %.0f' % math.degrees(best)
            return 0.0, w
        room = clip((best_free - 0.12) / max(p['slow_free'] - 0.12, 1e-3), 0.3, 1.0)
        v = p['v'] * room * math.cos(best) * min(1.0, dist / 0.15 + 0.3)
        self.mode = 'go %.0f free %.2f' % (math.degrees(best), best_free)
        return v, w


class WallFollower(object):
    def __init__(self, p=None):
        self.p = dict(WALL_DEFAULTS, **(p or {}))
        self.lost_travel = None
        self.mode = 'init'

    def step(self, scan, travel):
        '''Returns (v, w). Turning away from the wall: + for a right wall, - for a left wall.'''
        p = self.p
        away = 1.0 if p['side'] == 'right' else -1.0
        front = scan.sector_min(-25.0, 25.0)
        if front < p['front_stop']:
            self.mode = 'corner'
            return 0.0, away * p['w_turn']
        if front < p['front_turn']:
            self.mode = 'turn'
            return 0.03, away * p['w_turn']
        wall = scan.wall(p['side'])
        if wall is None:
            if self.lost_travel is None:
                self.lost_travel = travel
            if travel - self.lost_travel < p['gap_straight_m']:
                self.mode = 'gap'
                return p['v'], 0.0
            self.mode = 'seek'
            return 0.6 * p['v'], -away * p['w_seek']
        self.lost_travel = None
        dist, ang = wall
        # too close -> turn away; wall bending toward us ahead (ang toward 'away' side) -> turn away
        w = away * p['k_d'] * (p['d_wall'] - dist) + p['k_a'] * ang
        w = clip(w, -p['w_max'], p['w_max'])
        v = p['v'] * (0.5 if front < p['front_slow'] else 1.0) * (1.0 - 0.5 * min(1.0, abs(w) / p['w_max']))
        self.mode = 'wall %.2fm' % dist
        return v, w
