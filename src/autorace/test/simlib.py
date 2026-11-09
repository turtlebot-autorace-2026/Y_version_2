'''Tiny 2D simulator for the offline tests: LiDAR ray casting against boxes / circles and a
kinematic differential-drive robot. Geometry in Gazebo world metres (autorace_gazebo v014).'''
import math

import numpy as np

from autorace.mission.lidar import Scan


class World(object):
    def __init__(self):
        self.segs = []      # (x0, y0, x1, y1)
        self.circles = []   # (cx, cy, r)
        self.boxes = []     # (cx, cy, sx, sy) axis-aligned, for clearance checks

    def add_box(self, cx, cy, sx, sy, yaw=0.0):
        c, s = math.cos(yaw), math.sin(yaw)
        hx, hy = sx / 2.0, sy / 2.0
        pts = [(cx + c * a - s * b, cy + s * a + c * b) for a, b in ((-hx, -hy), (hx, -hy), (hx, hy), (-hx, hy))]
        for i in range(4):
            self.segs.append(pts[i] + pts[(i + 1) % 4])
        if abs(s) < 1e-6 or abs(c) < 1e-6:
            self.boxes.append((cx, cy, sx if abs(c) > 0.5 else sy, sy if abs(c) > 0.5 else sx))

    def add_circle(self, cx, cy, r):
        self.circles.append((cx, cy, r))

    def scan(self, pose, n=360, rmin=0.12, rmax=3.5, lidar_x=-0.032):
        x, y, yaw = pose
        ox, oy = x + lidar_x * math.cos(yaw), y + lidar_x * math.sin(yaw)
        ranges = np.full(n, np.inf)
        for i in range(n):
            a = yaw + 2 * math.pi * i / n
            dx, dy = math.cos(a), math.sin(a)
            best = rmax + 1
            for x0, y0, x1, y1 in self.segs:
                ex, ey = x1 - x0, y1 - y0
                den = dx * ey - dy * ex
                if abs(den) < 1e-12:
                    continue
                t = ((x0 - ox) * ey - (y0 - oy) * ex) / den
                u = ((x0 - ox) * dy - (y0 - oy) * dx) / den
                if t > 0 and 0 <= u <= 1:
                    best = min(best, t)
            for cx, cy, r in self.circles:
                fx, fy = ox - cx, oy - cy
                b = fx * dx + fy * dy
                c = fx * fx + fy * fy - r * r
                disc = b * b - c
                if disc >= 0:
                    t = -b - math.sqrt(disc)
                    if t > 0:
                        best = min(best, t)
            if rmin <= best <= rmax:
                ranges[i] = best
        return Scan(ranges, 0.0, 2 * math.pi / n, rmin, rmax, lidar_x)

    def clearance(self, x, y, radius):
        '''Distance from a robot circle to the nearest obstacle (negative = overlap).'''
        d = np.inf
        for x0, y0, x1, y1 in self.segs:
            ex, ey = x1 - x0, y1 - y0
            t = max(0.0, min(1.0, ((x - x0) * ex + (y - y0) * ey) / (ex * ex + ey * ey)))
            d = min(d, math.hypot(x - x0 - t * ex, y - y0 - t * ey))
        for cx, cy, r in self.circles:
            d = min(d, math.hypot(x - cx, y - cy) - r)
        for cx, cy, sx, sy in self.boxes:          # inside a box counts as overlap
            if abs(x - cx) < sx / 2 and abs(y - cy) < sy / 2:
                d = -1.0
        return d - radius


def cast(world, ox, oy, ang, rmin=0.12, rmax=3.5):
    '''Vectorised ray cast, one origin (ox, oy) and world angle per ray; inf = no return.'''
    dx, dy = np.cos(ang)[:, None], np.sin(ang)[:, None]
    ox, oy = ox[:, None], oy[:, None]
    best = np.full(ang.size, rmax + 1.0)
    if world.segs:
        S = np.array(world.segs)
        x0, y0, ex, ey = S[:, 0][None], S[:, 1][None], (S[:, 2] - S[:, 0])[None], (S[:, 3] - S[:, 1])[None]
        den = dx * ey - dy * ex
        with np.errstate(divide='ignore', invalid='ignore'):
            t = ((x0 - ox) * ey - (y0 - oy) * ex) / den
            u = ((x0 - ox) * dy - (y0 - oy) * dx) / den
        ok = (np.abs(den) > 1e-12) & (t > 0) & (u >= 0) & (u <= 1)
        best = np.minimum(best, np.where(ok, t, np.inf).min(axis=1))
    if world.circles:
        C = np.array(world.circles)
        fx, fy = ox - C[:, 0][None], oy - C[:, 1][None]
        b = fx * dx + fy * dy
        disc = b * b - (fx * fx + fy * fy - C[:, 2][None] ** 2)
        with np.errstate(invalid='ignore'):
            t = -b - np.sqrt(disc)
        best = np.minimum(best, np.where((disc >= 0) & (t > 0), t, np.inf).min(axis=1))
    return np.where((best >= rmin) & (best <= rmax), best, np.inf)


class Robot(object):
    '''Kinematic robot for the closed-loop runs. With no options it is the plain ideal robot (scan every
    tick from the true pose, odometry = truth). Real-robot options:
      odom_v, odom_w   odometry scale error (odometry reports (1 + e) x the true speed / turn rate)
      scan_hz          LiDAR rate (TurtleBot3 LDS-01: 300 rpm = 5 Hz); the scan is reused between sweeps
      smear            each ray taken at its own time during the 1 / scan_hz sweep, reported as one scan
      noise, noise_rel range noise sigma = noise + noise_rel * r (LDS-01: +-10 mm < 0.5 m, +-3.5 % beyond)
    The mission sees the odometry pose; the LiDAR, the lane stand-in and the clearance use the true pose.'''

    def __init__(self, world, pose, dt=0.1, odom_v=0.0, odom_w=0.0, scan_hz=None, smear=False, noise=0.0,
                 noise_rel=0.0, seed=0, n=360, lidar_x=-0.032):
        self.world, self.dt = world, dt
        self.true, self.odom = list(pose), list(pose)
        self.odom_v, self.odom_w = odom_v, odom_w
        self.scan_hz, self.smear, self.noise, self.noise_rel = scan_hz, smear, noise, noise_rel
        self.real_scan = bool(scan_hz or smear or noise or noise_rel)
        self.rng = np.random.default_rng(seed)
        self.n, self.lidar_x = n, lidar_x
        self.t, self.travel = 0.0, 0.0
        self.hist = [(0.0, tuple(self.true))]
        self.last, self.next_scan = None, 0.0

    def move(self, v, w):
        if self.smear:                          # pose history for the sweep
            h = self.dt / 4
            for k in range(4):
                self.true = integrate(self.true, v, w, h, sub=1)
                self.hist.append((self.t + (k + 1) * h, tuple(self.true)))
            self.hist = self.hist[-60:]
        else:
            self.true = integrate(self.true, v, w, self.dt)
        old = self.odom
        self.odom = integrate(self.odom, v * (1.0 + self.odom_v), w * (1.0 + self.odom_w), self.dt)
        self.travel += math.hypot(self.odom[0] - old[0], self.odom[1] - old[1])
        self.t += self.dt

    def _pose_at(self, t):
        for tt, p in reversed(self.hist):
            if tt <= t + 1e-9:
                return p
        return self.hist[0][1]

    def scan(self):
        if not self.real_scan:
            return self.world.scan(self.true)
        hz = self.scan_hz or 1.0 / self.dt
        if self.last is not None and self.t < self.next_scan - 1e-9:
            return self.last
        n = self.n
        a = 2 * math.pi * np.arange(n) / n
        if self.smear:
            P = np.array([self._pose_at(self.t - (1.0 - i / n) / hz) for i in range(n)])
        else:
            P = np.tile(np.array(self.true), (n, 1))
        r = cast(self.world, P[:, 0] + self.lidar_x * np.cos(P[:, 2]), P[:, 1] + self.lidar_x * np.sin(P[:, 2]),
                 P[:, 2] + a)
        if self.noise or self.noise_rel:
            fin = np.isfinite(r)
            r[fin] += self.rng.normal(0.0, 1.0, int(fin.sum())) * (self.noise + self.noise_rel * r[fin])
        self.last = Scan(r, 0.0, 2 * math.pi / n, 0.12, 3.5, self.lidar_x)
        self.next_scan = self.t + 1.0 / hz
        return self.last


def integrate(pose, v, w, dt, sub=4):
    x, y, yaw = pose
    for _ in range(sub):
        h = dt / sub
        x += v * h * math.cos(yaw)
        y += v * h * math.sin(yaw)
        yaw += w * h
    return [x, y, math.atan2(math.sin(yaw), math.cos(yaw))]


def follow_polyline(pose, path, v=0.08, lookahead=0.12, k=1.0):
    '''Stand-in for the lane node: pure pursuit on a polyline [(x, y), ...].'''
    p = np.asarray(path, float)
    dense = [p[0]]
    for a, b in zip(p[:-1], p[1:]):                     # resample every 1 cm
        n = max(1, int(math.hypot(*(b - a)) / 0.01))
        dense.extend(a + (b - a) * k / n for k in range(1, n + 1))
    p = np.array(dense)
    x, y, yaw = pose
    d = np.hypot(p[:, 0] - x, p[:, 1] - y)
    i = int(np.argmin(d))
    j = i
    while j < len(p) - 1 and d[j] < lookahead:
        j += 1
    tx, ty = p[j]
    a = math.atan2(ty - y, tx - x) - yaw
    a = math.atan2(math.sin(a), math.cos(a))
    return v, k * 2 * v * math.sin(a) / max(lookahead, 1e-3)
