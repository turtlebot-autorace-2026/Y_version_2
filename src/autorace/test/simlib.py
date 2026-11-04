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
