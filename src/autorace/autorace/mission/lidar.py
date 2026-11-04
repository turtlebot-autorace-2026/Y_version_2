'''
LaserScan helpers, no ROS. Robot frame: x forward, y left, angles CCW from forward (degrees in
the API). TurtleBot3 LDS: angle_min 0, 360 samples; the sensor sits lidar_x m from base_link.
'''
import math

import numpy as np


class Scan(object):
    def __init__(self, ranges, angle_min, angle_inc, range_min=0.12, range_max=3.5, lidar_x=-0.032):
        r = np.asarray(ranges, float)
        a = angle_min + angle_inc * np.arange(r.size)
        ok = np.isfinite(r) & (r >= max(range_min, 0.01)) & (r <= range_max)
        self.r = r[ok]
        self.a = np.arctan2(np.sin(a[ok]), np.cos(a[ok]))          # -pi..pi
        self.x = self.r * np.cos(self.a) + lidar_x
        self.y = self.r * np.sin(self.a)

    def sector_min(self, deg0, deg1, default=float('inf')):
        '''Smallest range with angle in [deg0, deg1] (degrees, -180..180, deg0 < deg1).'''
        a0, a1 = math.radians(deg0), math.radians(deg1)
        sel = (self.a >= a0) & (self.a <= a1)
        return float(self.r[sel].min()) if sel.any() else default

    def count_box(self, x0, x1, y0, y1):
        '''Points inside the robot-frame box.'''
        return int(((self.x >= x0) & (self.x <= x1) & (self.y >= y0) & (self.y <= y1)).sum())

    def wall(self, side, max_dist=0.6, deg=(40.0, 130.0), min_points=6):
        '''Line fit to the points on one side. side: right | left.
        Returns (distance m, angle rad) or None. angle > 0: the wall turns toward the robot's left ahead
        (for both sides, in the robot frame: y = tan(angle) * x + b).'''
        s = -1.0 if side == 'right' else 1.0
        a0, a1 = math.radians(deg[0]), math.radians(deg[1])
        sa = self.a * s
        sel = (sa >= a0) & (sa <= a1) & (self.r <= max_dist)
        if sel.sum() < min_points:
            return None
        x, y = self.x[sel], self.y[sel]
        if np.ptp(x) < 0.08:
            return None
        k, b = np.polyfit(x, y, 1)
        dist = abs(b) / math.sqrt(1.0 + k * k)
        return dist, math.atan(k)
