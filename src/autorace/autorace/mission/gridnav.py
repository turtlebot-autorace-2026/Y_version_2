'''
Small occupancy grid + path planner for the tunnel, built on the fly from LiDAR (no map file, no Nav2).
No ROS here (unit-testable).

Grid in the tunnel frame: f = ahead of the entrance, s = toward the exit side (left, or right when
the exit is on the right). Every LiDAR point adds a hit to its cell; >= hits_min hits = obstacle.
Unknown cells count as free, so the first plan goes straight for the exit and later plans go around
what the robot has seen by then. The planning domain is limited to the tunnel box plus a corridor
through the exit opening, so no plan leads out of the entrance and round the outside.

Plan: obstacles grown by r_hard (blocked) and r_soft (extra cost, keeps the path off obstacles),
cost-to-go from the goal by repeated 8-neighbour relaxation (numpy), then the robot walks down the
cost-to-go for `lookahead` m: that point is where to drive next.
'''
import math

import numpy as np

INF = 1e9
NEIGHBOURS = [(1, 0, 1.0), (-1, 0, 1.0), (0, 1, 1.0), (0, -1, 1.0),
              (1, 1, math.sqrt(2)), (1, -1, math.sqrt(2)), (-1, 1, math.sqrt(2)), (-1, -1, math.sqrt(2))]

GRID_DEFAULTS = {
    'res': 0.03,            # m per cell
    'hits_min': 2,          # LiDAR hits for an obstacle cell
    'r_hard': 0.11,         # m: no path closer to an obstacle (robot half width ~0.10)
    'r_soft': 0.25,         # m: closer than this costs extra
    'soft_cost': 3.0,       # extra cost per cell at r_hard, falling to 0 at r_soft
    'back_m': 0.15,         # domain: this far behind the tunnel frame origin, only in the entry corridor
    'entrance_m': 0.12,     # domain: before this (the entrance wall line) only the entry corridor
    'entry_half_m': 0.15,   # entry corridor half width there
    'side_back_m': 0.30,    # domain: this far away from the exit side (Gazebo side wall 0.21-0.26)
    'far_m': 0.30,          # domain: this far beyond the exit centre line (Gazebo far wall 0.22-0.27)
    'exit_half_m': 0.30,    # exit corridor half length (along f) through the opening
    'outside_m': 0.70,      # exit corridor beyond the opening
    'max_iter': 600,
}


def shift(a, di, dj, fill):
    '''out[i, j] = a[i - di, j - dj] (fill outside).'''
    out = np.full_like(a, fill)
    h, w = a.shape
    out[max(di, 0):h + min(di, 0), max(dj, 0):w + min(dj, 0)] = \
        a[max(-di, 0):h + min(-di, 0), max(-dj, 0):w + min(-dj, 0)]
    return out


def ring_distance(mask, steps):
    '''Approximate distance in cells to the nearest True cell (octagonal growth), capped at steps + 1.'''
    d = np.where(mask, 0, steps + 1).astype(np.int32)
    m = mask.copy()
    for k in range(1, steps + 1):
        n = m | shift(m, 1, 0, False) | shift(m, -1, 0, False) | shift(m, 0, 1, False) | shift(m, 0, -1, False)
        if k % 2 == 0:
            n = n | shift(m, 1, 1, False) | shift(m, 1, -1, False) | shift(m, -1, 1, False) | shift(m, -1, -1, False)
        d[n & ~m] = k
        m = n
    return d


class GridNav(object):
    def __init__(self, frame, exit_fwd, exit_side, sgn, p=None):
        '''frame: (x0, y0, yaw0) odometry pose of the entrance; exit_side >= 0 m toward the exit side;
        sgn: +1 exit on the left, -1 on the right.'''
        self.p = dict(GRID_DEFAULTS, **(p or {}))
        p = self.p
        self.x0, self.y0, self.th = frame
        self.c, self.s = math.cos(self.th), math.sin(self.th)
        self.sgn = sgn
        self.res = p['res']
        self.fmin, fmax = -p['back_m'] - 0.1, exit_fwd + p['far_m']
        self.smin, smax = -p['side_back_m'], exit_side + p['outside_m']
        self.shape = (int(math.ceil((fmax - self.fmin) / self.res)) + 1,
                      int(math.ceil((smax - self.smin) / self.res)) + 1)
        f = self.fmin + self.res * np.arange(self.shape[0])[:, None]
        s = self.smin + self.res * np.arange(self.shape[1])[None, :]
        # Unknown cells count as free, so everything outside the tunnel walls must be outside the domain:
        # otherwise a plan leaves through the entrance and drives round the outside of the walls (found in
        # the 2D sim). Behind the entrance wall line only the entry corridor; the box margins stay within
        # r_hard of the walls, where the walls' own growth blocks them.
        interior = (f >= -p['back_m']) & (f <= exit_fwd + p['far_m']) & (s >= -p['side_back_m']) \
            & (s <= exit_side + 0.05)
        interior &= (f >= p['entrance_m']) | (np.abs(s) <= p['entry_half_m'])
        corridor = (np.abs(f - exit_fwd) <= p['exit_half_m']) & (s >= exit_side - 0.05)
        self.domain = interior | corridor
        self.hits = np.zeros(self.shape, np.int32)
        self.D = None
        self.goal = None

    # ---- coordinates ------------------------------------------------------------------

    def to_fs(self, x, y):
        dx, dy = x - self.x0, y - self.y0
        return dx * self.c + dy * self.s, self.sgn * (-dx * self.s + dy * self.c)

    def to_xy(self, f, s):
        l = self.sgn * s
        return self.x0 + f * self.c - l * self.s, self.y0 + f * self.s + l * self.c

    def cell(self, x, y):
        f, s = self.to_fs(x, y)
        return int(round((f - self.fmin) / self.res)), int(round((s - self.smin) / self.res))

    def cell_xy(self, i, j):
        return self.to_xy(self.fmin + i * self.res, self.smin + j * self.res)

    def inside(self, i, j):
        return 0 <= i < self.shape[0] and 0 <= j < self.shape[1]

    # ---- map --------------------------------------------------------------------------

    def add_scan(self, scan, pose):
        x, y, yaw = pose
        c, s = math.cos(yaw), math.sin(yaw)
        wx = x + c * scan.x - s * scan.y
        wy = y + s * scan.x + c * scan.y
        dx, dy = wx - self.x0, wy - self.y0
        f = dx * self.c + dy * self.s
        sd = self.sgn * (-dx * self.s + dy * self.c)
        i = np.rint((f - self.fmin) / self.res).astype(int)
        j = np.rint((sd - self.smin) / self.res).astype(int)
        ok = (i >= 0) & (i < self.shape[0]) & (j >= 0) & (j < self.shape[1])
        np.add.at(self.hits, (i[ok], j[ok]), 1)

    def obstacles(self):
        return self.hits >= self.p['hits_min']

    # ---- plan -------------------------------------------------------------------------

    def plan(self, goal_xy):
        p = self.p
        soft = int(math.ceil(p['r_soft'] / self.res))
        d = ring_distance(self.obstacles(), soft) * self.res
        blocked = (d < p['r_hard']) | ~self.domain
        frac = np.clip((p['r_soft'] - d) / max(p['r_soft'] - p['r_hard'], 1e-6), 0.0, 1.0)
        cost = 1.0 + p['soft_cost'] * frac
        cost[blocked] = INF
        gi, gj = self.cell(*goal_xy)
        gi, gj = min(max(gi, 0), self.shape[0] - 1), min(max(gj, 0), self.shape[1] - 1)
        D = np.full(self.shape, INF)
        D[gi, gj] = 0.0
        for _ in range(p['max_iter']):
            Dn = D
            for di, dj, step in NEIGHBOURS:
                Dn = np.minimum(Dn, shift(D, di, dj, INF) + step * cost)
            Dn[blocked] = INF
            Dn[gi, gj] = 0.0
            if np.array_equal(Dn, D):
                break
            D = Dn
        self.D, self.goal, self.blocked = D, (gi, gj), blocked
        return D

    def cost_to_go(self, x, y):
        if self.D is None:
            return INF
        i, j = self.cell(x, y)
        return float(self.D[i, j]) if self.inside(i, j) else INF

    def next_point(self, x, y, lookahead):
        '''Walk down the cost-to-go from the robot for lookahead m. None: no path from here.'''
        if self.D is None:
            return None
        i, j = self.cell(x, y)
        if not self.inside(i, j):
            return None
        if self.D[i, j] >= INF:           # robot inside a grown obstacle: start from the best cell nearby
            r = int(math.ceil(0.12 / self.res))
            best = None
            for a in range(i - r, i + r + 1):
                for b in range(j - r, j + r + 1):
                    if self.inside(a, b) and self.D[a, b] < INF:
                        v = self.D[a, b] + 3.0 * math.hypot(a - i, b - j)
                        if best is None or v < best[0]:
                            best = (v, a, b)
            if best is None:
                return None
            i, j = best[1], best[2]
        steps = int(round(lookahead / self.res))
        for _ in range(steps):
            if (i, j) == self.goal:
                break
            nxt = min(((self.D[i + di, j + dj], i + di, j + dj) for di, dj, _s in NEIGHBOURS
                       if self.inside(i + di, j + dj)), default=None)
            if nxt is None or nxt[0] >= self.D[i, j]:
                break
            i, j = nxt[1], nxt[2]
        return self.cell_xy(i, j)

    def path_blocked(self, x, y, lookahead):
        '''True when the obstacles seen since the last plan now block the next lookahead m of the path.'''
        if self.D is None:
            return True
        soft = int(math.ceil(self.p['r_hard'] / self.res)) + 1
        d = ring_distance(self.obstacles(), soft) * self.res
        i, j = self.cell(x, y)
        if not self.inside(i, j):
            return False
        for _ in range(int(round(lookahead / self.res))):
            if (i, j) == self.goal:
                return False
            nxt = min(((self.D[i + di, j + dj], i + di, j + dj) for di, dj, _s in NEIGHBOURS
                       if self.inside(i + di, j + dj)), default=None)
            if nxt is None or nxt[0] >= self.D[i, j]:
                return False
            i, j = nxt[1], nxt[2]
            if d[i, j] < self.p['r_hard']:
                return True
        return False
