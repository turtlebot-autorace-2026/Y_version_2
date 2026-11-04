'''Camera colour rules (synthetic images) and LiDAR helpers (synthetic scans).'''
import math
import os
import sys

import cv2
import numpy as np

sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..'))
from autorace.mission.lidar import Scan  # noqa: E402
from autorace.mission.motion import line_pursuit, turn_cmd  # noqa: E402
from autorace.mission.vision import bar_state, classify_light  # noqa: E402

W, H = 640, 480
BGR = {'red': (20, 20, 255), 'yellow': (0, 200, 255), 'green': (60, 255, 20)}


def light_image(lit, floor_yellow=True):
    img = np.full((H, W, 3), 60, np.uint8)
    if floor_yellow:                                         # yellow lane line on the floor, low in the image
        cv2.line(img, (100, 470), (260, 400), BGR['yellow'], 8)
    cv2.rectangle(img, (470, 60), (530, 230), (30, 30, 30), -1)     # housing
    for i, c in enumerate(('red', 'yellow', 'green')):
        colour = BGR[c] if c == lit else (40, 40, 40)
        cv2.circle(img, (500, 90 + 55 * i), 18, colour, -1)
    return img


def test_light_colours():
    for c in ('red', 'yellow', 'green'):
        got, _ = classify_light(light_image(c))
        assert got == c, (c, got)
    got, _ = classify_light(light_image(None))
    assert got is None


def test_light_ignores_non_lens_shapes():
    img = light_image(None)
    cv2.rectangle(img, (300, 50), (620, 70), BGR['green'], -1)     # long green strip: not a lens
    got, _ = classify_light(img)
    assert got is None


def bar_image(down):
    img = np.full((H, W, 3), 50, np.uint8)
    if down:
        x0, x1, y0, y1 = 40, 600, 220, 245                   # lowered arm across the road
        for k, x in enumerate(range(x0, x1, 40)):
            cv2.rectangle(img, (x, y0), (min(x + 40, x1), y1), (20, 20, 230) if k % 2 == 0 else (240, 240, 240), -1)
    else:
        for k, y in enumerate(range(60, 300, 40)):           # raised arm: vertical stripes post at the side
            cv2.rectangle(img, (560, y), (580, y + 40), (20, 20, 230) if k % 2 == 0 else (240, 240, 240), -1)
    # STOP sign (red octagon, ~20 % of the width) at the right
    pts = np.array([[470 + 40 * math.cos(a), 140 + 40 * math.sin(a)] for a in np.linspace(0, 2 * math.pi, 9)[:-1]])
    cv2.fillPoly(img, [pts.astype(np.int32)], (20, 20, 220))
    return img


def test_bar_down_up():
    assert bar_state(bar_image(True))[0] == 'down'
    state, info = bar_state(bar_image(False))
    assert state == 'up', info['width']


def scan_of(points, n=360):
    '''Synthetic scan from robot-frame points (lidar at the robot centre).'''
    r = np.full(n, np.inf)
    for x, y in points:
        i = int(round(math.atan2(y, x) / (2 * math.pi) * n)) % n
        r[i] = min(r[i], math.hypot(x, y))
    return Scan(r, 0.0, 2 * math.pi / n, 0.12, 3.5, 0.0)


def test_wall_fit_right_and_left():
    wall = [(x, -0.25 + 0.2 * x) for x in np.linspace(-0.2, 0.4, 40)]      # right wall closing in ahead
    d, ang = scan_of(wall).wall('right')
    assert abs(d - 0.25 * math.cos(math.atan(0.2))) < 0.02 and ang > 0.1
    left = [(x, 0.3) for x in np.linspace(-0.2, 0.4, 40)]
    d, ang = scan_of(left).wall('left')
    assert abs(d - 0.3) < 0.02 and abs(ang) < 0.05
    assert scan_of(left).wall('right') is None


def test_sector_and_box():
    s = scan_of([(0.3, 0.0), (0.0, 0.4), (0.0, -0.2)])
    assert abs(s.sector_min(-10, 10) - 0.3) < 0.01
    assert abs(s.sector_min(80, 100) - 0.4) < 0.01 and abs(s.sector_min(-100, -80) - 0.2) < 0.01
    assert s.count_box(0.2, 0.4, -0.05, 0.05) == 1


def test_motion_primitives():
    (v, w), done = turn_cmd(0.0, math.pi / 2)
    assert v == 0.0 and w > 0 and not done
    (v, w), done = turn_cmd(0.0, 0.01)
    assert done
    (v, w), e, eh = line_pursuit((0.0, 0.0, 0.0), (0.0, -0.3), 0.0)          # line 0.3 m to the right
    assert w < 0 and abs(e - 0.3) < 1e-9 and abs(eh) < 1e-9
