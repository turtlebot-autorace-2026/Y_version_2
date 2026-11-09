'''Offline checks of the sign board, mission FSM and cmd_vel mux (no ROS).'''
import math
import os
import sys

import pytest

sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..'))
from autorace.mission.fsm_core import MissionFSM  # noqa: E402
from autorace.mission.missions import Ctx  # noqa: E402
from autorace.mission.mux_core import ZERO, CmdVelMux  # noqa: E402
from autorace.mission.signs import SignBoard  # noqa: E402


class Run(object):
    '''Drives straight along x; signs are set like mission_model does (level, on change only).'''

    def __init__(self, start_mode='go', missions=None, fsm=None, yaw=0.0, **mission_params):
        p = {'start_mode': start_mode}
        if missions is not None:
            p['missions'] = missions
        p.update(fsm or {})
        self.fsm = MissionFSM(p, mission_params)
        self.signs = SignBoard()
        self.t, self.x, self.yaw = 0.0, 0.0, math.radians(yaw)
        self.light, self.bar, self.seq, self.lane, self.lane_seq = None, None, 0, 'both', 0
        self.out = self.tick()

    def ctx(self):
        yaw = math.atan2(math.sin(self.yaw), math.cos(self.yaw))      # odometry yaw is wrapped
        return Ctx(t=self.t, x=self.x, yaw=yaw, travel=self.x, signs=self.signs, lane_status=self.lane,
                   lane_seq=self.lane_seq, light=self.light, light_seq=self.seq, bar=self.bar, bar_seq=self.seq)

    def turn(self, deg, dist, steps=20):
        '''Drive dist while the heading changes by deg (+ = left); returns the outputs of every tick.'''
        outs = []
        for _ in range(steps):
            self.yaw += math.radians(deg) / steps
            outs.append(self.drive(dist / steps, steps=1))
        return outs

    def tick(self):
        self.signs.touch(self.x)
        self.out = self.fsm.step(self.ctx())
        return self.out

    def sign(self, name, value):
        self.signs.set(name, value, self.t, self.x)

    def drive(self, dist, steps=10):
        for _ in range(steps):
            self.x += dist / steps
            self.t += 0.1
            self.seq += 1
            self.lane_seq += 1
            self.tick()
        return self.out


# ---------------------------------------------------------------- signs

def test_signboard_levels():
    s = SignBoard()
    s.set('left', True, 0.0, 1.0)
    s.set('left', True, 0.1, 1.2)        # repeated True is not a new rise
    s.touch(1.5)
    assert s.since_seen('left', 1.5) == 0.0
    s.set('left', False, 0.3, 1.6)
    assert abs(s.since_seen('left', 2.0) - 0.4) < 1e-9
    assert [r[0] for r in s.pop_rises()] == ['left'] and s.pop_rises() == []
    with pytest.raises(ValueError):
        s.set('traffic', True, 0, 0)


# ---------------------------------------------------------------- start

def test_traffic_light_needs_red_then_green():
    r = Run(start_mode='light')
    assert r.out.drive == 'stop' and r.fsm.vision() == 'light'
    r.light = 'green'                    # green without a red first: ignored
    r.drive(0.0, steps=5)
    assert r.out.drive == 'stop'
    r.light = 'red'
    r.drive(0.0, steps=3)
    r.light = 'yellow'
    r.drive(0.0, steps=3)
    r.light = 'green'
    r.drive(0.0, steps=1)
    assert r.out.drive == 'stop'         # needs 2 green frames
    r.drive(0.0, steps=1)
    assert r.out.drive == 'lane' and r.out.reset and r.fsm.state == 'LANE'
    r.drive(0.0, steps=1)
    assert not r.out.reset


def test_start_go():
    r = Run()
    assert r.fsm.state == 'LANE' and r.out.drive == 'lane' and r.out.reset


# ---------------------------------------------------------------- intersection

def test_left_right_without_triangle_do_nothing():
    '''Left/right seen near the start or at the parking-area left arrow never start anything.'''
    r = Run()
    r.sign('left', True)
    r.drive(0.5)
    r.sign('left', False)
    r.sign('right', True)
    r.drive(0.5)
    assert r.fsm.state == 'LANE' and r.out.follow_line == 'both'
    r.sign('right', False)
    r.sign('intersection', True)        # the real intersection still works afterwards
    r.drive(0.3)
    assert r.fsm.active.name == 'intersection' and r.fsm.active.phase == 'armed'
    r.sign('left', True)
    r.drive(0.1)
    assert r.out.follow_line == 'yellow' and r.out.slow


@pytest.mark.parametrize('yaw0', [0.0, 175.0])              # 175: odometry yaw wraps at +-180 in the ring
@pytest.mark.parametrize('side,line', [('left', 'yellow'), ('right', 'white')])
def test_intersection_round_the_island_to_the_exit(side, line, yaw0):
    '''Outer line from the decision until the robot has turned into the exit, not 1 m after the
    sign left the camera (old rule: back on both lines on the far side -> round the island again).'''
    s = 1.0 if side == 'left' else -1.0
    r = Run(yaw=yaw0)
    r.sign('intersection', True)
    r.drive(0.3)
    assert r.fsm.active.phase == 'armed' and r.out.slow and r.out.follow_line == 'both'
    r.sign('intersection', False)
    r.sign(side, True)
    r.drive(0.4)                                    # decided; approach the island
    r.sign(side, False)                             # the sign leaves the front camera at the fork
    outs = r.turn(s * 90, 0.45)                     # into the branch
    outs += r.turn(0, 0.3)                          # along the near side of the island
    outs += r.turn(-s * 180, 0.95)                  # round the island end
    outs += r.turn(0, 0.4)                          # along the far side: 2.1 m after the sign was lost
    assert all(o.follow_line == line and o.slow for o in outs)
    assert r.fsm.active.phase == 'far_side'
    outs = r.turn(s * 90, 0.45)                     # into the exit
    assert r.fsm.active.phase == 'exit' and all(o.follow_line == line for o in outs)
    r.turn(0, 0.3)
    assert r.fsm.state == 'LANE' and r.out.follow_line == 'both' and not r.out.slow
    assert r.fsm.finished['intersection'] == 'ok'
    r.sign('intersection', True)                    # once only
    r.sign('left', True)
    r.drive(0.3)
    assert r.fsm.state == 'LANE'


def test_intersection_bend_and_wobble_do_not_fake_the_exit():
    '''A bend of the approach road the other way after the decision, and lane wobble.'''
    r = Run()
    r.sign('intersection', True)
    r.sign('right', True)
    r.drive(0.1)
    r.turn(45, 0.3)                                 # approach bends left, branch is right
    for _ in range(3):
        r.turn(10, 0.1)
        r.turn(-10, 0.1)
    assert r.fsm.active.phase == 'branch'
    r.turn(-135, 0.5)                               # into the right branch
    r.turn(180, 0.95)                               # round the island end
    assert r.fsm.active.phase == 'far_side'
    r.turn(-90, 0.45)
    r.turn(0, 0.3)
    assert r.fsm.finished['intersection'] == 'ok' and r.out.follow_line == 'both'


def test_intersection_no_exit_pattern_hands_back_at_max_m():
    r = Run(intersection={'max_m': 2.0})
    r.sign('intersection', True)
    r.sign('left', True)
    r.drive(0.0, steps=1)                           # decided here
    r.drive(1.9)
    assert r.out.follow_line == 'yellow'
    r.drive(0.2)
    assert r.fsm.state == 'LANE' and r.out.follow_line == 'both'
    assert r.fsm.finished['intersection'] == 'ok'   # branch was taken: not started again


def test_turn_sign_starts_intersection_when_enabled():
    '''Practice map without the triangle sign (intersection_on_turn_sign); off by default (test above).'''
    r = Run(fsm={'intersection_on_turn_sign': True})
    r.sign('right', True)
    r.drive(0.1)
    assert r.fsm.active.name == 'intersection' and r.fsm.active.branch == 'right'
    assert r.out.follow_line == 'white' and r.out.slow


def test_intersection_abort_can_restart():
    r = Run(intersection={'arm_timeout_m': 2.0})
    r.sign('intersection', True)
    r.drive(0.1)
    r.sign('intersection', False)
    r.drive(2.0)
    assert r.fsm.state == 'LANE' and r.fsm.finished['intersection'] == 'abort'
    r.sign('intersection', True)
    r.drive(0.1)
    assert r.fsm.active is not None and r.fsm.active.name == 'intersection'


def test_both_directions_wait():
    r = Run()
    r.sign('intersection', True)
    r.sign('left', True)
    r.sign('right', True)
    r.drive(0.3)
    assert r.fsm.active.phase == 'armed' and r.out.follow_line == 'both' and r.out.slow
    r.sign('right', False)
    r.drive(0.1)
    assert r.out.follow_line == 'yellow'


# ---------------------------------------------------------------- pending / order

def test_sign_during_other_mission_is_pending():
    r = Run(intersection={'max_m': 0.7})
    r.sign('intersection', True)
    r.sign('right', True)
    r.drive(0.2)
    r.sign('construction', True)         # rises while the intersection mission runs
    r.sign('right', False)
    r.drive(0.3)
    assert r.fsm.active.name == 'intersection' and 'construction' in r.fsm.pending
    r.drive(0.4)
    assert r.fsm.active is not None and r.fsm.active.name == 'construction'


def test_tunnel_on_by_default_and_can_be_left_out():
    r = Run()
    r.sign('tunnel', True)
    r.drive(0.1)
    assert r.fsm.active is not None and r.fsm.active.name == 'tunnel'
    r = Run(missions=['intersection', 'construction', 'parking', 'level_crossing'])
    r.sign('tunnel', True)
    r.drive(0.5)
    assert r.fsm.state == 'LANE' and not r.fsm.pending


def test_traffic_light_no_camera():
    r = Run(start_mode='light', traffic_light={'no_camera_go_s': 5.0})
    r.light = None
    for _ in range(45):                  # no frames: seq does not change
        r.t += 0.1
        r.tick()
    assert r.out.drive == 'stop'
    for _ in range(10):
        r.t += 0.1
        r.tick()
    assert r.fsm.state == 'LANE' and r.fsm.finished['traffic_light'] == 'no_camera'
    r2 = Run(start_mode='light')         # off by default (robot)
    for _ in range(100):
        r2.t += 0.1
        r2.tick()
    assert r2.out.drive == 'stop'


def test_sign_still_visible_after_pending_expired():
    r = Run(intersection={'max_m': 1.95})
    r.sign('intersection', True)
    r.sign('right', True)
    r.drive(0.1)
    r.sign('construction', True)          # rises during the intersection, stays in view
    r.drive(0.3)
    r.sign('right', False)
    r.sign('intersection', False)
    r.drive(1.4)                          # 1.5 m since the construction sign rose: past pending_m (1.0)
    assert r.fsm.active.name == 'intersection'
    r.drive(0.3)                          # intersection ends; the sign is still in view -> construction
    assert r.fsm.active is not None and r.fsm.active.name == 'construction'


def test_pending_dropped_after_pending_m():
    r = Run(intersection={'max_m': 2.0}, missions=['intersection', 'construction', 'parking',
                                                    'level_crossing', 'tunnel'])
    r.sign('intersection', True)
    r.sign('right', True)
    r.drive(0.1)
    r.sign('tunnel', True)
    r.sign('right', False)
    r.drive(0.2)
    r.sign('tunnel', False)               # seen briefly, then gone
    r.drive(2.3)
    assert r.fsm.active is None and 'tunnel' not in r.fsm.pending


def test_unknown_params():
    with pytest.raises(KeyError):
        MissionFSM({'start': 'go'})
    with pytest.raises(KeyError):
        MissionFSM({}, {'parking': {'bay': 1.0}})
    with pytest.raises(ValueError):
        MissionFSM({'missions': ['tunnel', 'traffic_light']})


# ---------------------------------------------------------------- level crossing

def test_level_crossing_stop_wait_go():
    r = Run(level_crossing={'use_lidar': False})
    r.sign('stop', True)
    r.bar = 'up'
    r.drive(0.3)
    assert r.fsm.active.name == 'level_crossing' and r.out.slow and r.out.drive == 'lane'
    assert r.fsm.vision() == 'bar'
    r.bar = 'down'
    r.drive(0.0, steps=1)
    assert r.out.drive == 'lane'          # 1 frame is not enough
    r.drive(0.0, steps=1)
    assert r.out.drive == 'stop'
    r.drive(0.0, steps=20)                # waiting, bar down
    assert r.out.drive == 'stop'
    r.bar = 'up'
    r.drive(0.0, steps=4)
    assert r.out.drive == 'stop'
    r.drive(0.0, steps=1)                 # 5 frames up
    assert r.out.drive == 'lane' and r.out.reset
    r.drive(0.4)
    assert r.fsm.finished['level_crossing'] == 'ok'


def test_level_crossing_max_wait():
    r = Run(level_crossing={'use_lidar': False, 'max_wait_s': 5.0})
    r.sign('stop', True)
    r.bar = 'down'
    r.drive(0.0, steps=3)
    assert r.out.drive == 'stop'
    r.drive(0.0, steps=50)                # bar never opens: go before the 30 s rule
    assert r.out.drive == 'lane'


# ---------------------------------------------------------------- mux

def test_mux_selects_one_source():
    mux = CmdVelMux(source_timeout=0.5, mode_timeout=1.0, accel=0.0)
    assert mux.tick(0.0) == (ZERO, 'no drive mode from the FSM')
    mux.set_mode('lane', 0.0)
    assert mux.on_cmd('lane', 0.1, 0.2, 0.05) == (0.1, 0.2)
    assert mux.on_cmd('mission', 0.3, 0.0, 0.05) is None
    assert mux.tick(0.1) == (None, 'ok')
    assert mux.tick(0.7) == (ZERO, 'lane silent')
    mux.set_mode('mission', 0.7)
    assert mux.on_cmd('mission', 0.3, 0.0, 0.75) == (0.3, 0.0)
    mux.set_mode('stop', 0.8)
    assert mux.on_cmd('mission', 0.3, 0.0, 0.85) is None
    assert mux.tick(0.85) == (ZERO, 'mode stop')


def test_mux_start_ramp():
    '''After a stop the forward speed rises at accel m/s^2; w is scaled with v (same radius).'''
    mux = CmdVelMux(source_timeout=0.5, mode_timeout=1.0, accel=0.3)
    mux.set_mode('lane', 0.0)
    mux.tick(0.0)                                             # a zero command went out at t=0
    v, w = mux.on_cmd('lane', 0.18, 0.9, 0.1)
    assert abs(v - 0.03) < 1e-9 and abs(w - 0.9 * 0.03 / 0.18) < 1e-9
    v, w = mux.on_cmd('lane', 0.18, 0.9, 0.3)
    assert abs(v - 0.09) < 1e-9
    v, w = mux.on_cmd('lane', 0.18, 0.9, 0.7)
    assert (v, w) == (0.18, 0.9)                              # ramp done, command passes unchanged
    assert mux.on_cmd('lane', 0.10, 0.0, 0.75) == (0.10, 0.0)  # slowing down is never limited
    assert mux.on_cmd('lane', -0.06, 0.0, 0.8) == (-0.06, 0.0)  # reverse is not ramped
    mux.set_mode('stop', 1.0)
    mux.tick(1.0)
    mux.set_mode('mission', 1.05)
    assert abs(mux.on_cmd('mission', 0.06, 0.0, 1.1)[0] - 0.03) < 1e-9   # ramps again after the stop
    off = CmdVelMux(accel=0.0)
    off.set_mode('lane', 0.0)
    off.tick(0.0)
    assert off.on_cmd('lane', 0.18, 0.9, 0.1) == (0.18, 0.9)


def test_mux_fsm_dead():
    mux = CmdVelMux(source_timeout=0.5, mode_timeout=1.0)
    mux.set_mode('lane', 0.0)
    assert mux.on_cmd('lane', 0.1, 0.0, 1.5) is None
    assert mux.tick(1.5) == (ZERO, 'no drive mode from the FSM')
    with pytest.raises(ValueError):
        mux.set_mode('teleop', 2.0)
