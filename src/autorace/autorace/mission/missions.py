'''
Missions, no ROS. Each mission is started by the FSM, gets a Ctx every tick and returns Outputs.

Who drives (Outputs.drive -> cmd_vel_mux):
  lane     the lane node (lane_detection_ad); the mission only sets follow_line / slow / offset
  mission  Outputs.cmd from the mission (odometry / LiDAR manoeuvres)
  stop     zero
The lane node keeps running all the time. When control comes back to it from stop/mission, the
mission sets Outputs.reset so the node restarts its estimator (otherwise it would resume at the
speed it was commanding while it was muxed out).

Default distances are from the Gazebo world (autorace_gazebo v014, 260/227 scale), see README.
'''
import math

from autorace.mission.gridnav import GRID_DEFAULTS, INF, GridNav
from autorace.mission.motion import GAP_DEFAULTS, GapPlanner, WallFollower, drive_cmd, line_pursuit, turn_cmd, wrap

GOOD_LANE = ('both', 'yellow_only', 'white_only')


class Outputs(object):
    def __init__(self, drive='lane', follow_line='both', slow=False, offset=0.0, cmd=(0.0, 0.0), reset=False):
        self.drive = drive              # lane | mission | stop
        self.follow_line = follow_line  # both | yellow | white
        self.slow = slow                # lane node v_max_intersection
        self.offset = offset            # m, lane path offset, + = left
        self.cmd = cmd                  # (v, w) when drive == mission
        self.reset = reset              # restart the lane estimator this tick

    def as_dict(self):
        return dict(drive=self.drive, follow_line=self.follow_line, slow=self.slow,
                    offset=round(self.offset, 3), cmd=(round(self.cmd[0], 3), round(self.cmd[1], 3)),
                    reset=self.reset)


class Ctx(object):
    '''Everything a mission may look at in one tick.'''

    def __init__(self, t=0.0, x=0.0, y=0.0, yaw=0.0, travel=0.0, signs=None, lane_status='', lane_seq=0,
                 scan=None, light=None, light_seq=0, bar=None, bar_seq=0, lane_centre=None):
        self.t, self.x, self.y, self.yaw, self.travel = t, x, y, yaw, travel
        self.signs = signs
        self.lane_status, self.lane_seq = lane_status, lane_seq      # /lane/status and a message counter
        self.scan = scan                                            # lidar.Scan or None
        self.light, self.light_seq = light, light_seq               # red|yellow|green|None per camera frame
        self.bar, self.bar_seq = bar, bar_seq                       # down|up per camera frame
        self.lane_centre = lane_centre      # (lateral m + left, heading rad) of the lane centre, or None


class Mission(object):
    name = 'mission'
    vision = None          # 'light' | 'bar' | None: which camera check the node must run
    DEFAULTS = {}

    def __init__(self, p=None):
        self.p = dict(self.DEFAULTS)
        for k, v in (p or {}).items():
            if k not in self.DEFAULTS:
                raise KeyError('%s: unknown parameter %s' % (self.name, k))
            self.p[k] = v
        self.done, self.result, self.phase = False, None, 'idle'
        self.log = []

    def start(self, ctx):
        self.done, self.result = False, None
        self.t0, self.travel0 = ctx.t, ctx.travel
        self._enter('start', ctx)

    def _enter(self, phase, ctx):
        self.phase, self.phase_t, self.phase_travel = phase, ctx.t, ctx.travel
        self.phase_x, self.phase_y, self.phase_yaw = ctx.x, ctx.y, ctx.yaw
        self.log.append('%s: %s' % (self.name, phase))

    def _finish(self, result, why=''):
        self.done, self.result = True, result
        self.log.append('%s: done (%s) %s' % (self.name, result, why))

    def phase_dist(self, ctx):
        return ctx.travel - self.phase_travel

    def status(self):
        return '%s/%s' % (self.name, self.phase)


class _FrameCounter(object):
    '''Counts consecutive camera frames (by sequence number) that satisfy a condition.'''

    def __init__(self):
        self.seq, self.n = -1, 0

    def update(self, seq, ok):
        if seq != self.seq:
            self.seq = seq
            self.n = self.n + 1 if ok else 0
        return self.n


# ------------------------------------------------------------------------------- traffic light

class TrafficLight(Mission):
    '''Wait at the start line; go on green. Race timer starts at the first green; the robot must
    cross the start line while green (5 s).'''
    name = 'traffic_light'
    vision = 'light'
    DEFAULTS = {
        'need_red_first': True,     # accept green only after a red was seen (guards a static false green)
        'green_frames': 2,
        'fallback_s': 0.0,          # >0: go anyway this long after the last red/yellow (0 = off)
        'no_camera_go_s': 0.0,      # >0: go if no camera frame at all arrived this long (0 = off; robot: keep 0)
        'jitter_after_s': 0.0,      # >0: waiting this long, nudge back and forth (jitter_v for jitter_s each
                                    # way, net zero) so the robot is never "stopped" for 30 s before the start.
                                    # Only if the organizer says the 30 s rule applies while waiting for
                                    # green (rules file, question 7-3). 0 = off (default)
        'jitter_v': 0.02,
        'jitter_s': 0.5,
        'no_red_after_s': 4.0,      # waiting this long without ever seeing a red -> green alone is enough.
                                    # Red lens outside the ROI, or the light already green at put-down:
                                    # with need_red_first alone the robot never starts (the timer runs from
                                    # the first green). Not "green for N s": green lasts 5 s, so waiting on
                                    # green would start on the next red. 0 = off
    }

    def start(self, ctx):
        Mission.start(self, ctx)
        self.seen_red, self.last_ry_t = False, None
        self.green = _FrameCounter()
        self.seq0 = ctx.light_seq
        self._enter('wait', ctx)

    def step(self, ctx):
        if ctx.light in ('red', 'yellow'):
            self.last_ry_t = ctx.t
            self.seen_red = self.seen_red or ctx.light == 'red'
        no_red_ok = self.p['no_red_after_s'] > 0 and ctx.t - self.phase_t >= self.p['no_red_after_s']
        ok = ctx.light == 'green' and (self.seen_red or not self.p['need_red_first'] or no_red_ok)
        if self.green.update(ctx.light_seq, ok) >= self.p['green_frames']:
            self._finish('ok', 'green' if self.seen_red else 'green (no red seen in %.0f s)' % (ctx.t - self.phase_t))
            return Outputs(drive='lane', reset=True)
        if self.p['no_camera_go_s'] > 0 and ctx.light_seq == self.seq0 and ctx.t - self.phase_t >= self.p['no_camera_go_s']:
            self._finish('no_camera', 'no front-camera frame for %.1f s' % (ctx.t - self.phase_t))
            return Outputs(drive='lane', reset=True)
        if self.p['fallback_s'] > 0 and self.last_ry_t is not None and ctx.t - self.last_ry_t > self.p['fallback_s']:
            self._finish('fallback', 'no light for %.1f s' % (ctx.t - self.last_ry_t))
            return Outputs(drive='lane', reset=True)
        p = self.p
        if p['jitter_after_s'] > 0 and ctx.t - self.phase_t >= p['jitter_after_s']:
            k = int((ctx.t - self.phase_t - p['jitter_after_s']) / p['jitter_s'])
            return Outputs(drive='mission', cmd=(p['jitter_v'] if k % 2 == 0 else -p['jitter_v'], 0.0))
        return Outputs(drive='stop')

    def status(self):
        return '%s/%s red_seen=%s green_frames=%d' % (self.name, self.phase, self.seen_red, self.green.n)


# ------------------------------------------------------------------------------- intersection

class Intersection(Mission):
    '''Started by /intersection (triangle; FSM option intersection_on_turn_sign: also by /left /right).
    Slow until exactly one of /left, /right is up, then follow only the OUTER line of that branch
    until the robot has gone round the island and turned into the exit.

    Layout (Gazebo v014 and the club map): a stadium-shaped island inside a ring, the signs on the
    island facing the entry. Each branch: turn toward the sign side, go round the island end, run
    along the far side, turn toward the sign side again into the exit. The outer line of the branch
    bends into the exit by itself. Club map photo (2026-11-06): the island is yellow on one half and
    white on the other, each half next to the branch whose outer line has the other colour, so
    following one colour does not pick up the island (Gazebo island colours: not checked).

    Exit by heading, measured as the turn u since the decision (+ = toward the chosen side; only
    differences are used, so a bend in the approach road does not matter):
      into the branch          u rises to a peak P      (~ +90)
      round the island end     P - u >= around_deg      (~ 180)
      into the exit            u - (lowest u) >= exit_deg   (~ 90)
    then exit_m more on the outer line, then both lines again.

    Why not "hold_m after the sign was last seen" (the old rule): the sign leaves the front camera at
    the fork, so 1 m later the robot is still in the ring. Back on both lines there, the lane node's
    pair check rejects the pair at the exit mouth and keeps yellow (lane_mode) - on the branch whose
    island line is yellow that is the island, and the robot goes round again.'''
    name = 'intersection'
    DEFAULTS = {
        'arm_timeout_m': 4.0,       # m without a direction sign -> abort (not counted as done)
        'left_line': 'yellow',      # outer line of the left branch (robot drives with yellow on its left)
        'right_line': 'white',
        'around_deg': 140.0,        # turn away from the sign side after the peak = round the island end (~180)
        'exit_deg': 50.0,           # then back toward the sign side = into the exit (~90)
        'exit_m': 0.25,             # m on the outer line after the exit turn is seen
        'max_m': 5.0,               # m after the decision without the exit pattern -> hand back (done)
        'timeout_s': 120.0,
    }

    def start(self, ctx):
        Mission.start(self, ctx)
        self.branch, self.sgn = None, 0.0
        self.u = self.peak = self.low = 0.0
        self.prev_yaw = ctx.yaw
        self._enter('armed', ctx)

    def _track(self, ctx):
        '''u: turn since the decision, + toward the chosen side (odometry yaw, unwrapped).'''
        self.u += self.sgn * math.degrees(wrap(ctx.yaw - self.prev_yaw))
        self.prev_yaw = ctx.yaw

    def step(self, ctx):
        s, p = ctx.signs, self.p
        if self.phase == 'armed':
            l, r = s.active['left'], s.active['right']
            if l == r:
                if self.phase_dist(ctx) >= p['arm_timeout_m']:
                    self._finish('abort', 'no single left/right within %.1f m (%s)'
                                 % (p['arm_timeout_m'], 'both up' if l else 'none'))
                    return Outputs()
                return Outputs(slow=True)            # approach the fork slowly
            self.branch = 'left' if l else 'right'
            self.sgn = 1.0 if l else -1.0
            self.u = self.peak = self.low = 0.0
            self.prev_yaw = ctx.yaw
            self._enter('branch', ctx)
        line = p[self.branch + '_line']
        self._track(ctx)
        if self.phase_dist_total(ctx) >= p['max_m'] or ctx.t - self.decided_t >= p['timeout_s']:
            self._finish('ok', '%s, exit not seen within %.1f m / %.0f s, phase %s u=%.0f'
                         % (self.branch, p['max_m'], p['timeout_s'], self.phase, self.u))
            return Outputs()
        if self.phase == 'branch':
            self.peak = max(self.peak, self.u)
            if self.peak - self.u >= p['around_deg']:
                self.low = self.u
                self._enter('far_side', ctx)
        elif self.phase == 'far_side':
            self.low = min(self.low, self.u)
            if self.u - self.low >= p['exit_deg']:
                self._enter('exit', ctx)
        elif self.phase == 'exit':
            if self.phase_dist(ctx) >= p['exit_m']:
                self._finish('ok', '%s, out of the ring' % self.branch)
                return Outputs()
        return Outputs(follow_line=line, slow=True)

    def _enter(self, phase, ctx):
        Mission._enter(self, phase, ctx)
        if phase == 'branch':
            self.decided_t, self.decided_travel = ctx.t, ctx.travel

    def phase_dist_total(self, ctx):
        return ctx.travel - self.decided_travel

    def status(self):
        if self.branch is None:
            return '%s/%s' % (self.name, self.phase)
        return '%s/%s %s u=%.0f peak=%.0f' % (self.name, self.phase, self.branch, self.u, self.peak)


# ------------------------------------------------------------------------------- construction

class Construction(Mission):
    '''Started by /construction. The section is two lanes wide (Gazebo: 0.574 m between the lines,
    3 boxes 0.25 x 0.1 m alternating outer / inner / outer). The lane camera sees only about +-0.2 m
    to the side, so in each corridor the robot follows the line next to it (inner corridor = yellow,
    outer = white) and crosses between corridors with a short odometry manoeuvre.

    Gazebo map: box 1 sits right after the sharp inner (yellow) corner, so the mission starts on the
    white line: the wide white arc leads straight into the outer corridor. After box 3 the white
    line's exit arc leads back onto the normal lane centre.'''
    name = 'construction'
    DEFAULTS = {
        'start_line': 'white',      # line followed from the sign (see above)
        'spacing': 0.285,           # m between the corridor centres (section 0.574 - lane 0.286 ... 0.288)
        'corridor_half': 0.12,      # m, half width checked around a corridor centre
        'sweep_from': 0.08,         # m from the robot centre where the swept strip to the other corridor starts
        'sweep_ahead': 0.30,        # m ahead the swept strip is checked (about the manoeuvre length)
        'look_ahead': 0.55,         # m checked ahead (lane frame)
        'beside_back': 0.10,        # m behind the axle also checked in the target corridor (rear overhang)
        'min_points': 3,
        'aligned_deg': 25.0,        # change corridor only when the lane runs within this of the heading
        'change_v': 0.05,
        'change_lookahead': 0.12,
        'change_done_m': 0.025,
        'change_done_deg': 10.0,
        'change_max_m': 0.7,
        'done_clear_m': 0.6,        # m without boxes ahead in either corridor -> done
        'max_m': 5.0,
    }

    def start(self, ctx):
        Mission.start(self, ctx)
        self.line = self.p['start_line']
        self.last_obs_travel, self.seen_any, self.changes = None, False, 0
        self._enter('follow ' + self.line, ctx)

    def _blocked(self, xl, yl, lo, hi, back, ahead=None):
        '''Points in a lateral band [lo, hi] (lane frame, relative to the robot, + left), from back
        behind the axle to ahead (default look_ahead).'''
        ahead = self.p['look_ahead'] if ahead is None else ahead
        sel = (xl >= -back) & (xl <= ahead) & (yl >= lo) & (yl <= hi)
        return int(sel.sum()) >= self.p['min_points']

    def step(self, ctx):
        p = self.p
        if ctx.travel - self.travel0 >= p['max_m']:
            self._finish('ok', 'max distance')
            return Outputs()
        if self.phase == 'change':
            cmd, e_lat, e_head = line_pursuit((ctx.x, ctx.y, ctx.yaw), self.target_p0, self.target_phi,
                                              p['change_v'], p['change_lookahead'])
            if (abs(e_lat) < p['change_done_m'] and abs(e_head) < math.radians(p['change_done_deg'])) \
                    or self.phase_dist(ctx) >= p['change_max_m']:
                self.line = 'white' if self.line == 'yellow' else 'yellow'
                self._enter('follow ' + self.line, ctx)
                return Outputs(follow_line=self.line, slow=True, reset=True)
            return Outputs(drive='mission', cmd=cmd)

        # follow the line of the current corridor; look for boxes in both corridors (lane frame)
        out = Outputs(follow_line=self.line, slow=True)
        if ctx.scan is not None and ctx.lane_centre is not None:
            # no lane centre (line lost / stale): the corridor geometry is unknown, keep following; a
            # change decided on (0, 0) here would start the odometry manoeuvre toward a guessed corridor
            cy, cth = ctx.lane_centre
            c, s_ = math.cos(cth), math.sin(cth)
            xl, yl = ctx.scan.x * c + ctx.scan.y * s_, -ctx.scan.x * s_ + ctx.scan.y * c
            side = -1.0 if self.line == 'yellow' else 1.0          # other corridor: right of yellow's
            h = p['corridor_half']
            here = self._blocked(xl, yl, cy - h, cy + h, -0.05)
            # target corridor ahead, plus the whole strip the robot sweeps while crossing over (a box
            # beside the robot may stick out of the target corridor toward us); the strip only as far
            # ahead as the manoeuvre reaches
            t = cy + side * p['spacing']
            a, b = cy + side * p['sweep_from'], t
            there = self._blocked(xl, yl, t - h, t + h, p['beside_back']) or \
                self._blocked(xl, yl, min(a, b), max(a, b), p['beside_back'], p['sweep_ahead'])
            if here or there:
                self.last_obs_travel, self.seen_any = ctx.travel, True
            if here and not there and abs(cth) < math.radians(p['aligned_deg']):
                phi = ctx.yaw + cth
                nx, ny = -math.sin(phi), math.cos(phi)              # lane left normal (odometry)
                off = cy + side * p['spacing']
                self.target_p0, self.target_phi = (ctx.x + off * nx, ctx.y + off * ny), phi
                self.changes += 1
                self._enter('change', ctx)
                return self.step(ctx)
        if self.seen_any and ctx.travel - self.last_obs_travel >= p['done_clear_m']:
            self._finish('ok', '%d corridor changes' % self.changes)
            return Outputs()
        return out


# ------------------------------------------------------------------------------- parking

class Parking(Mission):
    '''Started by /parking. Gazebo map: a dead-end spur on the left of the road leads into a lot of
    three 0.285 m columns (left stall | entry corridor | right stall), stall row 0.94 m after the turn.

    lane yellow (turns left into the spur) -> lane in the spur -> straight to the stall row ->
    LiDAR: which stall is free -> turn 90 deg -> in -> wait -> back out -> turn 90 deg more (facing
    out) -> straight -> lane yellow (turns left onto the road) -> done.

    Corridor heading: first the road heading at the sign + 90 deg, then re-taken in the spur from
    odometry yaw + /lane/centre heading (circular mean). Odometry yaw error over the ~180 deg turned
    before the stall otherwise lands in the stall: 2D sim, yaw scale +-3 %: 5 deg in the stall and a
    corner on the stall line; with the lane anchor every run stays >= 24 mm inside (+-6 %, 3 deg noise).'''
    name = 'parking'
    DEFAULTS = {
        'spur_turn_deg': 80.0,      # left turn done = in the spur (spur_lane_m is measured from here)
        'spur_angle_deg': 90.0,     # spur heading = road heading at the sign + this (left = +)
        'spur_lane_m': 0.45,        # lane following in the spur after that
        'lot_drive_m': 0.49,        # straight to the stall row
        'occupied_range': 0.45,     # m, side sector closer than this = robot in that stall
        'default_side': 'left',     # both free (Gazebo has no house robot)
        'bay_drive_m': 0.30,        # stall centre 0.284 + 0.015: the footprint (0.10 m behind / 0.07 m ahead
                                    # of the axle) is centred, not the axle
        'anchor_from_m': 0.15,      # in the spur, lane heading samples from this far on
        'anchor_min': 5,            # at least this many samples, else keep the heading from the sign
        'anchor_max_dev_deg': 15.0, # lane heading further than this from the sign-based one: ignored
        'park_wait_s': 1.5,
        'scan_wait_s': 0.6,
        'exit_turn_deg': 60.0,      # left turn back onto the road
        'exit_lane_m': 0.25,
        'v': 0.06,
        'arm_max_m': 2.5,           # no turn into the spur within this -> abort
    }

    def start(self, ctx):
        Mission.start(self, ctx)
        self.side, self.ranges = None, None
        # corridor heading from the (straight) road heading at the sign, not from the yaw after lane
        # following in the spur, which may still be swinging
        self.yaw_ref = wrap(ctx.yaw + math.radians(self.p['spur_angle_deg']))
        self.anchor, self.anchor_dev = [], None
        self._enter('to_spur', ctx)

    def _anchor(self):
        '''Corridor heading from the lane samples taken in the spur, if there are enough and they agree
        with the sign-based heading.'''
        p = self.p
        if len(self.anchor) < p['anchor_min']:
            self.log.append('parking: %d lane heading samples in the spur, heading from the sign kept' % len(self.anchor))
            return
        a = math.atan2(sum(math.sin(v) for v in self.anchor), sum(math.cos(v) for v in self.anchor))
        dev = wrap(a - self.yaw_ref)
        if abs(dev) > math.radians(p['anchor_max_dev_deg']):
            self.log.append('parking: lane heading %.1f deg off the sign-based one, ignored' % math.degrees(dev))
            return
        self.yaw_ref, self.anchor_dev = a, math.degrees(dev)
        self.log.append('parking: corridor heading from the lane (%+.1f deg, %d samples)' % (self.anchor_dev, len(self.anchor)))

    def _dist(self, ctx):
        return math.hypot(ctx.x - self.phase_x, ctx.y - self.phase_y)

    def step(self, ctx):
        p, ph = self.p, self.phase
        if ph == 'to_spur':
            if wrap(ctx.yaw - self.phase_yaw) >= math.radians(p['spur_turn_deg']):
                self._enter('spur', ctx)
            elif self.phase_dist(ctx) > p['arm_max_m']:
                self._finish('abort', 'no left turn into the spur')
                return Outputs()
            return Outputs(follow_line='yellow', slow=True)
        if ph == 'spur':
            if ctx.lane_centre is not None and self.phase_dist(ctx) >= p['anchor_from_m']:
                self.anchor.append(wrap(ctx.yaw + ctx.lane_centre[1]))
            if self.phase_dist(ctx) >= p['spur_lane_m']:
                self._anchor()
                self._enter('lot', ctx)
            else:
                return Outputs(follow_line='yellow', slow=True)
        if self.phase == 'lot':
            cmd, fin = drive_cmd(self._dist(ctx), p['lot_drive_m'], ctx.yaw, self.yaw_ref, p['v'])
            if not fin:
                return Outputs(drive='mission', cmd=cmd)
            self._enter('scan', ctx)
        if self.phase == 'scan':
            if ctx.t - self.phase_t < p['scan_wait_s'] or ctx.scan is None:
                return Outputs(drive='mission')
            left, right = ctx.scan.sector_min(60, 120), ctx.scan.sector_min(-120, -60)
            self.ranges = (left, right)
            occ_l, occ_r = left < p['occupied_range'], right < p['occupied_range']
            if occ_l and not occ_r:
                self.side = 'right'
            elif occ_r and not occ_l:
                self.side = 'left'
            elif not occ_l and not occ_r:
                self.side = p['default_side']
            else:
                self.side = 'left' if left > right else 'right'
            self.sgn = 1.0 if self.side == 'left' else -1.0
            self.log.append('parking: left %.2f m right %.2f m -> %s' % (left, right, self.side))
            self._enter('turn_in', ctx)
        if self.phase == 'turn_in':
            cmd, fin = turn_cmd(ctx.yaw, self.yaw_ref + self.sgn * math.pi / 2)
            if not fin:
                return Outputs(drive='mission', cmd=cmd)
            self._enter('in', ctx)
        if self.phase == 'in':
            cmd, fin = drive_cmd(self._dist(ctx), p['bay_drive_m'], ctx.yaw, self.yaw_ref + self.sgn * math.pi / 2, p['v'])
            if not fin:
                return Outputs(drive='mission', cmd=cmd)
            self._enter('parked', ctx)
        if self.phase == 'parked':
            if ctx.t - self.phase_t < p['park_wait_s']:
                return Outputs(drive='mission')
            self._enter('out', ctx)
        if self.phase == 'out':
            cmd, fin = drive_cmd(self._dist(ctx), -p['bay_drive_m'], ctx.yaw, self.yaw_ref + self.sgn * math.pi / 2, p['v'])
            if not fin:
                return Outputs(drive='mission', cmd=cmd)
            self._enter('turn_out', ctx)
        if self.phase == 'turn_out':
            cmd, fin = turn_cmd(ctx.yaw, self.yaw_ref + math.pi)
            if not fin:
                return Outputs(drive='mission', cmd=cmd)
            self._enter('lot_out', ctx)
        if self.phase == 'lot_out':
            cmd, fin = drive_cmd(self._dist(ctx), p['lot_drive_m'], ctx.yaw, self.yaw_ref + math.pi, p['v'])
            if not fin:
                return Outputs(drive='mission', cmd=cmd)
            self._enter('exit', ctx)
            return Outputs(follow_line='yellow', slow=True, reset=True)
        if self.phase == 'exit':
            if wrap(ctx.yaw - self.phase_yaw) >= math.radians(p['exit_turn_deg']):
                self._enter('exit_straight', ctx)
            return Outputs(follow_line='yellow', slow=True)
        if self.phase == 'exit_straight':
            if self.phase_dist(ctx) >= p['exit_lane_m']:
                self._finish('ok', 'parked %s' % self.side)
                return Outputs()
            return Outputs(follow_line='yellow', slow=True)
        raise RuntimeError('parking: phase %s' % self.phase)

    def status(self):
        return '%s/%s side=%s' % (self.name, self.phase, self.side)


# ------------------------------------------------------------------------------- level crossing

class LevelCrossing(Mission):
    '''Started by /stop. Drive on slowly; sensor 1 lowers the bar as the robot passes it. Stop when
    the bar is seen down (camera red span, or LiDAR in front), wait until it is up, go.
    Gazebo: sensor 1 -> sensor 2 is 0.229 m, the bar closes in 1 s, sensor 2 is 0.069 m before the bar.'''
    name = 'level_crossing'
    vision = 'bar'
    DEFAULTS = {
        'down_frames': 2,
        'up_frames': 5,
        'use_lidar': True,          # LiDAR in front also counts as "bar down"
        'lidar_x': 0.35,            # m ahead
        'lidar_half_width': 0.10,
        'lidar_points': 3,
        'approach_max_m': 2.0,      # no bar within this -> done (abort)
        'max_wait_s': 25.0,         # bar still "down" this long: the camera rule is stuck (a bar lifts in a
                                    # few seconds). Do not drive through (sensor 2 at 6 cm = fail): creep at
                                    # creep_v while the LiDAR sees nothing in front, which also keeps the
                                    # robot moving for the 30 s no-motion rule. LiDAR in front still stops it
        'creep_v': 0.02,
        'creep_lidar_x': 0.20,      # LiDAR box ahead while creeping (shorter than the approach box)
        'go_m': 0.35,
    }

    def start(self, ctx):
        Mission.start(self, ctx)
        self.down, self.up = _FrameCounter(), _FrameCounter()
        self.lidar_n = 0
        self._enter('approach', ctx)

    def _lidar_down(self, ctx, x=None):
        if not self.p['use_lidar'] or ctx.scan is None:
            return False
        hw = self.p['lidar_half_width']
        return ctx.scan.count_box(0.05, self.p['lidar_x'] if x is None else x, -hw, hw) >= self.p['lidar_points']

    def step(self, ctx):
        if self.phase == 'approach':
            cam = self.down.update(ctx.bar_seq, ctx.bar == 'down')
            self.lidar_n = self.lidar_n + 1 if self._lidar_down(ctx) else 0
            if cam >= self.p['down_frames'] or self.lidar_n >= 2:
                self._enter('wait', ctx)
                return Outputs(drive='stop')
            if self.phase_dist(ctx) > self.p['approach_max_m']:
                self._finish('abort', 'no bar')
                return Outputs()
            return Outputs(slow=True)
        if self.phase == 'wait':
            clear = ctx.bar != 'down' and not self._lidar_down(ctx)
            if self.up.update(ctx.bar_seq, clear) >= self.p['up_frames']:
                self._enter('go', ctx)
                return Outputs(slow=True, reset=True)
            if ctx.t - self.phase_t > self.p['max_wait_s']:
                self.log.append('level_crossing: bar "down" for %.0f s, camera rule stuck -> creeping on LiDAR' % self.p['max_wait_s'])
                self._enter('creep', ctx)
            else:
                return Outputs(drive='stop')
        if self.phase == 'creep':
            if self.up.update(ctx.bar_seq, ctx.bar != 'down') >= self.p['up_frames'] or self.phase_dist(ctx) >= self.p['go_m']:
                self._enter('go', ctx)
                return Outputs(slow=True, reset=True)
            if self._lidar_down(ctx, self.p['creep_lidar_x']):
                return Outputs(drive='stop')              # something is there: hold (the robot did move)
            return Outputs(drive='mission', cmd=(self.p['creep_v'], 0.0))
        if self.phase == 'go':
            if self.phase_dist(ctx) >= self.p['go_m']:
                self._finish('ok', 'passed')
            return Outputs(slow=True)
        raise RuntimeError('level_crossing: phase %s' % self.phase)


# ------------------------------------------------------------------------------- tunnel

class Tunnel(Mission):
    '''Started by /tunnel. Dark inside, random obstacles: LiDAR + odometry only, driving to the exit.

      enter     lane until the lines are gone, walls are on both sides, or enter_max_m after the sign
      approach  straight on with the heading held until walls are on both sides (= the entrance) or
                approach_max_m. That pose is the tunnel frame: x along the entry heading, y to the left.
      goal      LiDAR builds a small occupancy grid of the tunnel while driving (gridnav.GridNav); a path
                to the target outside the exit is planned on it every replan_s (and when something new
                blocks it). The point lookahead m down that path is handed to the gap planner, which only
                keeps the robot off what the LiDAR sees right now.
      escape    no progress along the path for stall_s: right-hand wall following for escape_m, re-plan
      exit      lane lines again (once inside the exit), or the outside target reached -> lane, exit_m, done

    Exit position (exit_fwd, exit_left) in the tunnel frame, which starts where walls first show on both
    sides (Gazebo v014: 0.16 m before the inner face of the entrance wall): Gazebo exit lane centre
    2.01 m ahead, exit opening 1.95 m left. The 2025 team's Nav2 goal (their tunnel.zip) was 1.42 m ahead
    and 1.86 m left of their start somewhere inside the entrance, heading +93 deg: same arrangement
    (exit on the left, leaving at +90). Measure the real tunnel and set both; a plan only needs the
    opening within about +-0.25 m, the grid finds the opening itself.

    Why a grid and not a reactive rule: wall following has no target and circles a large obstacle away
    from the walls; steering at each tick toward the target walks into pockets that face the target.
    2D sim, random passable layouts: see test_sim.py.'''
    name = 'tunnel'
    DEFAULTS = {
        'lost_updates': 3,          # lane status not good this many messages -> approach
        'enter_max_m': 0.5,         # or this far after the sign
        'wall_both': 0.35,          # walls on both sides closer than this = in the entrance
        'approach_max_m': 0.6,      # straight on at most this far looking for the entrance
        'approach_v': 0.06,
        'exit_fwd': 2.01,           # m ahead of the tunnel frame origin to the exit lane centre
        'exit_left': 1.95,          # m left of the entry line to the exit opening (negative = right)
        'inside_m': 0.30,           # "inside the exit" from this far before the opening
        'outside_m': 0.40,          # target this far beyond the opening
        'reach_m': 0.15,            # target reached within this
        'replan_s': 1.0,
        'scan_gate_deg': 4.0,       # scan into the grid only if the robot turned less than this since the
                                    # previous one: an LDS sweep takes 0.2 s (5 Hz), so a scan taken while
                                    # turning is smeared, and the grid keeps every hit for good
        'lookahead': 0.25,          # m down the planned path handed to the gap planner
        'stall_s': 10.0,            # no progress along the path this long -> escape
        'stall_progress': 0.05,     # m of cost-to-go
        'escape_m': 0.6,            # wall following this far, then plan again
        'good_updates': 4,          # lane good this many messages (inside the exit) -> exit
        'max_s': 120.0,
        'exit_m': 0.15,
    }
    WALL_KEYS = ('side', 'd_wall', 'v', 'k_d', 'k_a', 'front_slow', 'front_turn', 'front_stop', 'w_turn',
                 'w_seek', 'gap_straight_m', 'w_max')
    GAP_KEYS = tuple(GAP_DEFAULTS)
    GRID_KEYS = tuple(GRID_DEFAULTS)

    def __init__(self, p=None):
        p = dict(p or {})
        def sub(prefix, keys):
            return dict((k[len(prefix):], p.pop(k)) for k in list(p) if k.startswith(prefix) and k[len(prefix):] in keys)
        wall, gap, grid = sub('wall_', self.WALL_KEYS), sub('gap_', self.GAP_KEYS), sub('grid_', self.GRID_KEYS)
        Mission.__init__(self, p)
        self.wall_p, self.gap_p, self.grid_p = wall, gap, grid

    def start(self, ctx):
        Mission.start(self, ctx)
        self.follower = WallFollower(self.wall_p)
        self.planner = GapPlanner(self.gap_p)
        self.bad, self.good = _FrameCounter(), _FrameCounter()
        self.frame, self.targets, self.k, self.grid = None, [], 0, None
        self.last_scan, self.escapes, self.plans = None, 0, 0
        self.scan_yaw, self.skipped = None, 0
        self._enter('enter', ctx)

    # ---- tunnel frame -----------------------------------------------------------------

    def _set_frame(self, ctx):
        p = self.p
        self.frame = (ctx.x, ctx.y, ctx.yaw)
        sgn = 1.0 if p['exit_left'] >= 0 else -1.0
        self.targets = [self._world(p['exit_fwd'], p['exit_left'] - sgn * p['inside_m']),
                        self._world(p['exit_fwd'], p['exit_left'] + sgn * p['outside_m'])]
        self.k = 0
        self.grid = GridNav(self.frame, p['exit_fwd'], abs(p['exit_left']), sgn, self.grid_p)
        self.log.append('tunnel: frame (%.2f, %.2f, %.0f deg), exit target (%.2f, %.2f)' % (
            ctx.x, ctx.y, math.degrees(ctx.yaw), self.targets[1][0], self.targets[1][1]))

    def _world(self, fwd, left):
        x0, y0, th = self.frame
        return (x0 + fwd * math.cos(th) - left * math.sin(th), y0 + fwd * math.sin(th) + left * math.cos(th))

    def _local(self, ctx):
        '''Robot position in the tunnel frame: (ahead of the entrance, left of the entry line).'''
        x0, y0, th = self.frame
        dx, dy = ctx.x - x0, ctx.y - y0
        return dx * math.cos(th) + dy * math.sin(th), -dx * math.sin(th) + dy * math.cos(th)

    def _target(self, ctx):
        tx, ty = self.targets[self.k]
        dx, dy = tx - ctx.x, ty - ctx.y
        return math.hypot(dx, dy), wrap(math.atan2(dy, dx) - ctx.yaw)

    def _start_goal(self, ctx):
        self._enter('goal', ctx)
        self.plan_t = -1e9
        self.best, self.best_t = INF, ctx.t

    def _progress(self, ctx):
        '''m still to go along the planned path (straight distance to the target before the first plan).'''
        c = self.grid.cost_to_go(ctx.x, ctx.y)
        return c * self.grid.res if c < INF else self._target(ctx)[0]

    def _map(self, ctx):
        '''New LiDAR scan into the grid; re-plan every replan_s or when the path is blocked now.'''
        if ctx.scan is None or ctx.scan is self.last_scan:
            return
        self.last_scan = ctx.scan
        g = self.grid
        turned = self.scan_yaw is not None and abs(wrap(ctx.yaw - self.scan_yaw)) > math.radians(self.p['scan_gate_deg'])
        self.scan_yaw = ctx.yaw
        if turned:
            self.skipped += 1                   # smeared: not into the grid (the gap planner still uses it)
        else:
            g.add_scan(ctx.scan, (ctx.x, ctx.y, ctx.yaw))
        if self.phase == 'goal' and (ctx.t - self.plan_t >= self.p['replan_s']
                                     or (not turned and g.path_blocked(ctx.x, ctx.y, self.p['lookahead']))):
            g.plan(self.targets[1])
            self.plan_t = ctx.t
            self.plans += 1
            now = self._progress(ctx)
            if now > self.best + 0.15:          # the map changed (a new obstacle): fresh chance
                self.best, self.best_t = now, ctx.t

    def _drive(self, ctx, dist, bearing):
        g = self.grid
        nxt = g.next_point(ctx.x, ctx.y, self.p['lookahead'])
        if nxt is None:                                          # no path on the grid: head for the target
            return self.planner.step(ctx.scan, bearing, dist)
        dx, dy = nxt[0] - ctx.x, nxt[1] - ctx.y
        b = wrap(math.atan2(dy, dx) - ctx.yaw)
        return self.planner.step(ctx.scan, b, dist, need=max(math.hypot(dx, dy), 0.15))

    # ---- main ---------------------------------------------------------------------------

    def step(self, ctx):
        p = self.p
        walls = ctx.scan is not None and ctx.scan.sector_min(60, 120) < p['wall_both'] \
            and ctx.scan.sector_min(-120, -60) < p['wall_both']
        if self.phase == 'enter':
            bad = self.bad.update(ctx.lane_seq, ctx.lane_status not in GOOD_LANE)
            if walls:
                self._set_frame(ctx)
                self._start_goal(ctx)
            elif bad >= p['lost_updates'] or self.phase_dist(ctx) >= p['enter_max_m']:
                self._enter('approach', ctx)
            else:
                return Outputs()
        if self.phase == 'approach':
            ahead = ctx.scan is not None and ctx.scan.sector_min(-25.0, 25.0) < 0.25   # something right ahead
            if walls or ahead or self.phase_dist(ctx) >= p['approach_max_m']:
                self._set_frame(ctx)
                self._start_goal(ctx)
            else:
                cmd, _ = drive_cmd(self.phase_dist(ctx), p['approach_max_m'], ctx.yaw, self.phase_yaw,
                                   p['approach_v'])
                return Outputs(drive='mission', cmd=cmd)
        if self.phase == 'exit':
            if self.phase_dist(ctx) >= p['exit_m']:
                self._finish('ok', 'out (%d plans, %d escapes, %d turning scans skipped)' % (
                    self.plans, self.escapes, self.skipped))
            return Outputs()

        # goal / escape
        if ctx.t - self.t0 > p['max_s']:
            self._enter('exit', ctx)
            self.log.append('tunnel: max_s, handing back to the lane')
            return Outputs(reset=True)
        self._map(ctx)
        fwd, left = self._local(ctx)
        sgn = 1.0 if p['exit_left'] >= 0 else -1.0
        if self.k == 0 and sgn * left >= abs(p['exit_left']) - p['inside_m'] and abs(fwd - p['exit_fwd']) < 0.2:
            self.k = 1                                           # inside the exit opening
        dist, bearing = self._target(ctx)
        good = self.good.update(ctx.lane_seq, ctx.lane_status in GOOD_LANE)
        if self.k == 1 and (good >= p['good_updates'] or self._target(ctx)[0] < p['reach_m']
                            or sgn * left >= abs(p['exit_left']) + p['outside_m']):
            self._enter('exit', ctx)
            return Outputs(reset=True)
        if ctx.scan is None:
            return Outputs(drive='mission')
        if self.phase == 'escape':
            if self.phase_dist(ctx) >= p['escape_m']:
                self._start_goal(ctx)
                self._map(ctx)
            else:
                return Outputs(drive='mission', cmd=self.follower.step(ctx.scan, ctx.travel))
        now = self._progress(ctx)
        if now < self.best - p['stall_progress']:
            self.best, self.best_t = now, ctx.t
        elif ctx.t - self.best_t > p['stall_s']:
            self.escapes += 1
            self.follower = WallFollower(self.wall_p)
            self._enter('escape', ctx)
            self.log.append('tunnel: no progress for %.0f s, wall following %.1f m' % (p['stall_s'], p['escape_m']))
            return Outputs(drive='mission', cmd=self.follower.step(ctx.scan, ctx.travel))
        return Outputs(drive='mission', cmd=self._drive(ctx, dist, bearing))

    def status(self):
        if self.phase in ('goal', 'escape'):
            mode = self.planner.mode if self.phase == 'goal' else self.follower.mode
            return '%s/%s%s %s plans=%d' % (self.name, self.phase, ' in-exit' if self.k else '', mode, self.plans)
        return '%s/%s' % (self.name, self.phase)
