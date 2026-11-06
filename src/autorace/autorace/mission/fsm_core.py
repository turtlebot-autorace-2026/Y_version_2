'''
Mission FSM core (no ROS, unit-testable).

  START (traffic light, or go) -> LANE <-> one mission at a time

A mission starts on the rising edge of its sign from mission_model (see signs.py):

  /intersection -> intersection     /construction -> construction     /parking -> parking
  /stop         -> level_crossing   /tunnel       -> tunnel

Order is not fixed (the layout may change on the day). A sign that rises while another mission
runs is kept as pending and starts when that one ends, if the robot has not driven more than
pending_m since. Each mission runs once; an aborted one (e.g. intersection without a direction
sign) may start again. /left /right never start anything on their own: they only count inside the
intersection mission, so a left/right sign seen anywhere else (near the start, the parking-area
left arrow) does nothing. Exception for practice maps without the triangle sign:
intersection_on_turn_sign = true makes /left /right start the intersection too.
'''
from autorace.mission.missions import (Construction, Intersection, LevelCrossing, Outputs, Parking, Tunnel,
                                       TrafficLight)

TRIGGERS = (('intersection', 'intersection'), ('construction', 'construction'), ('parking', 'parking'),
            ('stop', 'level_crossing'), ('tunnel', 'tunnel'))
CLASSES = {'traffic_light': TrafficLight, 'intersection': Intersection, 'construction': Construction,
           'parking': Parking, 'level_crossing': LevelCrossing, 'tunnel': Tunnel}
DEFAULTS = {
    'start_mode': 'light',          # light = wait for green | go = drive immediately
    'missions': ['intersection', 'construction', 'parking', 'level_crossing'],   # tunnel: add to enable
    'pending_m': 1.0,
    # /left or /right also start the intersection mission. Only for a practice map without the
    # triangle sign: on the course, the parking-area left arrow would start it too. Keep false there.
    'intersection_on_turn_sign': False,
}


class MissionFSM(object):
    def __init__(self, params=None, mission_params=None):
        '''params: DEFAULTS keys. mission_params: {mission name: {param: value}}.'''
        self.p = dict(DEFAULTS)
        for k, v in (params or {}).items():
            if k not in DEFAULTS:
                raise KeyError('unknown parameter %s' % k)
            self.p[k] = v
        if self.p['start_mode'] not in ('light', 'go'):
            raise ValueError('start_mode must be light or go')
        mp = mission_params or {}
        for name in mp:
            if name not in CLASSES:
                raise KeyError('unknown mission %s' % name)
        for name in self.p['missions']:
            if name not in CLASSES or name == 'traffic_light':
                raise ValueError('unknown mission %s in missions' % name)
        self.m = dict((name, cls(mp.get(name))) for name, cls in CLASSES.items())
        self.state = 'START'
        self.active = None
        self.finished = {}            # name -> result
        self.pending = {}             # name -> travel when its sign rose
        self.events = []
        self.started = False

    def _log(self, text):
        self.events.append(text)

    def _start(self, name, ctx):
        self.pending.pop(name, None)
        self.active = self.m[name]
        self.active.start(ctx)
        self.state = 'MISSION'
        self._log('start %s' % name)

    def vision(self):
        '''Camera check the node must run now: light | bar | None.'''
        return self.active.vision if self.active is not None else None

    def triggers(self):
        if self.p['intersection_on_turn_sign']:
            return TRIGGERS + (('left', 'intersection'), ('right', 'intersection'))
        return TRIGGERS

    def step(self, ctx):
        for name, t, travel in ctx.signs.pop_rises():
            for sign, mission in self.triggers():
                if sign == name and mission in self.p['missions'] and self.finished.get(mission) != 'ok':
                    if self.active is None or self.active.name != mission:
                        self.pending[mission] = travel
                        self._log('/%s -> %s pending' % (sign, mission))
        if not self.started:
            self.started = True
            if self.p['start_mode'] == 'light':
                self._start('traffic_light', ctx)
            else:
                self.state = 'LANE'
                self._log('start_mode go -> LANE')
                return self._collect(Outputs(reset=True))

        if self.state == 'MISSION':
            out = self.active.step(ctx)
            if self.active.done:
                self.finished[self.active.name] = self.active.result
                self.pending.pop(self.active.name, None)
                self.active, self.state = None, 'LANE'
                if out.drive == 'lane' and out.follow_line == 'both' and out.offset == 0.0 and not out.slow:
                    nxt = self._next(ctx)   # pending, or a sign still in view
                    if nxt is not None:
                        self._start(nxt, ctx)
                        out2 = self.active.step(ctx)
                        out2.reset = out2.reset or out.reset
                        return self._collect(out2)
            return self._collect(out)

        nxt = self._next(ctx)
        if nxt is not None:
            self._start(nxt, ctx)
            return self._collect(self.active.step(ctx))
        return self._collect(Outputs())

    def _still_visible(self, ctx):
        '''Missions never run whose sign is active right now: their rising edge may have come during
        another mission and expired from pending while the sign stayed in view.'''
        for sign, mission in self.triggers():
            if mission in self.p['missions'] and mission not in self.finished and ctx.signs.active.get(sign):
                return mission
        return None

    def _next(self, ctx):
        for name in list(self.pending):
            if ctx.travel - self.pending[name] > self.p['pending_m']:
                self._log('%s pending dropped (%.2f m since its sign)' % (name, ctx.travel - self.pending[name]))
                del self.pending[name]
        if not self.pending:
            return self._still_visible(ctx)
        return min(self.pending, key=self.pending.get)

    def _collect(self, out):
        for m in self.m.values():
            self.events.extend(m.log)
            m.log = []
        return out

    def status(self):
        s = 'state=%s' % self.state
        if self.active is not None:
            s += ' ' + self.active.status()
        if self.pending:
            s += ' pending=%s' % ','.join(sorted(self.pending))
        done = ','.join('%s:%s' % kv for kv in sorted(self.finished.items()))
        return s + ' done=[%s]' % done
