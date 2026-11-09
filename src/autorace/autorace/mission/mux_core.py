'''
cmd_vel mux core (no ROS, unit-testable).

Exactly one source reaches the robot: the one named by the latest drive mode (lane | mission | stop).
Stop (zero command) when:
  - the mode is 'stop',
  - the selected source has been silent for source_timeout,
  - no drive mode has arrived for mode_timeout (the FSM died).
Start ramp: after a stop (zero command, mode stop, silence) the forward speed may rise by at most
accel m/s^2, and the angular command is scaled with it so the turning radius stays the one the source
asked for (idea from the 2025 team's driving node, there 0.03 m/s per frame). The lane estimator is
restarted on every hand-back and would otherwise command full speed from the first frame.
'''

ZERO = (0.0, 0.0)


class CmdVelMux(object):
    SOURCES = ('lane', 'mission')

    def __init__(self, source_timeout=0.5, mode_timeout=1.0, accel=0.3):
        self.source_timeout = source_timeout
        self.mode_timeout = mode_timeout
        self.accel = accel                  # m/s^2, 0 = no ramp
        self.mode, self.mode_t = 'stop', None
        self.last = {s: (ZERO, None) for s in self.SOURCES}
        self.out_v, self.out_t = 0.0, None  # last forward speed sent to the robot

    def set_mode(self, mode, t):
        if mode not in self.SOURCES + ('stop',):
            raise ValueError('unknown drive mode %r' % mode)
        changed = mode != self.mode
        self.mode, self.mode_t = mode, t
        return changed

    def on_cmd(self, source, v, w, t):
        '''Returns the command to publish now, or None if this source is not selected.'''
        self.last[source] = ((v, w), t)
        if self.mode == source and self._mode_alive(t):
            return self._ramp(v, w, t)
        return None

    def _ramp(self, v, w, t):
        '''Limit the rise of the forward speed since the last published command.'''
        if self.accel > 0 and self.out_t is not None and v > 0.0:
            limit = max(self.out_v, 0.0) + self.accel * max(t - self.out_t, 0.0)
            if v > limit:
                w = w * limit / v
                v = limit
        self.out_v, self.out_t = v, t
        return (v, w)

    def _mode_alive(self, t):
        return self.mode_t is not None and t - self.mode_t <= self.mode_timeout

    def tick(self, t):
        '''Periodic check. Returns (command or None, reason). A command means: publish it now.'''
        if not self._mode_alive(t):
            return self._zero(t), 'no drive mode from the FSM'
        if self.mode == 'stop':
            return self._zero(t), 'mode stop'
        _, ts = self.last[self.mode]
        if ts is None or t - ts > self.source_timeout:
            return self._zero(t), '%s silent' % self.mode
        return None, 'ok'

    def _zero(self, t):
        self.out_v, self.out_t = 0.0, t
        return ZERO
