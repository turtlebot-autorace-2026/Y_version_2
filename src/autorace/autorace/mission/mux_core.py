'''
cmd_vel mux core (no ROS, unit-testable).

Exactly one source reaches the robot: the one named by the latest drive mode (lane | mission | stop).
Stop (zero command) when:
  - the mode is 'stop',
  - the selected source has been silent for source_timeout,
  - no drive mode has arrived for mode_timeout (the FSM died).
'''

ZERO = (0.0, 0.0)


class CmdVelMux(object):
    SOURCES = ('lane', 'mission')

    def __init__(self, source_timeout=0.5, mode_timeout=1.0):
        self.source_timeout = source_timeout
        self.mode_timeout = mode_timeout
        self.mode, self.mode_t = 'stop', None
        self.last = {s: (ZERO, None) for s in self.SOURCES}

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
            return (v, w)
        return None

    def _mode_alive(self, t):
        return self.mode_t is not None and t - self.mode_t <= self.mode_timeout

    def tick(self, t):
        '''Periodic check. Returns (command or None, reason). A command means: publish it now.'''
        if not self._mode_alive(t):
            return ZERO, 'no drive mode from the FSM'
        if self.mode == 'stop':
            return ZERO, 'mode stop'
        _, ts = self.last[self.mode]
        if ts is None or t - ts > self.source_timeout:
            return ZERO, '%s silent' % self.mode
        return None, 'ok'
