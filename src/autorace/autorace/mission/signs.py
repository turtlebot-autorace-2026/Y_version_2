'''
Sign states from the senior's mission_model node (lane_detection_dl/mission_model).

mission_model publishes std_msgs/Bool, latched (transient local), ONLY WHEN A STATE CHANGES:
True after 3 frames in a row with a close box (box >= 10 % of the image height), False after
5 frames in a row without it. So a sign is a level (active between True and False), not a
stream of True messages.

  topic          sign name used here      mission
  /parking       parking                  parking
  /left /right   left right               intersection direction
  /no_entry      no_entry                 (not used)
  /stop          stop                     level crossing
  /construction  construction             construction
  /intersection  intersection             intersection (arms left/right)
  /tunnel        tunnel                   tunnel
'''

SIGN_TOPICS = (('parking', '/parking'), ('left', '/left'), ('right', '/right'), ('no_entry', '/no_entry'),
               ('stop', '/stop'), ('construction', '/construction'), ('intersection', '/intersection'),
               ('tunnel', '/tunnel'))
SIGNS = tuple(n for n, _ in SIGN_TOPICS)


class SignBoard(object):
    def __init__(self):
        self.active = dict((n, False) for n in SIGNS)
        self.last_active_travel = dict((n, None) for n in SIGNS)   # odometry travel while last active
        self.rises = []                                             # (name, t, travel) not yet consumed

    def set(self, name, value, t, travel):
        if name not in self.active:
            raise ValueError('unknown sign %r' % name)
        value = bool(value)
        if value and not self.active[name]:
            self.rises.append((name, t, travel))
        if value or self.active[name]:
            self.last_active_travel[name] = travel
        self.active[name] = value

    def touch(self, travel):
        '''Called every tick: an active sign is "seen" at the current travel.'''
        for n in SIGNS:
            if self.active[n]:
                self.last_active_travel[n] = travel

    def since_seen(self, name, travel):
        '''m driven since the sign was last active (0 while active, None if never seen).'''
        last = self.last_active_travel[name]
        return None if last is None else (0.0 if self.active[name] else travel - last)

    def pop_rises(self):
        out, self.rises = self.rises, []
        return out

    def summary(self):
        on = [n for n in SIGNS if self.active[n]]
        return ' '.join(on) or '-'
