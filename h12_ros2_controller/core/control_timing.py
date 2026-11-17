import math
import time


class ControlTiming:
    '''Pace fixed-dt control on an injected clock with wall-clock safety polling'''

    def __init__(self, now=None):
        self._now = time.monotonic if now is None else now

    def now(self):
        value = float(self._now())
        if not math.isfinite(value):
            raise ValueError('Control clock must be finite')
        return value

    def wait(self, started, period, *, check, cancel_check=None, wall_deadline):
        '''Return False on cancellation/deadline without advancing another tick'''
        previous = started
        while True:
            if cancel_check is not None and cancel_check():
                return False
            check()
            now = self.now()
            if now < previous:
                raise RuntimeError('Control clock moved backwards; holding last command')
            previous = now
            remaining = period - (now - started)
            wall_remaining = wall_deadline - time.monotonic()
            if wall_remaining <= 0.0:
                return False
            if remaining <= 1e-9:
                return True
            # recompute elapsed control time after every poll, including a pause
            time.sleep(min(0.01, remaining, wall_remaining))
