"""Measurement-time interpolation without extrapolation or receive-time stamping."""
from bisect import bisect_left
import math


class JointBuffer:
    def __init__(self, history_seconds=5.0):
        self.history_seconds = history_seconds
        self.samples = []

    def add(self, stamp, angle, velocity):
        if not all(math.isfinite(value) for value in (stamp, angle, velocity)):
            return
        if self.samples and stamp < self.samples[-1][0] - self.history_seconds:
            self.samples.clear()
        index = bisect_left(self.samples, (stamp,))
        sample = (stamp, angle, velocity)
        if index < len(self.samples) and self.samples[index][0] == stamp:
            self.samples[index] = sample
        else:
            self.samples.insert(index, sample)
        cutoff = self.samples[-1][0] - self.history_seconds
        while self.samples and self.samples[0][0] < cutoff:
            self.samples.pop(0)

    def at(self, stamp):
        index = bisect_left(self.samples, (stamp,))
        if index < len(self.samples) and self.samples[index][0] == stamp:
            return self.samples[index][1:]
        if index == 0 or index == len(self.samples):
            return None
        t0, q0, w0 = self.samples[index - 1]
        t1, q1, w1 = self.samples[index]
        fraction = (stamp - t0) / (t1 - t0)
        delta = math.remainder(q1 - q0, 2 * math.pi)
        return q0 + fraction * delta, w0 + fraction * (w1 - w0)
