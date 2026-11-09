"""Bounded timing samples and explicit, staged Rolling timing profiles."""
from collections import deque
import math

PROFILES = {'baseline': (120, 100), 'trial': (100, 80)}


class Latency:
    def __init__(self, capacity=256):
        self.capacity = capacity
        self.samples = {}
        self.counts = {}

    def add(self, name, milliseconds):
        if not math.isfinite(milliseconds) or milliseconds < 0:
            return
        self.samples.setdefault(name, deque(maxlen=self.capacity)).append(float(milliseconds))
        self.counts[name] = self.counts.get(name, 0) + 1

    def snapshot(self):
        result = {}
        for name, values in self.samples.items():
            ordered = sorted(values)
            result[name] = {'count': self.counts[name], 'retained': len(ordered),
                            'last': round(values[-1], 3), 'max': round(ordered[-1], 3)}
            for percentile in (50, 95, 99):
                result[name][f'p{percentile}'] = round(ordered[math.ceil(len(ordered) * percentile / 100) - 1], 3)
        return result

    def assessment(self, parameters, profile):
        stats = self.snapshot()
        required = ('window_plan', 'window_rpc', 'ack', 'loop_interval')
        enough = ('replace_lead_ms' in parameters and
                  all(stats.get(k, {}).get('retained', 0) >= 30 for k in required))
        # Use observed maxima as well as P99; an isolated long stall must not be
        # hidden by quantiles. This is advice, never an automatic runtime change.
        # ACK includes the publication RPC and the next observation. Count it
        # once; only excess scheduling delay is added to that measured interval.
        budget = (stats['window_plan']['max'] + max(stats['window_rpc']['max'], stats['ack']['max'])
                  + max(0., stats['loop_interval']['max'] - 1000 / 30)
                  + parameters['replace_lead_ms'] if enough else None)
        fits = budget is not None and budget < PROFILES['trial'][1]
        return {'profile': profile, 'initial_lead_ms': PROFILES[profile][0],
                'replacement_lead_ms': PROFILES[profile][1], 'sample_ready': enough,
                'observed_budget_ms': round(budget, 3) if budget is not None else None,
                'trial_fits_observed_budget': fits, 'automatic_change': False}
