"""Reuse an acknowledged final target only while the same controller still holds it."""
import math


class CompletedHold:
    def __init__(self, names):
        self.names = names
        self.saved = None

    @staticmethod
    def identity(snapshot):
        state = snapshot.get('controller')
        age = snapshot.get('state_age_s')
        if (state is None or type(age) not in (int, float) or not 0 <= age <= .2
                or state['state'] != 0 or state['last_result'] != 0 or state['stop_reason'] != 6):
            return None
        return state['epoch'], state['last_sequence']

    def capture(self, entry, stream, snapshot):
        self.saved = None
        identity = self.identity(snapshot)
        if (identity != (stream.epoch, stream.sequence) or snapshot.get('phase') != 3
                or not stream.finished or not stream.final_sent or stream.pending is not None
                or not stream.committed):
            return
        last = stream.committed[-1]
        if (len(last.q) != len(self.names) or not all(math.isfinite(q) for q in last.q)
                or any(v != 0 for v in last.v)):
            return
        self.saved = (entry, identity, dict(zip(self.names, last.q)))

    def take(self, entry, snapshot):
        saved, self.saved = self.saved, None
        if saved is not None and saved[0] is entry and self.identity(snapshot) == saved[1]:
            return saved[2]
        return None
