from acvs.context.context import Context
from math import sqrt

def dist(a, b):
    return sqrt((a[0] - b[0])**2 + (a[1] - b[1])**2)

class DiverContext(Context):
    def __init__(self, id, time_seen, conf, cp, pd):
        super().__init__()
        self.id = id
        self.currently_seen = True
        self.last_seen = time_seen
        self.confidence = conf
        self.center_point = cp
        self.pseudodistance = pd
        self.pseudoangle = dist([0.5,0.5], cp)

    def get_pseudopose(self):
        return [self.pseudodistance, self.pseudoangle]

    def get_current_belief(self):
        return (self.currently_seen, self.last_seen, self.relative_position)

    def get_last_observation(self):
        return (self.currently_seen, self.last_seen, self.relative_position)

    def report_observation(self, seen, time_seen, conf,cp, pd):
        self.currently_seen = seen
        self.last_seen = time_seen
        self.confidence = conf
        self.center_point = cp
        self.pseudodistance = pd
        self.pseudoangle = dist([0.5,0.5], cp)

        