from acvs.context import Context

class DiverContext(Context):
    visibility_timeout = 2.0
    def __init__(self, diver_id, time_seen, rel_pos):
        super().__init__(self)
        self.diver_id = diver_id
        self.currently_seen = True
        self.last_seen = time_seen
        self.relative_position = rel_pos

    def get_current_belief(self):
        return (self.currently_seen, self.last_seen, self.relative_position)

    def get_last_observation(self):
        return (self.currently_seen, self.last_seen, self.relative_position)

    def report_observation(self, seen, time_seen, rel_pos):
        self.currently_seen = seen
        self.last_seen = time_seen
        self.relative_position = rel_pos

        