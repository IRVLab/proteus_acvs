from acvs.context import Context

class MissionContext(Context):
    
    def __init__(self, mission_id):
        super().__init__()
        self.mission_id = mission_id

    def get_current_belief(self):
        raise NotImplementedError("Not implemented in base class, try an instantiation")

    def get_last_observation(self):
        raise NotImplementedError("Not implemented in base class, try an instantiation")

    def report_observation(self):
        raise NotImplementedError("Not implemented in base class, try an instantiation")