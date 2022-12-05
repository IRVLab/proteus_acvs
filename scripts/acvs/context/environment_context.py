from acvs.context import Context

class EnvironmentContext(Context):
    
    def __init__(self, env_id , type, vis, amb, rest):
        super().__init__(self)
        self.environment_id = env_id
        self.type = type 
        self.visibilty = vis
        self.ambient_noise = amb
        self.restrictions = rest

    def get_current_belief(self):
        return (self.type, self.visibilty, self.ambient_noise, self.restrictions)

    def get_last_observation(self):
        return (self.visibilty, self.ambient_noise)

    def report_observation(self, visibility, ambient):
        self.visibilty = visibility
        self.ambient_noise = ambient
        