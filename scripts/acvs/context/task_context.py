from acvs.context import Context

class TaskContext(Context):
    
    def __init__(self, task_id):
        super().__init__(self)
        self.task_id = task_id

    def get_current_belief(self):
        raise NotImplementedError("Not implemented in base class, try an instantiation")

    def get_last_observation(self):
        raise NotImplementedError("Not implemented in base class, try an instantiation")

    def report_observation(self):
        raise NotImplementedError("Not implemented in base class, try an instantiation")