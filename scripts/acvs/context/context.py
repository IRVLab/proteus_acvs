import uuid

class Context(object):
    def __init__(self):
        self.id = uuid.uuid1()

    def get_current_belief(self):
        raise NotImplementedError("Not implemented in base class, try an instantiation")

    def get_last_observation(self):
        raise NotImplementedError("Not implemented in base class, try an instantiation")

    def report_observation(self):
        raise NotImplementedError("Not implemented in base class, try an instantiation")