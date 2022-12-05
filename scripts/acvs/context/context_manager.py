from acvs.context import DiverContext, EnvironmentContext, TaskContext, MissionContext

class ContextManager(object):
    storage_fp = '~/.proteus/context/'
    def __init__(self):
        self.divers = []
        self.environment = None
        self.task = None
        self.mission = None

    def report_diver(self, diver):
        pass
    
    def report_environment(self, env):
        pass

    def report_task(self, task):
        pass

    def report_mission(self, mission):
        pass
    