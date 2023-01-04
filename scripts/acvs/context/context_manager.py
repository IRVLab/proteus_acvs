import rospy
from proteus_msgs.msg import DiverGroup

from acvs.context.diver_context import DiverContext

class ContextManager(object):
    _storage_fp = '~/.proteus/context/'
    _diver_timeout = 10.0

    def __init__(self, diver_topic, env_topics, task_topics, mission_topics):
        self.divers = {}
        self.environment = None
        self.task = None
        self.mission = None

        # We're only implementing diver context here because it's 1/3/23
        self.diver_sub = rospy.Subscriber(diver_topic, DiverGroup, self.report_divers)
    
    def update_context(self):
        # First, check the diver list and cull any who have timed out.
        ids = []
        for key in self.divers.copy():
            diver = self.divers[key]
            if (not diver.currently_seen) and ((rospy.Time.now() - diver.last_seen).to_sec() > ContextManager._diver_timeout):
                self.divers.pop(key)
            else:
                ids.append(key)

        rospy.logdebug(f"ContextManager currently knows of {len(self.divers)} divers, ids: {ids}")

    def select_interactant(self, selection_method="combination_score", **kwargs):
        if selection_method == "combination_score":
            max_score = 0
            selected = None
            now = rospy.Time.now()

            for _, dc in self.divers.items():
                sc = 0
                if dc.currently_seen:
                    dist_term = 0.001 if (5.0 - dc.pseudodistance) < 0 else (5.0 - dc.pseudodistance)
                    conf_term = dc.confidence
                    recency_term = (now - dc.last_seen).to_sec()
                    sc = dist_term * conf_term * recency_term

                if sc > max_score:
                    max_score = sc
                    selected= dc

            return selected
            
        elif selection_method == "min_distance":
            pd = 10.0
            closest = None
            for _, dc in self.divers.items():
                if dc.currently_seen and dc.pseudodistance < pd:
                    pd = dc.pseudodistance
                    closest = dc

            return closest

        elif selection_method == "max_confidence":
            conf = 0.0
            most_confident = None
            for _, dc in self.divers.items():
                if dc.currently_seen and dc.confidence > conf:
                    conf = dc.confidence
                    most_confident = dc

            return most_confident
            
        elif selection_method == "min_recency":
            time = rospy.Time.from_sec(0)
            most_recent = None
            for _, dc in self.divers.items():
                if dc.currently_seen and dc.last_seen > time:
                    time = dc.last_seen
                    most_recent = dc

            return most_recent

        elif selection_method == "known_id":
            id = kwargs['id']
            if id in self.divers.keys():
                return self.divers[id]
            else:
                return None

        else:
            raise ValueError("Unknown interactant selection_method")

    def report_divers(self, msg):
        # First, we have to process the incoming list of divers.
        for diver in msg.divers:
            id = diver.diver_id
            last_seen = diver.last_seen
            visible = diver.currently_seen
            conf = diver.estimated_confidence
            drp = diver.relative_position
            center_point = [drp.center_point_rel.x, drp.center_point_rel.y]
            pseudodistance = drp.distance.distance_ratio

            if id in self.divers.keys():
                self.divers[id].report_observation(visible, last_seen, conf, center_point, pseudodistance)
            elif visible:
                self.divers[id] = DiverContext(id, last_seen, conf, center_point, pseudodistance)
        
    
    def report_environment(self, source, msg):
        pass

    def report_task(self, source, task):
        pass

    def report_mission(self, source, mission):
        pass
    