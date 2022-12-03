import rospy, actionlib

from proteus_msgs.srv import SymbolDirectional, SymbolQuantity, SymbolTrigger
from proteus_msgs.msg import CommunicateToDiverAction


class CommunicationDispatcher(object):
    def __init__(self, select_vector_fp):
        self.server = actionlib.SimpleActionServer('communicate_to_diver', CommunicateToDiverAction, self.execute, False)
        self.select_vector = select_vector_fp
        self.server.start()

    def execute(self, goal):
        rospy.loginfo(f"Selecting vectors for content: \"{goal.content}\"")
        selections = self.select_vector(goal)
        
        rospy.loginfo(f"Got {len(selections)} selected vectors by policy.")
        for vec in selections:
            rospy.loginfo(f"Using {vec}")
        
        self.server.set_succeeded()