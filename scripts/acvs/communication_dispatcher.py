import actionlib

from proteus_msgs.srv import SymbolDirectional, SymbolQuantity, SymbolTrigger
from proteus_msgs.msg import CommunicateToDiverAction

class CommunicationDispatcher(object):
    def __init__(self):
        self.server = actionlib.SimpleActionServer('communicate_to_diver', CommunicateToDiverAction, self.execute, False)

        self.server.start()

    def execute(self, goal):

        self.server.set_succeeded()