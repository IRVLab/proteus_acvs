import actionlib

from proteus_msgs.srv import SymbolDirectional, SymbolQuantity, SymbolTrigger
from proteus_msgs.msg import CommunicateToDiverAction


class CommunicationDispatcher(object):
    def __init__(self, select_vector_fp):
        self.server = actionlib.SimpleActionServer('communicate_to_diver', CommunicateToDiverAction, self.execute, False)
        self.select_vector = select_vector_fp
        self.server.start()

    def execute(self, goal):
        
        self.select_vector()
        self.server.set_succeeded()