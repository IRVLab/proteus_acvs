#! /usr/bin/python3

import rospy
import actionlib

from proteus.symbol import Symbol
from proteus.vector import Vector

from proteus_msgs.msg import DiverGroup, Diver
from proteus_msgs.srv import SymbolDirectional, SymbolQuantity, SymbolTrigger

from acvs.communication_dispatcher import CommunicationDispatcher
from acvs.policies.communication_policy import CommunicationPolicy
from acvs.policies.static_policy import StaticPolicy
from acvs.policies.random_policy import RandomPolicy


class ACVSNode(object):
    def __init__(self):
        rospy.init_node("acvs_muxer", anonymous=False)

        # Set up a context manager with topic subscriptions to the appropriate topics.
        #self.context = ContextManager()

        # Pull in rosparams and build langauge info out of them.
        vector_params = rospy.get_param('/loco/proteus/vectors/out')
        vectors = []
        for key,val in vector_params.items():
            v = Vector('out')
            v.parse_from_rosparam(key, val)
            vectors.append(v)

        symbol_params = rospy.get_param('/loco/proteus/symbols/out')
        symbols = []    
        for key,val in symbol_params.items():
            s = Symbol('out')
            s.parse_from_rosparam(key, val)
            symbols.append(s)

        print(vectors)
        print(symbols)

        # Pull in configuration parameters for ACVS via rosparam
        min_v = None
        max_v = None
        self.policy = RandomPolicy((vectors, symbols), min_v, max_v)

        # Next, build up a data structure with service proxies.
        # static_symbols {symbol: {vector: ServiceProxy } }
        # dynamic_vectors { vector: publicationFunction }

        # Next, let's build our communication dispatcher
        self.dispatcher = CommunicationDispatcher(self.execute(self))




    def execute(self, goal):
        # This gets called from the Communication dispatcher. 
        # Why didn't I just make ACVSNode a child of SimpleActionServer....who knows

        print("I'm in the ACVS Node")
        print(type(self))



if __name__ == "__main__":
    acvs = ACVSNode()
    rospy.spin()