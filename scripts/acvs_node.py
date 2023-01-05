#! /usr/bin/python3

import rospy
import actionlib

from proteus.symbol import Symbol
from proteus.vector import Vector

from proteus_msgs.msg import CommunicateToDiverAction, CommunicateToDiverFeedback, CommunicateToDiverResult

from acvs.context.context_manager import ContextManager
from acvs.communication_dispatcher import CommunicationDispatcher
from acvs.policies import StaticPolicy, RandomPolicy, HeuristicPolicy, POMDPPolicy

import rospkg
rospack = rospkg.RosPack()


class ACVSNode(object):
    _feedback = CommunicateToDiverFeedback()
    _result = CommunicateToDiverResult()

    def __init__(self):
        rospy.init_node("acvs_muxer", anonymous=False)

        # Set up a context manager with topic subscriptions to the appropriate topics.
        self.context = ContextManager('/loco/proteus/context/divers', '', '','')

        # Pull in rosparams and build langauge info out of them.
        vector_params = rospy.get_param('/loco/proteus/vectors/out')
        vectors = {}
        for key,val in vector_params.items():
            v = Vector('out')
            v.parse_from_rosparam(key, val)

            # Filter out implicit vectors, because we don't have the structure to deal with them rn.
            if v.explicitness == "explicit":
                vectors[v.id] = v

        symbol_params = rospy.get_param('/loco/proteus/symbols/out')
        symbols = {}
        for key,val in symbol_params.items():
            s = Symbol('out')
            s.parse_from_rosparam(key, val)
            symbols[s.id] = s

        self.policy_type = rospy.get_param("/loco/proteus/acvs/policy/type", "static")

        if self.policy_type == "static":
            vector = rospy.get_param("/loco/proteus/acvs/policy/selected", "digital_display")
            self.policy = StaticPolicy((symbols, vectors), vector)

        elif self.policy_type == "random":
            # Pull in configuration parameters for ACVS via rosparam
            min_v = rospy.get_param("/loco/proteus/acvs/policy/min_vectors", 1)
            max_v = rospy.get_param("/loco/proteus/acvs/policy/max_vectors", 3)
            self.policy = RandomPolicy((symbols, vectors), min_v, max_v)

        elif self.policy_type == "heuristic":
            heuristic_fp = rospack.get_path('proteus_acvs') + '/' + rospy.get_param("/loco/proteus/acvs/policy/rules_file", "hueristics/basic.yaml")
            self.policy = HeuristicPolicy((symbols, vectors), heuristic_fp)

        elif self.policy_type == "pomdp":
            pass

        else:
            rospy.logerr(f"Policy type [{self.policy_type}] unrecognized")
            raise NotImplemented(f"Policy type [{self.policy_type}] unrecognized")

        # Next, build up a data structure with service proxies.
        # static_symbols {symbol: {vector: ServiceProxy } }
        # dynamic_vectors { vector: publicationFunction }

        vector_endpoints = {}
        base_ns = rospy.get_namespace().replace('acvs/', '')
        for vk, vector in vectors.items():
            vector_endpoints[vk] = {'static': {}, 'dynamic': None}
            if vector.has_dynamic:
                vector_endpoints[vk]['dynamic'] = (rospy.get_namespace() + vector.namespace_prefix + '/dynamic_input', None)
            if vector.has_static:
                for sk, symbol in symbols.items():  
                    vector_endpoints[vk]['static'][sk] = (base_ns + vector.namespace_prefix + '/' + symbol.name.replace(' ', '_'), symbol)

        # Next, let's build our communication dispatcher
        self.dispatcher = CommunicationDispatcher(vector_endpoints)

        # Now, the Action server.
        self._as = actionlib.SimpleActionServer('communicate_to_diver', CommunicateToDiverAction, self.handle_communication_goal, False)
        self._as.start()

    def handle_communication_goal(self, goal):
        rospy.loginfo(f"Selecting vectors for content: {goal.symbol}")

        selections = []
        if type(self.policy) in [StaticPolicy, RandomPolicy]:
            selections = self.policy.select_vector(goal)
        elif type(self.policy) in [HeuristicPolicy, POMDPPolicy]:
            rel_dc = self.context.select_interactant()
            selections = self.policy.select_vector(goal, rel_dc)
        else:
            raise TypeError(f"What kind of policy is this: {type(self.policy)}")

        
        selected_ids = [vec.id for vec in selections]
        rospy.loginfo(f"Got {len(selected_ids)} selected vectors by {self.policy_type} policy.")
        rospy.loginfo(f"Selected vectors: {selected_ids}")
        # self.dispatcher.dispatch_communication(goal, selections)

        self._result.vectors_used = selected_ids
        self._as.set_succeeded(self._result)

if __name__ == "__main__":
    acvs = ACVSNode()

    r = rospy.Rate(10)
    while not rospy.is_shutdown():
        # Keep our context manager current. Subscriber callbacks add new data, but we need to make sure we know what's going on.
        acvs.context.update_context() 
        r.sleep()