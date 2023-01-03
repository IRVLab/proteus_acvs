import rospy, actionlib

from threading import Thread
from rospy import ServiceProxy

from std_msgs.msg import String
from proteus_msgs.srv import SymbolDirectional, SymbolQuantity, SymbolTrigger
from proteus_msgs.msg import CommunicateToDiverAction, CommunicateToDiverFeedback, CommunicateToDiverResult

from proteus.utils import assign_service_type

class CommunicationDispatcher(object):
    _feedback = CommunicateToDiverFeedback()
    _result = CommunicateToDiverResult()

    def __init__(self, vector_endpoints, select_vector_fp):
        self._as = actionlib.SimpleActionServer('communicate_to_diver', CommunicateToDiverAction, self.execute, False)
        self.vector_endpoints = vector_endpoints
        self.select_vector = select_vector_fp
        self._as.start()

    def execute(self, goal):
        rospy.loginfo(f"Selecting vectors for content: \"{goal.symbol}\"")
        selections = self.select_vector(goal)
        
        rospy.loginfo(f"Got {len(selections)} selected vectors by policy.")
        proxies = []
        for vec in selections:
            rospy.loginfo(f"Using {vec}")
            if goal.dynamic:
                grn, _ = self.vector_endpoints[vec.id]['dynamic']
                pub = rospy.Publisher(grn, String, queue_size=5)
                msg = String()
                msg.data = goal.content
                pub.publish(msg)

            else:
                grn, symbol = self.vector_endpoints[vec.id]['static'][goal.symbol]

                service_type = assign_service_type(symbol.input_required)
                proxy = ServiceProxy(grn, service_type)

                if service_type == SymbolTrigger:
                    t = Thread(target=proxy)
                    proxies.append(t)
                    t.start()
                elif service_type == SymbolQuantity:
                    t = Thread(target=proxy, args=[float(goal.content)])
                    proxies.append
                    t.start()
                else:
                    raise NotImplementedError(f"Service calls of type {service_type} have not yet been implemented in the Communication Dispatcher class")

        for t in proxies:
            t.join()

        self._result.vectors_used = [vec.id for vec in selections]
        self._as.set_succeeded(self._result)



        
        