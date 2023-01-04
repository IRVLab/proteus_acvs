import rospy, actionlib

from threading import Thread
from rospy import ServiceProxy

from std_msgs.msg import String
from proteus_msgs.srv import SymbolDirectional, SymbolQuantity, SymbolTrigger
from proteus.utils import assign_service_type

class CommunicationDispatcher(object):
    def __init__(self, vector_endpoints):
        self.vector_endpoints = vector_endpoints

    def dispatch_communication(self, comm, vectors):
        proxies = []
        for vec in vectors:
            rospy.loginfo(f"Using {vec}")
            if comm.dynamic:
                grn, _ = self.vector_endpoints[vec.id]['dynamic']
                pub = rospy.Publisher(grn, String, queue_size=5)
                msg = String()
                msg.data = comm.content
                pub.publish(msg)

            else:
                grn, symbol = self.vector_endpoints[vec.id]['static'][comm.symbol]
                service_type = assign_service_type(symbol.input_required)
                proxy = ServiceProxy(grn, service_type)

                if service_type == SymbolTrigger:
                    t = Thread(target=proxy)
                    proxies.append(t)
                    t.start()
                elif service_type == SymbolQuantity:
                    t = Thread(target=proxy, args=[float(comm.content)])
                    proxies.append
                    t.start()
                else:
                    raise NotImplementedError(f"Service calls of type {service_type} have not yet been implemented in the Communication Dispatcher class")

        for t in proxies:
            t.join()



        
        