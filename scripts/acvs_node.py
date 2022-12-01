#! /usr/bin/python

import rospy
import actionlib

from proteus_msgs.msg import DiverGroup, Diver
from proteus_msgs.srv import SymbolDirectional, SymbolQuantity, SymbolTrigger


class ACVSNode(object):
    def __init__(self):
        self.dispatcher = CommunicationDispatcher()
        self.policy = ACVS

if __name__ == "__main__":
    