#! /usr/bin/python3

import rospy
import random
import numpy as np

from std_msgs.msg import Header
from geometry_msgs.msg import Point
from proteus_msgs.msg import DiverGroup, Diver, RelativePosition, Pseudodistance


def clamp(val, min, max):
    if val < min:
        return min
    elif val > max:
        return max
    else:
        return val

def random_change(old,  delta_range, min, max):
    sign = random.choice([1,-1])
    
    delta = random.choice(np.linspace(delta_range[0], delta_range[1]))
    new = old + (sign * delta)

    return clamp(new, min, max)


def generate_diver_group_msg(last):
    msg = DiverGroup()
    diver = Diver()
    diver.header = Header()
    diver.diver_id = "Washburn"
    diver.last_seen = rospy.Time.now()
    diver.currently_seen = True
    diver.location = RelativePosition()

    if not last:
        diver.location.center_point_rel = Point()
        diver.location.distance = Pseudodistance()
        diver.location.confidence = 0.75

        diver.location.center_point_rel.x = 0.5
        diver.location.center_point_rel.y = 0.5
        diver.location.distance.distance_ratio = 0.5
        

    else:
        last = last.divers[0]
        diver.location.center_point_rel = Point()
        diver.location.distance = Pseudodistance()
        diver.location.confidence = random_change(last.location.confidence, (0.001, 0.1), 0, 1.0)

        diver.location.center_point_rel.x = random_change(last.location.center_point_rel.x, (0.001, 0.1), 0, 1.0)
        diver.location.center_point_rel.y = random_change(last.location.center_point_rel.y, (0.001, 0.1), 0, 1.0)
        diver.location.distance.distance_ratio = random_change(last.location.distance.distance_ratio, (0.001, 0.1), 0, 1.0)

    diver.estimated_confidence = diver.location.confidence
    diver.location.distance.confidence = diver.location.confidence
    msg.divers.append(diver)
    return msg


if __name__ == "__main__":
    rospy.init_node('fake_diver_pub', anonymous=False)
    rospy.logwarn("IF YOU AREN'T MICHAEL YOU SHOULDN'T USE THIS IT'S FAKE AND BAD")
    publisher = rospy.Publisher('/loco/proteus/divers', DiverGroup, queue_size=5)

    rate = rospy.Rate(1)
    last_msg = None
    while not rospy.is_shutdown():
        msg = generate_diver_group_msg(last_msg)
        publisher.publish(msg)
        last_msg = msg

        rate.sleep()