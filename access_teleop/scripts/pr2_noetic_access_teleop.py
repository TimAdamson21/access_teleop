#!/usr/bin/env python2

import rospy  # If you are doing ROS in python, then you will always need this import

# Message imports go here
import moveit_msgs.msg
import geometry_msgs.msg
from std_msgs.msg import String


# Service imports go here

# All other imports go here

# Python 2/3 compatibility imports
from __future__ import print_function
from six.moves import input

import sys
import copy
import moveit_commander

try:
    from math import pi, tau, dist, fabs, cos
except:  # For Python 2 compatibility
    from math import pi, fabs, cos, sqrt

    tau = 2.0 * pi

    def dist(p, q):
        return sqrt(sum((p_i - q_i) ** 2.0 for p_i, q_i in zip(p, q)))


from moveit_commander.conversions import pose_to_list


# Hyper-parameters go here
SLEEP_RATE = 10


class PR2Controller(object):
    """
    Note: In the init function, I will first give a description of each part, and then I will give an example
    """

    def __init__(self):
        # Everything besides pubs, subs, services, and service proxies go here
        print("Initializing node")
        robot = moveit_commander.RobotCommander()

        # Publishers go here

        # Service Proxies go here

        # Subscribers go here

        # Services go here



def main():
    rospy.init_node("pr2_noetic_access_teleop")

    pr2_controller = PR2Controller()

    rate = rospy.Rate(SLEEP_RATE)

    # This while loop is what keeps the node from dying
    while not rospy.is_shutdown():
        # If I wanted my node to constantly publish something, I would put the publishing here
        rate.sleep()


if __name__ == '__main__':
    main()