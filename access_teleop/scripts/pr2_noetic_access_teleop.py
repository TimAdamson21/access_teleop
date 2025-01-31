#!/usr/bin/env python3
# NOTICE: Noetic uses python3

from __future__ import print_function
import rospy  # If you are doing ROS in python, then you will always need this import

# Message imports go here
import moveit_msgs.msg
import geometry_msgs.msg
from std_msgs.msg import String


# Service imports go here

# All other imports go here

# Python 2/3 compatibility imports
from six.moves import input

import sys
import copy
import moveit_commander

from math import pi, tau, dist, fabs, cos


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
        self.robot = moveit_commander.RobotCommander()
        self.scene = moveit_commander.PlanningSceneInterface()

        self.group_right_arm = moveit_commander.MoveGroupCommander("right_arm")
        self.group_left_arm = moveit_commander.MoveGroupCommander("left_arm")

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