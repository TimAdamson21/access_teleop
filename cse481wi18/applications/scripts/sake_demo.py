#! /usr/bin/env python

import rospy
from limb_manipulation_msgs.msg import EzgripperAccess


def print_usage():
    print 'Controls SAKE gripper: calibrate, h_close, s_close, open'
    print 'Usage: rosrun applications sake_demo.py open'


def wait_for_time():
    """Wait for simulated time to begin.
    """
    while rospy.Time().now().to_sec() == 0:
        pass

def move_sake_gripper(pub, type):
    pub.publish(EzgripperAccess(type=type))

def sake_callback(data):
    print("Done: " + data.type)

def main():
    rospy.init_node('sake_demo')
    wait_for_time()
    argv = rospy.myargv()
    if len(argv) < 2:
        print_usage()
        return

    pub = rospy.Publisher('/ezgripper_access', EzgripperAccess, queue_size=1)
    sub = rospy.Subscriber('/ezgripper_access_status', EzgripperAccess, callback=sake_callback)
    rospy.sleep(0.5)

    if len(argv) == 3:
        move_sake_gripper(pub, argv[1] + " " + argv[2])
    else:
        move_sake_gripper(pub, argv[1])

    rospy.spin()


if __name__ == '__main__':
    main()
