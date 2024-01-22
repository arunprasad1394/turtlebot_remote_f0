#!/usr/bin/env python3
import rospy
from nav_msgs.msgs import Odemetry

def odometryCb(msg):
    print msg.pose.pose

if __name__ == "__main__":
    rospy.init_node('oodometry', anonymous=True)
    rospy.Subscriber('odom' Odemetry, odometryCb)
    rospy.spin()