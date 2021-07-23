#!/usr/bin/env python
# -*- coding: utf-8 -*-
import os
import math
import rospy
import actionlib
from move_base_msgs.msg import MoveBaseAction
from utils import navigation

def process(x=3.0, y=3.6, angle=90):
    move_base_name = "move_base"
    # connect to move_base action server
    ac = actionlib.SimpleActionClient(move_base_name, MoveBaseAction)
    # wait until connnect to move_base action server
    while not ac.wait_for_server(rospy.Duration(5)):
        rospy.loginfo("Waiting for the move_base action server to come up")
    rospy.loginfo("The server comes up")
    # send navigation goal x, y, theta
    navigation(ac, x, y, math.radians(angle)) # convert degrees to radians

def demo():
    script_name = os.path.basename(__file__)
    rospy.init_node(os.path.splitext(script_name)[0])
    rospy.sleep(0.5)  # rospy.Time.now() returns 0, without this sleep.

    rospy.loginfo("%s:Started", rospy.get_name())
    process(1.0, 2.5, -90)
    rospy.loginfo("%s:Exiting", rospy.get_name())

if __name__ == '__main__':
    try:
        demo()
    except Exception as e:
        rospy.logerr("%s:%s", rospy.get_name(), str(e))
        exit(1)