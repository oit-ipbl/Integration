#!/usr/bin/env python
# -*- coding: utf-8 -*-
import math
import os
import random
import rospy
from std_msgs.msg import String
from utils import RosWinMessenger

def process():
    rospy.sleep(10)
    node_name = rospy.get_name()

    rate = rospy.Rate(2)  # Keep loop with 2hz
    # for send message to windows
    # Create a publisher which can publish String type topics.
    to_win_pub = rospy.Publisher("/from_ros", String, queue_size=1)

    # for recieve message from windows
    # Wrapper class to receive messages from Windows
    messenger = RosWinMessenger(to_win_pub, "/from_windows")

    for i in range(0, 10):   
        # Publish messages to Windows
        message = "Hello! this is ROS " + str(i)
        rospy.loginfo("%s:Sending message to win(%d):%s", node_name, i, message)
        # See https://github.com/oit-ipbl/robots/blob/main/basics/basics_01.md#summary-of-talkerpy
        to_win_pub.publish(message) 

        rate.sleep()

    # Receive messages from Windows
    for i in range(0, 10):
        message = messenger.wait_response(timeout=5)
        if message is not None:
            rospy.loginfo("%s:Receive from win(%d):%s", node_name, i, message)

def main():
    script_name = os.path.basename(__file__)
    rospy.init_node(os.path.splitext(script_name)[0])
    rospy.sleep(0.5)  # rospy.Time.now() returns 0, without this sleep.

    rospy.loginfo("%s:Started", rospy.get_name())

    process()
    rospy.loginfo("%s:Exiting", rospy.get_name())


if __name__ == '__main__':
    try:
        main()
    except Exception as e:
        rospy.logerr("%s:%s", rospy.get_name(), str(e))
        exit(1)
