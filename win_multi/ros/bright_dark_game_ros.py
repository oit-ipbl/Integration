#!/usr/bin/env python
# -*- coding: utf-8 -*-
import math
import os
import random
import rospy
from std_msgs.msg import String
from utils import RosWinMessenger

def play_bright_dark_game():
    rospy.sleep(3)
    node_name = rospy.get_name()
    # Prepare to play Windows game A
    # Specify topic names to commnicate with play_with_ros_test_a.py
    to_win_pub = rospy.Publisher("/from_ros", String, queue_size=1)
    messenger = RosWinMessenger(to_win_pub, "/from_windows")
    # Start game sequence A
    rospy.loginfo("%s:Try to start bright dark game", node_name)
 
    # Send game start signal to Windows, and wait Windows side response.
    message_from_win = messenger.wait_response(
        "[bdg]start", ["[bdg]start"], timeout=30)
    if message_from_win:
        rospy.loginfo("%s:Receive from win:%s",
                      node_name, message_from_win)
    else:
        rospy.logerr("%s:Timeout. can't start Windows bright dark game", node_name)
        return "[bdg]timeout"
 
    # Select bright or dark
    bd_types = ["[bdg]bright", "[bdg]dark"]
    bd_type = random.choice(bd_types)
    rospy.loginfo("%s:Robot selects '%s'", node_name, bd_type)
 
    # Send robot's choice to windows brignt dark game
    message_from_win = messenger.wait_response(
        bd_type, ["[bdg]correct", "[bdg]wrong"], timeout=30)
    if message_from_win:
        rospy.loginfo("%s:Receive from win:%s",
                      node_name, message_from_win)
    else:
        rospy.logerr(
            "%s:Timeout. can't get Windows bright dark game result", node_name)
        return "[bdg]timeout"
    rospy.sleep(3)
    return message_from_win                          


def demo():
    script_name = os.path.basename(__file__)
    rospy.init_node(os.path.splitext(script_name)[0])
    rospy.sleep(0.5)  # rospy.Time.now() returns 0, without this sleep.

    rospy.loginfo("%s:Started", rospy.get_name())
    rospy.sleep(10)
    node_name = rospy.get_name()
    print("---bdg---")
    result = play_bright_dark_game()
    rospy.sleep(10)

    #node.process()
    rospy.loginfo("%s:Exiting", rospy.get_name())

# If you want to test this program only, please execute "rosrun oit_pbl_ros_samples bright_dark_game_ros.py" 
if __name__ == '__main__':
    try:
        demo()
    except Exception as e:
        rospy.logerr("%s:%s", rospy.get_name(), str(e))
        exit(1)
