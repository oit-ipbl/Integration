#!/usr/bin/env python
# -*- coding: utf-8 -*-
import math
import os
import random
import rospy
from std_msgs.msg import String
from utils import RosWinMessenger

def play_show_hand_game():
    rospy.sleep(3) 
    node_name = rospy.get_name()
    # Prepare to play show hand game
    # Specify topic names to commnicate with show hand game
    to_win_pub = rospy.Publisher("/from_ros", String, queue_size=1)
    messenger = RosWinMessenger(to_win_pub, "/from_windows")
    # Start game sequence
    rospy.loginfo("%s:Try to start show hand game", node_name)

    # Send game start signal to Windows, and wait Windows side response.
    message_from_win = messenger.wait_response(
        "[rpsw]start", ["[rpsw]start"], timeout=30)
    if message_from_win:
        rospy.loginfo("%s:Receive from win:%s",
                          node_name, message_from_win)
    else:
        rospy.logerr("%s:Timeout. can't start show hand game on windows", node_name)
        return "[rpsr]timeout"

    # Select robot's hand_type
    hand_types = ["[rpsr]rock", "[rpsr]paper", "[rpsr]scissors"]
    hand_type = random.choice(hand_types)
    rospy.loginfo("%s:Robot selects '%s'", node_name, hand_type)

    # Send hand_type chosen by robot to windows show hand game, and wait game result
    message_from_win = messenger.wait_response(
        hand_type, ["[rpsw]win", "[rpsw]draw", "[rpsw]lose"], timeout=30)
    if message_from_win:
        rospy.loginfo("%s:Receive from win:%s",
                    node_name, message_from_win)
    else:
        rospy.logerr(
            "%s:Timeout. can't get show hand game result on windows", node_name)
        return "[rpsr]timeout"
    rospy.sleep(3)
    return message_from_win                          

def demo():
    script_name = os.path.basename(__file__)
    rospy.init_node(os.path.splitext(script_name)[0])
    rospy.sleep(0.5)  # rospy.Time.now() returns 0, without this sleep.

    rospy.loginfo("%s:Started", rospy.get_name())
    rospy.sleep(10)
    node_name = rospy.get_name()
    print("---rpsr---")
    result = play_show_hand_game()
    rospy.sleep(10)

    #node.process()
    rospy.loginfo("%s:Exiting", rospy.get_name())

# If you want to test this program only, please execute "rosrun oit_pbl_ros_samples show_hand_game_ros.py" 
if __name__ == '__main__':
    try:
        demo()
    except Exception as e:
        rospy.logerr("%s:%s", rospy.get_name(), str(e))
        exit(1)