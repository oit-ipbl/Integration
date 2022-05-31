#!/usr/bin/env python
# -*- coding: utf-8 -*-
import math
import os
import random
import rospy
from std_msgs.msg import String
from utils import RosWinMessenger

def play_rps_game():
    rospy.sleep(3) 
    node_name = rospy.get_name()
    # Prepare to play rps game
    # Specify topic names to commnicate with rps game
    to_win_pub = rospy.Publisher("/from_ros", String, queue_size=1)
    messenger = RosWinMessenger(to_win_pub, "/from_windows")
    # Start game sequence
    rospy.loginfo("%s:Try to start rps game", node_name)

    # Send game start signal to Windows, and wait Windows side response.
    message_from_win = messenger.wait_response(
        "[rps]start", ["[rps]start"], timeout=30)
    if message_from_win:
        rospy.loginfo("%s:Receive from win:%s",
                          node_name, message_from_win)
    else:
        rospy.logerr("%s:Timeout. can't start rps game on windows", node_name)
        return "[rps]timeout"

    # Select robot's hand_type
    hand_types = ["[rps]rock", "[rps]paper", "[rps]scissors"]
    hand_type = random.choice(hand_types)
    rospy.loginfo("%s:Robot selects '%s'", node_name, hand_type)

    # Send robot's choice to windows game "[rps]rock", "[rps]paper", "[rps]scissors", and wait game result
    # 1st argument: Message for publishing.
    # 2nd argument: Target keywords to receive. Your program will wait until receiving one of the word in the list, ["[rps]win", "[rps]draw", "[rps]lose"].
    message_from_win = messenger.wait_response(
        hand_type, ["[rps]win", "[rps]draw", "[rps]lose"], timeout=30)
    if message_from_win:
        rospy.loginfo("%s:Receive from win:%s",
                    node_name, message_from_win)
    else:
        rospy.logerr(
            "%s:Timeout. can't get rps game result on windows", node_name)
        return "[rps]timeout"
    rospy.sleep(3)
    return message_from_win                          

def demo():
    script_name = os.path.basename(__file__)
    rospy.init_node(os.path.splitext(script_name)[0])
    rospy.sleep(0.5)  # rospy.Time.now() returns 0, without this sleep.

    rospy.loginfo("%s:Started", rospy.get_name())
    rospy.sleep(10)
    node_name = rospy.get_name()
    print("---rps---")
    result = play_rps_game()
    rospy.sleep(10)

    rospy.loginfo("%s:Exiting", rospy.get_name())

# If you want to test this program, please execute "rosrun oit_pbl_ros_samples rps_game_on_ros.py" 
if __name__ == '__main__':
    try:
        demo()
    except Exception as e:
        rospy.logerr("%s:%s", rospy.get_name(), str(e))
        exit(1)