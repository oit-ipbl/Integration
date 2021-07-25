#!/usr/bin/env python
# -*- coding: utf-8 -*-
import math
import os
import random
import rospy
from std_msgs.msg import String
from utils import RosWinMessenger
import show_hand_game_ros as shgr
import bright_dark_game_ros as bdg

def end_game():
    rospy.sleep(3)
    node_name = rospy.get_name()
    # Specify topic names to commnicate with windows side
    to_win_pub = rospy.Publisher("/from_ros", String, queue_size=1)
    messenger = RosWinMessenger(to_win_pub, "/from_windows")
    rospy.loginfo("%s:Try to end game", node_name)

    # Send The end signal to Windows, and wait Windows side response.
    message_from_win = messenger.wait_response(
        "The end", None, timeout=30)
    if message_from_win:
        rospy.loginfo("%s:Receive from win:%s",
                      node_name, message_from_win)
    else:
        rospy.logerr("%s:Timeout. can't end Windows game process.", node_name)

def process():
    rospy.sleep(10)
    node_name = rospy.get_name()

    print("---shg---")
    result = shgr.play_show_hand_game()
    rospy.sleep(5)
    print("---bdg---")
    result_bdg = bdg.play_bright_dark_game()
    rospy.sleep(5)
    end_game()
    rospy.loginfo("/* GAME:%s */", result)

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
