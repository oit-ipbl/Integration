#!/usr/bin/env python
# -*- coding: utf-8 -*-
import math
import os
import random
import rospy
from std_msgs.msg import String
from utils import RosWinMessenger

import actionlib
from move_base_msgs.msg import MoveBaseAction
from utils import navigation


class Navigation(object):
    def __init__(self):
        self.move_base_name = "move_base"

    def process(self, x=3.0, y=3.6, angle=90):
        # connect to move_base action server
        ac = actionlib.SimpleActionClient(self.move_base_name, MoveBaseAction)
        # wait until connnect to move_base action server
        while not ac.wait_for_server(rospy.Duration(5)):
            rospy.loginfo("Waiting for the move_base action server to come up")
        rospy.loginfo("The server comes up")
        # send navigation goal x, y, theta
        navigation(ac, x, y, math.radians(angle)) # convert degrees to radians


class gameWithWinNode(object):
    def __init__(self):
        pass

    def play_show_hand_game(self):
        rospy.sleep(3)
        node_name = rospy.get_name()
        # Prepare to play Windows game A
        # Specify topic names to commnicate with play_with_ros_test_a.py
        to_win_pub = rospy.Publisher("/from_ros", String, queue_size=1)
        messenger = RosWinMessenger(to_win_pub, "/from_windows")
        # Start game sequence A
        rospy.loginfo("%s:Try to start show hand game", node_name)

        # Send game start signal to Windows, and wait Windows side response.
        message_from_win = messenger.wait_response(
            "start show hand game", None, timeout=30)
        if message_from_win:
            rospy.loginfo("%s:Receive from win:%s",
                          node_name, message_from_win)
        else:
            rospy.logerr("%s:Timeout. can't start Windows show hand game", node_name)
            return "None"

        # Select robot's hand_type
        hand_types = ["left", "right"]
        hand_type = random.choice(hand_types)
        rospy.loginfo("%s:Robot selects '%s'", node_name, hand_type)

        # Send robot's choice to windows game Rock, Paper, Scissors, and wait game result
        message_from_win = messenger.wait_response(
            hand_type, ["win", "lose", "even"], timeout=30)
        if message_from_win:
            rospy.loginfo("%s:Receive from win:%s",
                          node_name, message_from_win)
        else:
            rospy.logerr(
                "%s:Timeout. can't get Windows show hand game result", node_name)
            return "None"
        rospy.sleep(3)
        return message_from_win                          

    def play_bright_dark_game(self):
        rospy.sleep(3)
        node_name = rospy.get_name()
        # Prepare to play Windows game A
        # Specify topic names to commnicate with play_with_ros_test_a.py
        to_win_pub = rospy.Publisher("/from_ros", String, queue_size=1)
        messenger = RosWinMessenger(to_win_pub, "/from_windows")
        # Start game sequence A
        rospy.loginfo("%s:Try to start show hand game", node_name)

        # Send game start signal to Windows, and wait Windows side response.
        message_from_win = messenger.wait_response(
            "start bright dark game", None, timeout=30)
        if message_from_win:
            rospy.loginfo("%s:Receive from win:%s",
                          node_name, message_from_win)
        else:
            rospy.logerr("%s:Timeout. can't start Windows bright dark game", node_name)
            return "None"

        # Select robot's hand_type
        hand_types = ["bright", "dark"]
        hand_type = random.choice(hand_types)
        rospy.loginfo("%s:Robot selects '%s'", node_name, hand_type)

        # Send robot's choice to windows game Rock, Paper, Scissors, and wait game result
        message_from_win = messenger.wait_response(
            hand_type, ["win", "lose", "even"], timeout=30)
        if message_from_win:
            rospy.loginfo("%s:Receive from win:%s",
                          node_name, message_from_win)
        else:
            rospy.logerr(
                "%s:Timeout. can't get Windows bright dark game result", node_name)
            return "None"
        rospy.sleep(3)
        return message_from_win                          

    def play_rps_game(self):
        rospy.sleep(3)
        node_name = rospy.get_name()
        # Prepare to play Windows game A
        # Specify topic names to commnicate with play_with_ros_test_a.py
        to_win_pub = rospy.Publisher("/from_ros", String, queue_size=1)
        messenger = RosWinMessenger(to_win_pub, "/from_windows")
        # Start game sequence A
        rospy.loginfo("%s:Try to start rps game", node_name)

        # Send game start signal to Windows, and wait Windows side response.
        message_from_win = messenger.wait_response(
            "start rps game", None, timeout=30)
        if message_from_win:
            rospy.loginfo("%s:Receive from win:%s",
                          node_name, message_from_win)
        else:
            rospy.logerr("%s:Timeout. can't start Windows rps game", node_name)
            return "None"

        # Select robot's hand_type
        hand_types = ["rock", "paper", "scissors"]
        hand_type = random.choice(hand_types)
        rospy.loginfo("%s:Robot selects '%s'", node_name, hand_type)

        # Send robot's choice to windows game Rock, Paper, Scissors, and wait game result
        message_from_win = messenger.wait_response(
            hand_type, ["win", "lose", "even"], timeout=30)
        if message_from_win:
            rospy.loginfo("%s:Receive from win:%s",
                          node_name, message_from_win)
        else:
            rospy.logerr(
                "%s:Timeout. can't get Windows rps game result", node_name)
            return "None"
        rospy.sleep(3)
        return message_from_win


    def end_game(self):
        rospy.sleep(3)
        node_name = rospy.get_name()
        # Prepare to play Windows game A
        # Specify topic names to commnicate with play_with_ros_test_a.py
        to_win_pub = rospy.Publisher("/from_ros", String, queue_size=1)
        messenger = RosWinMessenger(to_win_pub, "/from_windows")
        # Start game sequence A
        rospy.loginfo("%s:Try to end game", node_name)

        # Send game start signal to Windows, and wait Windows side response.
        message_from_win = messenger.wait_response(
            "end game", None, timeout=30)
        if message_from_win:
            rospy.loginfo("%s:Receive from win:%s",
                          node_name, message_from_win)
        else:
            rospy.logerr("%s:Timeout. can't end Windows game process.", node_name)


    def process(self):
        # for navigation
        script_name = os.path.basename(__file__)
        rospy.init_node(os.path.splitext(script_name)[0])
        rospy.sleep(0.5)  # rospy.Time.now() returns 0, without this sleep.
        node = Navigation()

        # process
        rospy.sleep(10)
        node_name = rospy.get_name()

        print("---shg---")
        result_shg = self.play_show_hand_game()
        rospy.sleep(10)

        print("---nav---")
        rospy.loginfo("%s:Started", rospy.get_name())
        node.process(x=2.0, y=2.5, angle=270)
        rospy.loginfo("%s:Exiting", rospy.get_name())        
        rospy.sleep(10)

        print("---bdg---")
        result_bdg = self.play_bright_dark_game()
        rospy.sleep(10)


        print("---nav---")
        rospy.loginfo("%s:Started", rospy.get_name())
        node.process(x=3.0, y=3.6, angle=90)
        rospy.loginfo("%s:Exiting", rospy.get_name())        
        rospy.sleep(10)

        print("---rps---")
        result_rps = self.play_rps_game()  # Play game A
        rospy.sleep(10)

        print("---nav---")
        rospy.loginfo("%s:Started", rospy.get_name())
        node.process(x=10.0, y=10.6, angle=90)
        rospy.loginfo("%s:Exiting", rospy.get_name())        
        rospy.sleep(10)

        print("---end---")
        self.end_game()

        # Show game results
        rospy.loginfo("/* GAME RESULTS */")
        rospy.loginfo("/* GAME (shg):%s */", result_shg)
        rospy.loginfo("/* GAME (bdg):%s */", result_bdg)
        rospy.loginfo("/* GAME (rps):%s */", result_rps)
        rospy.loginfo("/* ------------ */")


def main():
    script_name = os.path.basename(__file__)
    rospy.init_node(os.path.splitext(script_name)[0])
    rospy.sleep(0.5)  # rospy.Time.now() returns 0, without this sleep.

    node = gameWithWinNode()
    rospy.loginfo("%s:Started", rospy.get_name())

    node.process()
    rospy.loginfo("%s:Exiting", rospy.get_name())


if __name__ == '__main__':
    try:
        main()
    except Exception as e:
        rospy.logerr("%s:%s", rospy.get_name(), str(e))
        exit(1)
