#!/usr/bin/env python
# -*- coding: utf-8 -*-
#!!!!!This program must be executed on windows python3 environment!!!!!

import random

def show_hand(win_msg):
    hand_types = ["rock", "paper", "scissors"]
    hand_type = random.choice(hand_types)

    return hand_type

def judge_game(win_hand, ros_hand):
    print("win_hand:", win_hand, "\tros_hand:", ros_hand)

    result = "[rps]win"
    #result = "[rps]lose"
    #result = "[rps]draw"

    return result

def start_game(topic_name_from_win, ros_bridge_tcp):
    # definition of message types receiving from ros
    hand_types = ["[rps]rock", "[rps]paper", "[rps]scissors"]

    # Send start message and wait hand type selected by ROS
    pub_msg = {
        "op": "publish",
        "topic": topic_name_from_win,
        "msg": {"data": "[rps]OK_start"}
    }
    message_from_ros = ros_bridge_tcp.wait_response(pub_msg, hand_types, timeout=30)
    if message_from_ros:
        print("\nReceive from ROS:", message_from_ros, "\n")

    # Decide your hand
    window_message = "Rock, Paper, Scissors, Go!"
    win_hand = show_hand(window_message)

    # Judge
    result = judge_game(win_hand, message_from_ros)
    pub_msg = {
        "op": "publish",
        "topic": topic_name_from_win,
        "msg": {"data": result}
    }
    # Send game result to ROS.
    print(result)
    # In this program, ros never return to the following wait_response
    ros_bridge_tcp.wait_response(pub_msg, ["[rps]OK_result"], timeout=5)


def demo():
    # Decide ROS hand
    hand_types = ["[rps]rock", "[rps]paper", "[rps]scissors"]
    message_from_ros = random.choice(hand_types)
    print("\nReceive from ROS:", message_from_ros, "\n")

    # Decide your hand
    window_message = "Rock, Paper, Scissors, Go!"
    win_hand = show_hand(window_message)

    # Judge
    result = judge_game(win_hand, message_from_ros)
    print("\nWindows:", result, "\n")


if __name__=="__main__":
    demo()