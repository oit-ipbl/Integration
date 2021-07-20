#!/usr/bin/env python
# -*- coding: utf-8 -*-
import random
# from rosbridge_tcp import RosBridgeTCP
# from ros_utils import build_ros_array_msg


def decide_win_hand(hand_types):
    ## hand gesticulation with image processing ###
    hand_type = random.choice(hand_types)
    ###############################################
    print("Windows side selected hand type '" + hand_type + "'")
    return hand_type

def judge_game(win_hand, ros_hand):
    print("Win:", win_hand, "\tROS:", ros_hand)
    ## judge rps game by using 2 hand_type ########
    return random.choice(["win", "lose", "even"])
    ###############################################

def start_game(topic_name_from_win, ros_bridge_tcp):
    message_from_ros = ros_bridge_tcp.wait_response(timeout=30)
    if message_from_ros:
        print("Receive from ROS:" + message_from_ros)

    # Decide your hand
    hand_types = ["rock","paper","scissors"]
    hand_type = decide_win_hand(hand_types)

    # Send your hand type to ROS, and wait ROS robot's hand type.
    pub_msg = {
        "op": "publish",
        "topic": topic_name_from_win,
        "msg": {"data": hand_type}
    }
    message_from_ros = ros_bridge_tcp.wait_response(
        pub_msg, hand_types, timeout=30)
    if message_from_ros:
        print("Receive from ROS:" + message_from_ros)

    # Judge
    result = judge_game(hand_type, message_from_ros)
    pub_msg = {
        "op": "publish",
        "topic": topic_name_from_win,
        "msg": {"data": result}
    }
    # Send game result to ROS.
    print(result)
    ros_bridge_tcp.wait_response(pub_msg, ["OK"], timeout=5)

def demo_DIP_game():
    # Decide your hand
    hand_types = ["rock","paper","scissors"]
    hand_type = decide_win_hand(hand_types)

    message_from_ros = random.choice(hand_types)
    print("Receive from ROS:" + message_from_ros)

    # Judge
    result = judge_game(hand_type, message_from_ros)
    print("Windows:", result)

if __name__=="__main__":
    demo_DIP_game()
