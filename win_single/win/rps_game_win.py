#!/usr/bin/env python
# -*- coding: utf-8 -*-
#!!!!!This program must be executed on windows python3 environment!!!!!
from rosbridge_tcp import RosBridgeTCP
import random

## Let's modify the code to correctly judge rock-paper-scissors.
def judge_game(win_hand, ros_hand):
    print("win_hand:", win_hand, "\tros_hand:", ros_hand)

    result = "[rps]win"

    return result

def main():
	# Wrapper class for TCP/IP communication
    ros_bridge_tcp = RosBridgeTCP()
    # Prepare to publish ROS topic from Windows
    topic_name_from_win = "/from_windows"
    advertise_msg = {
        "op": "advertise",
        "topic": topic_name_from_win,
        "type": "std_msgs/String"
    }
    ros_bridge_tcp.send_message(advertise_msg) # Send advertise message to ROS
    # Prepare to subscribe to ROS topic
    topic_name_from_ros = "/from_ros"
    subscribe_msg = {
        "op": "subscribe",
        "topic": topic_name_from_ros,
        "type": "std_msgs/String"
    }
    ros_bridge_tcp.send_message(subscribe_msg)

    print("Wait ROS messages...")

    while True:
        messages = ros_bridge_tcp.wait() #Wait for message from ROS and assign the response into 'messages'
        for message in messages:
            # If message['msg']['data'] is "[rps]start", rps_game starts.
            if message['msg']['data'] == "[rps]start":
                # definition of message types receiving from ros
                hand_types = ["[rps]rock", "[rps]paper", "[rps]scissors"]

                # Send start message and wait hand type selected by ROS
                pub_msg = {
                    "op": "publish",
                    "topic": topic_name_from_win,
                    "msg": {"data": "[rps]start"}
                }
                # 1st argument: Message for publishing.
                # 2nd argument: Target keywords to receive. Your program will wait until receiving one of the word in the list, 'hand_types'.
                message_from_ros = ros_bridge_tcp.wait_response(pub_msg, hand_types, timeout=30)
                if message_from_ros:
                    print("\nReceive from ROS:", message_from_ros, "\n")

                # Decide your hand
                win_hand = random.choice(hand_types)
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
                ros_bridge_tcp.wait_response(pub_msg, ["[rps]result"], timeout=5)
                break
        else:
            continue
        break

    try:
        ros_bridge_tcp.terminate()
        ros_bridge_tcp = None
    except Exception as e:
        print(str(e))

if __name__ == '__main__':
    main()
