#!/usr/bin/env python
# -*- coding: utf-8 -*-
#!!!!!This program must be executed on windows python3 environment!!!!!
from rosbridge_tcp import RosBridgeTCP
import show_hand_game_win as shgw #Image Processing Program that communicate with ROS

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
            # If message['msg']['data'] is "[shg]start show hand game", then shg.star_game is invoked
            if message['msg']['data'] == "[shg]start":
                shgw.start_game(topic_name_from_win, ros_bridge_tcp)
            if message['msg']['data'] == "[all]end":
                pub_msg = {
                    "op": "publish",
                    "topic": topic_name_from_win,
                    "msg": {"data": "[all]finished"}
                }
                print("Every game has finished!")
                # Send message "Every game has finished!" to ros.
                ros_bridge_tcp.wait_response(pub_msg, ["[all]finished"], timeout=5)
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
