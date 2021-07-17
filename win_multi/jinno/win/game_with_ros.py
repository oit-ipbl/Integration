#!/usr/bin/env python
# -*- coding: utf-8 -*-
from rosbridge_tcp import RosBridgeTCP
from ros_utils import build_ros_array_msg
import rps_game as rps
import gcp_game as gcp
# import timepytho


def main():
    ros_bridge_tcp = RosBridgeTCP()
    topic_name_from_win = "/from_windows"
    advertise_msg = {
        "op": "advertise",
        "topic": topic_name_from_win,
        "type": "std_msgs/String"
    }
    ros_bridge_tcp.send_message(advertise_msg)
    # subscribe to ros topic
    topic_name_from_ros = "/from_ros"
    subscribe_msg = {
        "op": "subscribe",
        "topic": topic_name_from_ros,
        "type": "std_msgs/String"
    }
    ros_bridge_tcp.send_message(subscribe_msg)

    # tm = time.time()
    while True:
        messages = ros_bridge_tcp.wait()
        for message in messages:
            # print(message)
            # if message['topic'] == "/from_ros_rps":
            if message['msg']['data'] == "rps game start":
                rps.start_game(topic_name_from_win, ros_bridge_tcp)
            if message['msg']['data'] == "gcp game start":
                gcp.start_game(topic_name_from_win, ros_bridge_tcp)
            if message['msg']['data'] == "end game":
                pub_msg = {
                    "op": "publish",
                    "topic": topic_name_from_win,
                    "msg": {"data": "game is finished!"}
                }
                # Send game result to ROS.
                print("Game is finished!")
                ros_bridge_tcp.wait_response(pub_msg, ["OK"], timeout=10)
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
