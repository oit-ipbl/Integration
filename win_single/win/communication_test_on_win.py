#!/usr/bin/env python
# -*- coding: utf-8 -*-
import random
import time
from rosbridge_tcp import RosBridgeTCP
from ros_utils import build_ros_array_msg


def main():
    # Wrapper class for TCP/IP communication
    ros_bridge_tcp = RosBridgeTCP()
    # Prepare to publish ROS topic from Windows
    topic_name_from_win = "/from_windows"
    advertise_msg = {
        "op": "advertise", # "op" should be "advertise"
        "topic": topic_name_from_win, # Topic name
        "type": "std_msgs/String"     # Topic type
    }
    ros_bridge_tcp.send_message(advertise_msg) # Send advertise message to ROS
    # Prepare to subscribe to ROS topic
    topic_name_from_ros = "/from_ros"
    subscribe_msg = {
        "op": "subscribe", # "op" should be "subscribe"
        "topic": topic_name_from_ros, # Topic name
        "type": "std_msgs/String"     # Topic type
    }
    ros_bridge_tcp.send_message(subscribe_msg) # Send subscribe message to ROS
    tm = time.time()
    received = 0
    while time.time() - tm < 20 and received < 10:
        messages = ros_bridge_tcp.wait() # Waiting messages from ROS. wait() returns an array. 
        received += len(messages)
        for m in messages:
            print(str(m)) # Output each element of the messages.
    for i in range(0, 10):
        # Make a message to ROS
        message = "Hello this is Windows " + str(i)
        pub_msg = {
            "op": "publish", # "op" should be "publish"
            "topic": topic_name_from_win,  # Topic name. The topic type is String, that is already determined.
            "msg": {"data": message} # Set the string you want to send to ROS, as the value of the "data" key .
        }
        ros_bridge_tcp.send_message(pub_msg) # Send the messsage to ROS.
        print("Sending ros message: " + str(pub_msg))
        time.sleep(1)

    try:
        ros_bridge_tcp.terminate() # Close TCP/IP connection.
        ros_bridge_tcp = None
    except Exception as e:
        print(str(e))


if __name__ == '__main__':
    main()
