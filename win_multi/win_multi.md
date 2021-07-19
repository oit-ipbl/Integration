# Message exchange between a ROS node and multi Windows programs

[README](../README.md)

---

## Objectives

Make Windonws programs and a ROS node which can communicate with each other via TCP/IP.

ROS node controls multi Windows programs, therefore the node should define multi topic names.  
Each topic name corresponds to each Windows program, and the ROS node can select Windows program to exchange messages by specifing a topic name.

## Prerequisite

You have to finish all of [robots](https://github.com/oit-ipbl/robots) and [image processing](https://github.com/oit-ipbl/image_processing).

## Practice

### Make a Windows side python program Main
```python
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
```

### Make a Windows side python program GCP

- Make a python file named `play_with_ros_test_a.py` in Windows directory `C:\oit\py21\code` and edit it with VSCode.
  - See [image processing development](https://github.com/oit-ipbl/portal/blob/main/setup/python%2Bvscode.md).

Type the following template. It's OK copy and paste.

```python
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

def judge_gcp_game(win_hand, ros_hand):
    print("Win:", win_hand, "\tROS:", ros_hand)
    ## judge rps game by using 2 hand_type ########
    return random.choice(["win", "lose", "even"])
    ###############################################

def start_game(topic_name_from_win, ros_bridge_tcp):
    message_from_ros = ros_bridge_tcp.wait_response()
    if message_from_ros:
        print("Receive from ROS:" + message_from_ros)

    # Decide your hand
    hand_types = ["gu", "pa", "choki"]
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
    result = judge_gcp_game(hand_type, message_from_ros)
    pub_msg = {
        "op": "publish",
        "topic": topic_name_from_win,
        "msg": {"data": result}
    }
    # Send game result to ROS.
    print(result)
    ros_bridge_tcp.wait_response(pub_msg, ["OK"], timeout=10)
```

### Make a Windows side python program RSP

- Make a python file named `play_with_ros_test_b.py` in Windows directory `C:\oit\py21\code` and edit it with VSCode.
  - See [image processing development](https://github.com/oit-ipbl/portal/blob/main/setup/python%2Bvscode.md).

Type the following template. It's OK copy and paste.

```python
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

def judge_rps_game(win_hand, ros_hand):
    print("Win:", win_hand, "\tROS:", ros_hand)
    ## judge rps game by using 2 hand_type ########
    return random.choice(["win", "lose", "even"])
    ###############################################

def start_game(topic_name_from_win, ros_bridge_tcp):
    message_from_ros = ros_bridge_tcp.wait_response()
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
    result = judge_rps_game(hand_type, message_from_ros)
    pub_msg = {
        "op": "publish",
        "topic": topic_name_from_win,
        "msg": {"data": result}
    }
    # Send game result to ROS.
    print(result)
    ros_bridge_tcp.wait_response(pub_msg, ["OK"], timeout=10)
```

### Make a ROS node

Open `~/catkin_ws/src/oit_pbl_ros_samples/` by Visual Studio Code editor, and edit `communication_multi_test.py`. See [Developing inside the ROS container with VSCode](https://github.com/oit-ipbl/portal/blob/main/setup/remote_with_vscode.md).

Type the following template. It's OK copy and paste.

```python
#!/usr/bin/env python
# -*- coding: utf-8 -*-
import math
import os
import random
import rospy
from std_msgs.msg import String
from utils import RosWinMessenger


class gameWithWinNode(object):
    def __init__(self):
        pass

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
            "rps game start", None, timeout=30)
        if message_from_win:
            rospy.loginfo("%s:Receive from win:%s",
                          node_name, message_from_win)
        else:
            rospy.logerr("%s:Timeout. can't start Windows rps game", node_name)
            return "even"

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
            return "even"
        rospy.sleep(3)
        return message_from_win                          

    def play_gcp_game(self):
        rospy.sleep(3)
        node_name = rospy.get_name()
        # Prepare to play Windows game A
        # Specify topic names to commnicate with play_with_ros_test_a.py
        to_win_pub = rospy.Publisher("/from_ros", String, queue_size=1)
        messenger = RosWinMessenger(to_win_pub, "/from_windows")
        # Start game sequence A
        rospy.loginfo("%s:Try to start gcp game", node_name)

        # Send game start signal to Windows, and wait Windows side response.
        message_from_win = messenger.wait_response(
            "gcp game start", None, timeout=30)
        if message_from_win:
            rospy.loginfo("%s:Receive from win:%s",
                          node_name, message_from_win)
        else:
            rospy.logerr("%s:Timeout. can't start Windows gcp game", node_name)
            return "even"

        # Select robot's hand_type
        hand_types = ["gu", "pa", "choki"]
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
                "%s:Timeout. can't get Windows gcp game result", node_name)
            return "even"
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
        rospy.sleep(10)
        node_name = rospy.get_name()
        print("---rps---")
        result_A = self.play_rps_game()  # Play game A
        rospy.sleep(10)
        print("---gcp---")
        result_B = self.play_gcp_game()  # Play game B
        rospy.sleep(10)
        print("---end---")
        self.end_game()
        # Show game results
        rospy.loginfo("/* GAME RESULTS */")
        rospy.loginfo("/* GAME (A):%s */", result_A)
        rospy.loginfo("/* GAME (B):%s */", result_B)
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
```

### Run

#### ROS side

At first, launch the simulator.

```shell
$ roslaunch oit_stage_ros navigation.launch
```

After a while run a sample program, `communication_multi_test.py`.

```shell
$ rosrun oit_pbl_ros_samples communication_multi_test.py
```

#### Run Windows side programs within about 10 seconds

- Command prompt 1

```cmd
C:\oit\py21\code>python ./play_with_ros_test_a.py
Waiting ROS message... 
Receive from ROS:Start your game! # The outputs will appear after a few seconds
Receive from ROS:scissors
```

- Command prompt 2

```cmd
C:\oit\py21\code>python ./play_with_ros_test_b.py
Waiting ROS message... 
Receive from ROS:Start your game! # The outputs will appear after a few seconds
Receive from ROS:3
```

- ROS side terminal output is here.

```shell
$ rosrun oit_pbl_ros_samples communication_multi_test.py
[INFO] [1626227423.808533, 2825.500000]: /communication_multi_test:Started
[INFO] [1626227436.816055, 2838.500000]: /communication_multi_test:Try to start game A
[INFO] [1626227441.856712, 2843.500000]: /communication_multi_test:Receive from win:scissors
[INFO] [1626227441.859784, 2843.500000]: /communication_multi_test:Robot selects 'scissors'
[INFO] [1626227442.867919, 2844.500000]: /communication_multi_test:Receive from win:lose
[INFO] [1626227448.802668, 2850.500000]: /communication_multi_test:Try to start game B
[INFO] [1626227451.863745, 2853.500000]: /communication_multi_test:Receive from win:3
[INFO] [1626227451.870351, 2853.500000]: /communication_multi_test:Robot selects '3'
[INFO] [1626227453.887412, 2855.600000]: /communication_multi_test:Receive from win:right
[INFO] [1626227456.880912, 2858.600000]: /* GAME RESULTS */
[INFO] [1626227456.884404, 2858.600000]: /* GAME (A):lose */
[INFO] [1626227456.887439, 2858.600000]: /* GAME (B):right */
[INFO] [1626227456.891372, 2858.600000]: /* ------------ */
[INFO] [1626227456.895187, 2858.600000]: /communication_multi_test:Exiting
```

### Sequence of the programs

- ROS node, `communication_multi_test.py`, sends a game start signal to `play_with_ros_test_a.py`.
- Windows side game A program, `play_with_ros_test_a.py` is waiting the signal.
- After that ROS node selects hand type randomly from `["rock", "paper", "scissors"]` and send the hand type to `play_with_ros_test_a.py`.
- `play_with_ros_test_a.py` is waiting the hand type selected by the robot. After recieving the hand type, `play_with_ros_test_a.py` selects hand type randomly from `["rock", "paper", "scissors"]` and judge game result randomly.
- `play_with_ros_test_a.py` return the result to the ROS node.
- Almost same procedure will be done for commnication between ROS node and `play_with_ros_test_b.py`.
- Finally, ROS node shows the twe results.

## Exercise (integration 2)

Add navigation function between game A and game B. See [Robot control 3](https://github.com/oit-ipbl/robots/blob/main/robot_control/robot_control_03.md#robot-control-3).

## Challenge (integration 3)

Windows side programs, `play_with_ros_test_a.py` and `play_with_ros_test_b.py` is dummy programs. They just return randomly selected results.
Modify them to real games using web camera and media-pipe library.

We assume that `play_with_ros_test_a.py` is "Rock, Paper and Scissors" game, and `play_with_ros_test_b.py` is "Finger number count game(指の数を当てるゲームって英語で良いフレーズないですか)".
