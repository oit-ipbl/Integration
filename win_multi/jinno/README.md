# 追加したプログラム類
## single
- ROS
  - [win_multi/jinno/ros/game_with_win_single.py]
- Win
  - [win_multi/jinno/win/game_with_ros_single.py]
  - [win_multi/jinno/win/show_hand_game.py]
    - 右手か左手かをROSが指定するので，その手をカメラに写す
## multi
- ROS
  - [win_multi/jinno/ros/game_with_win_multi.py]
- Win
  - [win_multi/jinno/win/game_with_ros_multi.py]
  - [win_multi/jinno/win/show_hand_game.py]
    - 右手か左手かをROSが指定するので，その手をカメラに写す
  - [win_multi/jinno/win/bright_dark_game.py]
    - brightかdarkかをROSが指定するので，カメラ画像をそのようにする（照明に向ける or 手で隠す）
  - [win_multi/jinno/win/rps_game.py]
    - ダミーのじゃんけんゲーム

# ros側
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

# windows側
## ベースのプログラム
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

## 各ゲームプログラム
### ロックペーパーシザーズ
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


# class GameRPS():
#     def __init__(self):
#         self.ros_bridge_tcp = RosBridgeTCP()
#         self.topic_name_from_win = "/from_windows_rps"
#         self.hand_types = ["rock", "paper", "scissors"]
#         advertise_msg = {
#             "op": "advertise",
#             "topic": self.topic_name_from_win,
#             "type": "std_msgs/String"
#         }
#         self.ros_bridge_tcp.send_message(advertise_msg)
#         # subscribe to ros topic
#         topic_name_from_ros = "/from_ros_rps"
#         subscribe_msg = {
#             "op": "subscribe",
#             "topic": topic_name_from_ros,
#             "type": "std_msgs/String"
#         }
#         self.ros_bridge_tcp.send_message(subscribe_msg)

#     def decide_win_hand(self, hand_types):
#         ## hand gesticulation with image processing ###
#         hand_type = random.choice(hand_types)
#         ###############################################
#         print("Windows side selected hand type '" + hand_type + "'")
#         return hand_type

#     def judge_rps_game(self, win_hand, ros_hand):
#         print("Win:", win_hand, "\tROS:", ros_hand)
#         ## judge rps game by using 2 hand_type ########
#         return random.choice(["win", "lose", "even"])
#         ###############################################

#     def start_game(self):
#         # Decide your hand
#         hand_type = self.decide_win_hand(self.hand_types)

#         # Send your hand type to ROS, and wait ROS robot's hand type.
#         pub_msg = {
#             "op": "publish",
#             "topic": self.topic_name_from_win,
#             "msg": {"data": hand_type}
#         }
#         message_from_ros = self.ros_bridge_tcp.wait_response(pub_msg, self.hand_types, timeout=30)
#         if message_from_ros:
#             print("Receive from ROS:" + message_from_ros)
#             result = self.judge_rps_game(hand_type, message_from_ros)
#             pub_msg = {
#                 "op": "publish",
#                 "topic": self.topic_name_from_win,
#                 "msg": {"data": result}
#             }
#             # Send game result to ROS.
#             print(result)
#             self.ros_bridge_tcp.wait_response(pub_msg, timeout=10)
```

### ぐーちょきぱー
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



# class GameGCP():
#     def __init__(self):
#         self.ros_bridge_tcp = RosBridgeTCP()
#         self.topic_name_from_win = "/from_windows_gcp"
#         self.hand_types = ["gu", "pa", "choki"]
#         advertise_msg = {
#             "op": "advertise",
#             "topic": self.topic_name_from_win,
#             "type": "std_msgs/String"
#         }
#         self.ros_bridge_tcp.send_message(advertise_msg)
#         # subscribe to ros topic
#         topic_name_from_ros = "/from_ros_gcp"
#         subscribe_msg = {
#             "op": "subscribe",
#             "topic": topic_name_from_ros,
#             "type": "std_msgs/String"
#         }
#         self.ros_bridge_tcp.send_message(subscribe_msg)

#     def decide_win_hand(self, hand_types):
#         ## hand gesticulation with image processing ###
#         hand_type = random.choice(hand_types)
#         ###############################################
#         print("Windows side selected hand type '" + hand_type + "'")
#         return hand_type

#     def judge_gcp_game(self, win_hand, ros_hand):
#         print("Win:", win_hand, "\tROS:", ros_hand)
#         ## judge rps game by using 2 hand_type ########
#         return random.choice(["win", "lose", "even"])
#         ###############################################

#     def start_game(self, hand_types):
#         # Decide your hand
#         hand_type = self.decide_win_hand(hand_types)

#         # Send your hand type to ROS, and wait ROS robot's hand type.
#         pub_msg = {
#             "op": "publish",
#             "topic": self.topic_name_from_win,
#             "msg": {"data": hand_type}
#         }
#         message_from_ros = self.ros_bridge_tcp.wait_response(pub_msg, hand_types, timeout=30)
#         if message_from_ros:
#             print("Receive from ROS:" + message_from_ros)
#             result = self.judge_gcp_game(hand_type, message_from_ros)
#             pub_msg = {
#                 "op": "publish",
#                 "topic": self.topic_name_from_win,
#                 "msg": {"data": result}
#             }
#             # Send game result to ROS.
#             print(result)
#             self.ros_bridge_tcp.wait_response(pub_msg, timeout=10)
```
