# Message exchange between a ROS node and multi Windows programs

[README](../README.md)

---

## Objectives

Let's Make multiple image processing programs on windows and a ROS node which can communicate with each other via TCP/IP.

ここでは複数の画像処理プログラムとROSノードを連携させる方法を学修する

## Prerequisite

You have to finish all of [robots](https://github.com/oit-ipbl/robots), [image processing](https://github.com/oit-ipbl/image_processing), and [Message exchange between a Windows program and a ROS node](win_single/win_single.md)

## Practice1(Single image processing program and a ros node)
### Make a Windows side python program
- 以下の2つのファイルを`code`フォルダ(on windows)に保存しましょう． 
  - ファイルをダウンロードしたい場合はリンクをクリックしてから，`Raw`をクリックしてダウンロードしましょう.
  - [start_on_windows_single.py](./win/start_on_windows_single.py)
    - Windowsで最初に起動し，ROSからのメッセージを待つ．すべての画像処理プログラムはこのモジュールから呼び出される
  - [show_hand_game_win.py](./win/show_hand_game_win.py)
    - ROSとコミュニケーションする画像処理プログラム

#### show_hand_game_win.py
- このプログラムはmediapipesを利用して，コンピュータが指示した手をユーザがカメラに翳すゲームです
- もしあなたがこのプログラムをテストしたければ，あなたはこのプログラムを下記コマンドで実行できる．
  - `python show_hand_game_win.py`
  - あなたが画像処理プログラムを作成する場合も，単独で実行できる関数(e.x. demo())を用意することを強くお勧めする．
- このプログラムの start_game functionは `start_on_windows_single.py` から呼び出される．
  - start_game functionでは，ROSとの間でメッセージの送受信が行われる．例えば，`message_from_ros = ros_bridge_tcp.wait_response(pub_msg, hand_types, timeout=30)` では，pub_msgをROSに送信し，hand_typesに含まれる文字列がROSから返ってくることを最大30秒待ち，返り値をmessage_from_rosに保存する
  - ここで，ROSと交換するメッセージにはこのプログラムに属するメッセージであることを示すprefix(e.x. [shg])を付与することを我々は強く推奨する．
```python
def start_game(topic_name_from_win, ros_bridge_tcp):
    # definition of message types receiving from ros
    hand_types = ["[shg]left", "[shg]right"]

    # Send start message and wait hand type selected by ROS
    pub_msg = {
        "op": "publish",
        "topic": topic_name_from_win,
        "msg": {"data": "[shg]OK_start"}
    }
    message_from_ros = ros_bridge_tcp.wait_response(pub_msg, hand_types, timeout=30)
    if message_from_ros:
        print("\nReceive from ROS:", message_from_ros, "\n")

    # Decide your hand
    window_message = "show only your " + message_from_ros + " hand!"
    left, right = show_hand(window_message)

    # Judge
    result = judge_game(left, right, message_from_ros)
    pub_msg = {
        "op": "publish",
        "topic": topic_name_from_win,
        "msg": {"data": result}
    }
    # Send game result to ROS.
    print(result)
    # In this program, ros never return to the following wait_response
    ros_bridge_tcp.wait_response(pub_msg, ["[shg]OK_result"], timeout=5)
```

#### start_on_windows_single.py
- 新しくROSとコミュニケーションする画像処理プログラムを追加する場合は`start_on_windows_single.py`の下記部分を編集しなければならない
  - `if message['msg']['data'] == "[shg]start show hand game":` はROSから`[shg]start show hand game`というメッセージが届いたときに実行される条件式である
  - `shg.start_game(topic_name_from_win, ros_bridge_tcp)` はROSとコミュニケーションする画像処理プログラム（すなわち`show_hand_game_win.py`のstart_game functionを呼ぶ．
    - `import show_hand_game_win as shg` is also required.

```python
    while True:
        messages = ros_bridge_tcp.wait() #Wait for message from ROS and assign the response into 'messages'
        for message in messages:
            # If message['msg']['data'] is "[shg]start show hand game", then shg.star_game is invoked
            if message['msg']['data'] == "[shg]start show hand game":
                shg.start_game(topic_name_from_win, ros_bridge_tcp)
            if message['msg']['data'] == "The end":
                pub_msg = {
                    "op": "publish",
                    "topic": topic_name_from_win,
                    "msg": {"data": "Every game has finished!"}
                }
                print("Every game has finished!")
                # Send message "Every game has finished!" to ros.
                ros_bridge_tcp.wait_response(pub_msg, ["Every game has finished"], timeout=5)
                break
        else:
            continue
        break
```

### Make a ROS node
- Open `~/catkin_ws/` by Visual Studio Code editor, and add the following files into `~/catkin_ws/src/oit_pbl_ros_samples/scripts/`. See [Developing inside the ROS container with VSCode](https://github.com/oit-ipbl/portal/blob/main/setup/remote_with_vscode.md).

- 以下の2つのファイルを`~/catkin_ws/src/oit_pbl_ros_samples/scripts/`フォルダ(on ROS)に保存しましょう． 
  - ファイルをダウンロードしたい場合はリンクをクリックしてから，`Raw`をクリックしてダウンロードしましょう.
  - [start_on_ros_single.py](./ros/start_on_ros_single.py)
    - Windowsで最初に起動し，ROSからのメッセージを待つ．すべての画像処理プログラムはこのモジュールから呼び出される
  - [show_hand_game_ros.py](./ros/show_hand_game_ros.py)
    - `show_hand_game_win.py`とコミュニケーションするROS側プログラム

#### show_hand_game_ros.py
- このプログラムの `play_show_hand_game` functionが`show_hand_game_win.py`と通信を行う．
- 通常，このプログラムは`start_on_ros_single.py` からモジュールとして呼び出されるが，デモ用に単独で呼び出すこともできる
- このファイルを単独で実行するためには下記のStepが必要
```sh
chmod u+x show_hand_game_ros.py
roslaunch oit_stage_ros navitation.launch
```
- 次にWindows側のプログラムをWindowsで起動する．
  - `python show_hand_game_win.py`
    - `python start_on_windows_single.py`でも構いません
- 次に` rosrun oit_pbl_ros_samples show_hand_game_ros.py` と実行するとROSを通して`demo()` functionが呼び出され，ROSとWindowsの間で通信が行われる

```python
def process():
    rospy.sleep(10)
    node_name = rospy.get_name()

    print("---shg---")
    result = shgr.play_show_hand_game()
    rospy.sleep(5)
    end_game()
    rospy.loginfo("/* GAME:%s */", result)
```

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
