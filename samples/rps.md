# Rock-paper-scissors game using message exchange between windows side and ros side

[README](../README.md)

---

## Objectives

This page explains how to make communication games using message exchange between windows side and ros side.

## Complicated Interactions between windows side and ros side

- Normally, communication games will start when the exchanging message is sent between the Windows side and the ROS side.
- Hereafter, the program using message exchange between the Windows side and the ROS side is implemented, after the above starting process has been done. 

### Make a Windows side python program
- Save the following two files into `C:\oit\py22_ipbl\code` on Windows.
  - If you want to download the files, click the following links and then download from a `Raw` button.
  - [rps_game_on_win.py](../win_single/win/rps_game_on_win.py)
    - This program has to run before any other programs, except that the navigation program(navigation.launch) runs.
    - When executed, this program waits for messages from the ROS. 

#### rps_game_on_win.py
- First, this program's `main()` function waits a message `[rps]start` from the ROS side. After confirming the `[rps]start` message from ROS,  send `[rps]start` from the Windows side to start the rock-paper-scissors game.
- `message_from_ros = ros_bridge_tcp.wait_response(pub_msg, hand_types, timeout=30)` runs the following three steps
    1. It sends message `pub_msg` to the ROS
    1. It waits up to `timeout=30` seconds for a message that matching the string in `hand_types` to recieve. (* `hand_types` is the `list`, and it has string message)
    1. It stores the recieved message in the `message_from_ros` if the message is recieved
  - It is strongly recommended to prefix the messages sent/received with the prefix corresponding to the communication games.
- judge_game function compares win_hand and ros_hand to determine the winner. Let's try to complete it.

### Make a ROS node
- Open `~/catkin_ws/` by Visual Studio Code editor, and add the following files into `~/catkin_ws/src/oit_pbl_ros_samples/scripts/`. See [Developing inside the ROS container with VSCode](https://github.com/oit-ipbl/portal/blob/main/setup/remote_with_vscode.md).
  - If you want to download the files, click the following links and then download from the `Raw` button.
  - [rps_game_on_ros.py](../win_single/ros/rps_game_on_ros.py)
    - This program runs on the ROS container, and send the message to the Windows side.

#### rps_game_on_ros.py
- `play_rps_game` function of this program communicates with `rps_game_on_win.py` on Windows.
- After preparing Publisher, send game start signal `[rps]start` to Windows, and wait Windows side response(2nd argument `[rps]start`).
  - `message_from_win = messenger.wait_response("[rps]start", ["[rps]start"], timeout=30)`
- After receiving `[rps]start` from Windows side, select rock-paper-scissors(hand_types) and send it to Windows side, and wait for the result (`["[rps]win", "[rps]draw", "[rps]lose"]`).

```python
def play_rps_game():
    rospy.sleep(3) 
    node_name = rospy.get_name()
    # Prepare to play rpsgame
    # Specify topic names to commnicate with rps game
    to_win_pub = rospy.Publisher("/from_ros", String, queue_size=1)
    messenger = RosWinMessenger(to_win_pub, "/from_windows")
    # Start game sequence
    rospy.loginfo("%s:Try to start rps game", node_name)

    # Send game start signal [rps]start to Windows, and wait Windows side response(2nd argument [rps]start).
    message_from_win = messenger.wait_response(
        "[rps]start", ["[rps]start"], timeout=30)
    if message_from_win:
        rospy.loginfo("%s:Receive from win:%s",
                          node_name, message_from_win)
    else:
        rospy.logerr("%s:Timeout. can't start rps game on windows", node_name)
        return "[rps]timeout"

    # Select robot's hand_type
    hand_types = ["[rps]rock", "[rps]paper", "[rps]scissors"]
    hand_type = random.choice(hand_types)
    rospy.loginfo("%s:Robot selects '%s'", node_name, hand_type)

    # Send hand_type chosen by robot to windows rps game, and wait game result
    message_from_win = messenger.wait_response(
        hand_type, ["[rps]win", "[rps]draw", "[rps]lose"], timeout=30)
    if message_from_win:
        rospy.loginfo("%s:Receive from win:%s",
                    node_name, message_from_win)
    else:
        rospy.logerr(
            "%s:Timeout. can't get rps game result on windows", node_name)
        return "[rps]timeout"
    rospy.sleep(3)
    return message_from_win      
```

### The execution order of these programs
- At first, you have to run the navigation program on the ROS side if you have not run that yet.

```sh
$ roslaunch oit_stage_ros navigation.launch
```

<!-- - 次に，Windows sideのプログラムを実行しましょう(rps_game_on_win.py)
  - このプログラムはROSからのメッセージを無限ループで待ちます -->
- You have to run the `rps_game_on_win.py` on the Windows side.
    - This program waits for the messages from the ROS as the infinite loop after this program is running.

```cmd
C:\\...\code> python rps_game_on_win.py
```

<!-- - 最後に，ROS sideのプログラム（rps_game_on_ros.py）を実行しましょう
  - このプログラムは最初に[rps]startというメッセージをWindows sideに送信します
  - 実行権限を付与するのを忘れないこと-->
- Finally, you have to run the `rps_game_on_ros.py` on the ROS side.
    - This program sends the message, "[ros]start", to the Windows side.
    - This program requires execute permission.

```sh
$ chmod u+x rps_game_on_ros.py
$ rosrun oit_pbl_ros_samples rps_game_on_ros.py
```

### :o:Exercise(Rock-Paper-Scissors)
<!-- - ROSsideの[rps]startメッセージをWindows sideが受信したあと，ゲームが実行されていることを確認しましょう -->
- Please confirm that the game process is running after the Windows side has been received the message "[rps] start" sent from the ROS side.

#### Windows side
```cmd
Wait ROS messages...

Receive from ROS: [rps]paper

win_hand: [rps]paper    ros_hand: [rps]paper
[rps]win
```

#### ROS side
```sh
the rosdep view is empty: call 'sudo rosdep init' and 'rosdep update'
[INFO] [1627481095.838116, 3946.900000]: /rps_game_ros:Started
---rps---
[INFO] [1627481108.835301, 3959.900000]: /rps_game_ros:Try to start rps game
[INFO] [1627481113.971241, 3965.000000]: /rps_game_ros:Receive from win:[rps]start
[INFO] [1627481113.973128, 3965.000000]: /rps_game_ros:Robot selects '[rps]paper'
[INFO] [1627481117.021211, 3968.000000]: /rps_game_ros:Receive from win:[rps]win
[INFO] [1627481129.951446, 3981.000000]: /rps_game_ros:Exiting
```

The wrapper class, `RosBridgeTCP` and `RosWinMessenger`, have `wait_response` method.
The method can specify target keywords to receive as the 2nd argument, and block the program until receiving one of the keywords.
This sample uses this method.
