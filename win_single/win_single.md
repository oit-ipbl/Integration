# Message exchange between a Windows program and a ROS node

[README](../README.md)

---

## Objectives

Let's make message exchange programs on windows and a ROS node that can communicate with each other via TCP/IP.
This page explains how to exchange messages between many image processing programs on Windows and the ROS node.

## Prerequisite

You have to finish all of [robots](https://github.com/oit-ipbl/robots) and [image processing](https://github.com/oit-ipbl/image_processing).

## Practice(Exchange messages between Windows and ROS container)

### Make a python program on Windows

- Save the following two files into `C:\oit\py21\code` on Windows.
  - If you want to download the files, click the following links and click a `Raw` button. After that, right-click on the screen, select the save a page to a file with a new name.
    - [ros_utils.py](https://raw.githubusercontent.com/oit-ipbl/Integration/main/ros_utils.py?token=AAOIT6P3QT2X54UTN6FAHLLA72Z3M)
    - [rosbridge_tcp.py](https://raw.githubusercontent.com/oit-ipbl/Integration/main/rosbridge_tcp.py?token=AAOIT6PZGCQYHDAXF2DTVZ3A72Z7A)
- Install `bson` package into the image processing development environment by using the following command.
  ```
  python -m pip install bson
  ```
- The communication process with the ROS named 'communication_with_ros_test.py' is as follows. Please save it to C:\oit\py21\code on Windows and edit it with VSCode. 
  - See [image processing development](https://github.com/oit-ipbl/portal/blob/main/setup/python%2Bvscode.md).

```python
#!/usr/bin/env python
# -*- coding: utf-8 -*-
import random
import time
from rosbridge_tcp import RosBridgeTCP
from ros_utils import build_ros_array_msg

def main():
    # for send message to ROS container
    # Wrapper class for TCP/IP communication
    ros_bridge_tcp = RosBridgeTCP()
    # Prepare to publish ROS topic from Windows
    topic_name_from_win = "/from_windows"
    advertise_msg = {
        "op": "advertise", # "op" should be "advertise"
        "topic": topic_name_from_win, # Topic name
        "type": "std_msgs/String"     # Topic type
    }

    # for recieve message from ROS container
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
```

### Make a ROS node in ROS Container

- The communication process with the Windows named `communication_test.py` is as follows. Please save it to `~/catkin_ws/src/oit_pbl_ros_samples/` in ROS cintainer and edit it with VSCode. 
  - See [Developing inside the ROS container with VSCode](https://github.com/oit-ipbl/portal/blob/main/setup/remote_with_vscode.md).
  - Allow the permission for the execution for this file.
  ```
  chmod u+x communication_test.py
  ```

```python
#!/usr/bin/env python
# -*- coding: utf-8 -*-
import math
import os
import random
import rospy
from std_msgs.msg import String
from utils import RosWinMessenger

def process():
    rospy.sleep(10)
    node_name = rospy.get_name()

    rate = rospy.Rate(2)  # Keep loop with 2hz
    # for send message to windows
    # Create a publisher which can publish String type topics.
    to_win_pub = rospy.Publisher("/from_ros", String, queue_size=1)

    # for recieve message from windows
    # Wrapper class to receive messages from Windows
    messenger = RosWinMessenger(to_win_pub, "/from_windows")

    for i in range(0, 10):   
        # Publish messages to Windows
        message = "Hello! this is ROS " + str(i)
        rospy.loginfo("%s:Sending message to win(%d):%s", node_name, i, message)
        # See https://github.com/oit-ipbl/robots/blob/main/basics/basics_01.md#summary-of-talkerpy
        to_win_pub.publish(message) 

        rate.sleep()

    # Receive messages from Windows
    for i in range(0, 10):
        message = messenger.wait_response(timeout=5)
        if message is not None:
            rospy.loginfo("%s:Receive from win(%d):%s", node_name, i, message)

def main():
    script_name = os.path.basename(__file__)
    rospy.init_node(os.path.splitext(script_name)[0])
    rospy.sleep(0.5)  # rospy.Time.now() returns 0, without this sleep.

    rospy.loginfo("%s:Started", rospy.get_name())

    process()
    rospy.loginfo("%s:Exiting", rospy.get_name())


if __name__ == '__main__':
    try:
        main()
    except Exception as e:
        rospy.logerr("%s:%s", rospy.get_name(), str(e))
        exit(1)
```

### Run the programs to exchange messages

#### Launch the ROS navigation and run the program in ROS Container

  - launch the ROS navigation.
  ```shell
  $ roslaunch oit_stage_ros navigation.launch
  ```

  - Open another terminal emulator and run `communication_test.py` like the following.
  ```shell
  $ rosrun oit_pbl_ros_samples communication_test.py
  ```

#### Run the program on Windows within about 10 seconds after launching the ROS programs

  - Open the windows terminal (or powershell) and move to `C:\oit\py21\code` 
  - Run `communication_with_ros_test.py` like the following.
  ```cmd
  C:\oit\py21\code>python ./communication_with_ros_test.py
  ```

- You can see the following output in the Windows terminal (or powershell). (Windows side)

```cmd
 ./communication_with_ros_test.py
{'topic': '/from_ros', 'msg': {'data': 'Hello! this is ROS 1'}, 'op': 'publish'}
{'topic': '/from_ros', 'msg': {'data': 'Hello! this is ROS 2'}, 'op': 'publish'}
{'topic': '/from_ros', 'msg': {'data': 'Hello! this is ROS 3'}, 'op': 'publish'}
{'topic': '/from_ros', 'msg': {'data': 'Hello! this is ROS 4'}, 'op': 'publish'}
{'topic': '/from_ros', 'msg': {'data': 'Hello! this is ROS 5'}, 'op': 'publish'}
{'topic': '/from_ros', 'msg': {'data': 'Hello! this is ROS 6'}, 'op': 'publish'}
{'topic': '/from_ros', 'msg': {'data': 'Hello! this is ROS 7'}, 'op': 'publish'}
{'topic': '/from_ros', 'msg': {'data': 'Hello! this is ROS 8'}, 'op': 'publish'}
{'topic': '/from_ros', 'msg': {'data': 'Hello! this is ROS 9'}, 'op': 'publish'}
Sending ros message: {'op': 'publish', 'topic': '/from_windows', 'msg': {'data': 'Hello this is Windows 0'}}
Sending ros message: {'op': 'publish', 'topic': '/from_windows', 'msg': {'data': 'Hello this is Windows 1'}}
Sending ros message: {'op': 'publish', 'topic': '/from_windows', 'msg': {'data': 'Hello this is Windows 2'}}
Sending ros message: {'op': 'publish', 'topic': '/from_windows', 'msg': {'data': 'Hello this is Windows 3'}}
Sending ros message: {'op': 'publish', 'topic': '/from_windows', 'msg': {'data': 'Hello this is Windows 4'}}
Sending ros message: {'op': 'publish', 'topic': '/from_windows', 'msg': {'data': 'Hello this is Windows 5'}}
Sending ros message: {'op': 'publish', 'topic': '/from_windows', 'msg': {'data': 'Hello this is Windows 6'}}
Sending ros message: {'op': 'publish', 'topic': '/from_windows', 'msg': {'data': 'Hello this is Windows 7'}}
Sending ros message: {'op': 'publish', 'topic': '/from_windows', 'msg': {'data': 'Hello this is Windows 8'}}
Sending ros message: {'op': 'publish', 'topic': '/from_windows', 'msg': {'data': 'Hello this is Windows 9'}}
```

- You can see the following output in the Terminal Emulator. (ROS node)

```shell
$ rosrun oit_pbl_ros_samples communication_test.py
[INFO] [1626179870.881054, 81.900000]: /communication_test:Started
[INFO] [1626179880.874691, 91.900000]: /communication_test:Sending message to win(0):Hello! this is ROS 0
[INFO] [1626179881.358568, 92.400000]: /communication_test:Sending message to win(1):Hello! this is ROS 1
[INFO] [1626179881.890727, 92.900000]: /communication_test:Sending message to win(2):Hello! this is ROS 2
[INFO] [1626179882.374326, 93.400000]: /communication_test:Sending message to win(3):Hello! this is ROS 3
[INFO] [1626179882.856942, 93.900000]: /communication_test:Sending message to win(4):Hello! this is ROS 4
[INFO] [1626179883.392875, 94.400000]: /communication_test:Sending message to win(5):Hello! this is ROS 5
[INFO] [1626179883.880240, 94.900000]: /communication_test:Sending message to win(6):Hello! this is ROS 6
[INFO] [1626179884.364151, 95.400000]: /communication_test:Sending message to win(7):Hello! this is ROS 7
[INFO] [1626179884.899607, 95.900000]: /communication_test:Sending message to win(8):Hello! this is ROS 8
[INFO] [1626179885.386283, 96.400000]: /communication_test:Sending message to win(9):Hello! this is ROS 9
[INFO] [1626179893.690161, 104.700000]: /communication_test:Receive from win(1):Hello this is Windows 0
[INFO] [1626179894.692005, 105.700000]: /communication_test:Receive from win(2):Hello this is Windows 1
[INFO] [1626179896.709420, 107.700000]: /communication_test:Receive from win(3):Hello this is Windows 3
[INFO] [1626179897.713940, 108.700000]: /communication_test:Receive from win(4):Hello this is Windows 4
[INFO] [1626179898.722476, 109.700000]: /communication_test:Receive from win(5):Hello this is Windows 5
[INFO] [1626179899.724748, 110.700000]: /communication_test:Receive from win(6):Hello this is Windows 6
[INFO] [1626179900.722184, 111.700000]: /communication_test:Receive from win(7):Hello this is Windows 7
[INFO] [1626179901.731512, 112.700000]: /communication_test:Receive from win(8):Hello this is Windows 8
[INFO] [1626179902.740314, 113.700000]: /communication_test:Receive from win(9):Hello this is Windows 9
[INFO] [1626179902.746700, 113.700000]: /communication_test:Exiting
```

### Sequence of the programs

- At first, ROS node, `communication_test.py`, sends messages to the Windows side program, `communication_with_ros_test.py`. The messages are like this `Hello! this is ROS 0`
- Windows side, `communication_with_ros_test.py`, outputs the received messages like this `{'topic': '/from_ros', 'msg': {'data': 'Hello! this is ROS 0'}, 'op': 'publish'}`. This communication process is repeated 10 times.
- After that the Windows side program, `communication_with_ros_test.py`, sends messages and ROS node receieves them. The exchanged messages are like this, `Hello this is Windows 0`.

### Important notice

- Please see the following example output.

  ```shell
  $ rosrun oit_pbl_ros_samples communication_test.py
  ...
  [INFO] [1626179893.690161, 104.700000]: /communication_test:Receive from win(1):Hello this is Windows 0
  [INFO] [1626179894.692005, 105.700000]: /communication_test:Receive from win(2):Hello this is Windows 1
  [INFO] [1626179896.709420, 107.700000]: /communication_test:Receive from win(3):Hello this is Windows 3
  [INFO] [1626179897.713940, 108.700000]: /communication_test:Receive from win(4):Hello this is Windows 4
  [INFO] [1626179898.722476, 109.700000]: /communication_test:Receive from win(5):Hello this is Windows 5
  ...
  [INFO] [1626179902.746700, 113.700000]: /communication_test:Exiting
  ```

  - You'd think "Where is `Hello this is Windows 2` ?". It cannot know. Actually, that message is missing.
  - ROS style communication using publisher and subscriber is not so reliable.
  - You have to consider this fact to integrate Windows and ROS programs. The most simple way is to send the same message multiple times.

### Increase communication reliability

The wrapper classes have usefule methods.
Add the following code to the programs.

- Windows side, `communication_with_ros_test.py`

```python
def main():
    ros_bridge_tcp = RosBridgeTCP()
    # publisher from windows
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
    tm = time.time()
    received = 0
    while time.time() - tm < 20 and received < 10:
        messages = ros_bridge_tcp.wait()
        received += len(messages)
        for m in messages:
            print(str(m))
    for i in range(0, 10):
        message = "Hello this is windows " + str(i)
        pub_msg = {
            "op": "publish",
            "topic": topic_name_from_win,
            "msg": {"data": message}
        }
        ros_bridge_tcp.send_message(pub_msg)
        print("Sending ros message: " + str(pub_msg))
        time.sleep(1)
    
    # Add from here
    hand_types = ["rock", "paper", "scissors"]
    hand_type = random.choice(hand_types)
    print("Windows side selected hand type '" + hand_type + "'")

    # Send your hand type to ROS, and wait ROS robot's hand type.
    pub_msg = {
        "op": "publish",
        "topic": topic_name_from_win,
        "msg": {"data": hand_type}
    }
    # 1st argument: Message for publishing.
    # 2nd argument: Target keywords to receive. Your program will wait until receiving one of the word in the array, 'hand_types'.
    message_from_ros = ros_bridge_tcp.wait_response(pub_msg, hand_types, timeout=30)
    if message_from_ros:
        print("Receive from ROS:" + message_from_ros)

    # Judge
    result = random.choice(["win", "lose", "even"])
    pub_msg = {
        "op": "publish",
        "topic": topic_name_from_win,
        "msg": {"data": result}
    }
    # Send game result to ROS.
    ros_bridge_tcp.wait_response(pub_msg, timeout=10)
    # Add until here

    try:
        ros_bridge_tcp.terminate()
        ros_bridge_tcp = None
    except Exception as e:
        print(str(e))
```

- ROS side, `communication_test.py`

```python
    def process(self):
        node_name = rospy.get_name()
        rospy.sleep(10)
        rate = rospy.Rate(2)  # Keep loop with 2hz
        to_win_pub = rospy.Publisher("/from_ros", String, queue_size=1)
        messenger = RosWinMessenger(to_win_pub, "/from_windows")

        # Publish message to windows
        for i in range(0, 10):
            message = "Hello! this is ROS " + str(i)
            rospy.loginfo("%s:Sending message to win(%d):%s",
                          node_name, i, message)
            to_win_pub.publish(message)
            rate.sleep()

        # Receive message from windows
        for i in range(0, 10):
            message = messenger.wait_response(timeout=5)
            if message is not None:
                rospy.loginfo("%s:Receive from win(%d):%s",
                              node_name, i, message)

        # Add from here
        # Select robot's hand_type
        hand_types = ["rock", "paper", "scissors"]
        hand_type = random.choice(hand_types)
        rospy.loginfo("%s:Robot selects '%s'", node_name, hand_type)
        # Send robot's choice to windows game Rock, Paper, Scissors, and wait game result
        # 1st argument: Message for publishing.
        # 2nd argument: Target keywords to receive. Your program will wait until receiving one of the word in the array, ["win", "lose", "even"].
        message_from_win = messenger.wait_response(
            hand_type, ["win", "lose", "even"], timeout=30)
        if message_from_win:
            rospy.loginfo("%s:Receive from win:%s",
                          node_name, message_from_win)
        # Add until here
```

Run the programs with same procedure mentioned above. You can see the outputs like this.

- Windows side

```cmd
C:\oit\py21\code>python ./communication_with_ros_test.py
{'topic': '/from_ros', 'msg': {'data': 'Hello! this is ROS 1'}, 'op': 'publish'}
{'topic': '/from_ros', 'msg': {'data': 'Hello! this is ROS 2'}, 'op': 'publish'}
...
Sending ros message: {'op': 'publish', 'topic': '/from_windows', 'msg': {'data': 'Hello this is windows 8'}}
Sending ros message: {'op': 'publish', 'topic': '/from_windows', 'msg': {'data': 'Hello this is windows 9'}}
Windows side selected hand type 'scissors' # New output
Receive from ROS:rock # New output
```

- ROS side

```shell
$ rosrun oit_pbl_ros_samples communication_test.py
[INFO] [1626185350.386625, 5561.400000]: /communication_test:Started
[INFO] [1626185360.397046, 5571.400000]: /communication_test:Sending message to win(0):Hello! this is ROS 0
[INFO] [1626185360.885427, 5571.900000]: /communication_test:Sending message to win(1):Hello! this is ROS 1
...
[INFO] [1626185380.094479, 5591.100000]: /communication_test:Receive from win(8):Hello this is windows 7
[INFO] [1626185382.111843, 5593.100000]: /communication_test:Receive from win(9):Hello this is windows 9
[INFO] [1626185382.115293, 5593.100000]: /communication_test:Robot selects 'rock' # New output
[INFO] [1626185385.136705, 5596.100000]: /communication_test:Receive from win:lose  # New output
[INFO] [1626185385.140013, 5596.100000]: /communication_test:Exiting
```

The wrapper class, `RosBridgeTCP` and `RosWinMessenger`, have `wait_response` method.
The method can specify target keywords to receive as the 2nd argument, and block the program until receiving one of the keywords.
The samples use this method and implement a dummy Rock, Paper and Scissors game.

## Exercise (integration 1)

The Windows program choice game result rondomly. See `communication_with_ros_test.py`.

```python
    # Judge
    result = random.choice(["win", "lose", "even"])
    pub_msg = {
        "op": "publish",
        "topic": topic_name_from_win,
        "msg": {"data": result}
    }
    # Send game result to ROS.
    ros_bridge_tcp.wait_response(pub_msg, timeout=10)
```

However, the Windows side program knows both robot's hand type and Windows side hand type, that is selected randomly.  
Modify the program to return correct judgement result, which is led from the both hand types.  
Of-course, the result should be send from Windows to ROS.
