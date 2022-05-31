# Message exchange between a python program running on Windows and a ROS node running on ubuntu

[README](../README.md)

---

## Objectives

Let's develop a program to exchange messages between a Python program on Windows and a ROS node on ubuntu using TCP/IP.

## Prerequisite

You have to finish all `Pre-learning Assignments` of [robots](https://github.com/oit-ipbl/robots) and [image processing](https://github.com/oit-ipbl/image_processing).

## Practice(Exchange messages between python on Windows and ROS container on ubuntu)

### Make a python program on Windows

- Save the following two files into `C:\oit\py22_ipbl\code` on Windows.
  - If you want to download the files, click the following links and click a `Raw` button. After that, right-click on the screen, select the save a page to a file with a new name.
    - [ros_utils.py](../ros_utils.py)
    - [rosbridge_tcp.py](../rosbridge_tcp.py)
- Install `bson` package into the image processing development environment by using the following command.
  ```
  C:\\...\code> python -m pip install bson
  ```
- The communication process with the ROS named `communication_test_on_win.py` is as follows. Please save it to `C:\oit\py22_ipbl\code` on Windows and edit it with VSCode. 
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

- The communication process with the Windows named `communication_test_on_ros.py` is as follows. Please save it to `~/catkin_ws/src/oit_pbl_ros_samples/scripts/` in ROS cintainer and edit it with VSCode. 
  - See [Developing inside the ROS container with VSCode](https://github.com/oit-ipbl/portal/blob/main/setup/remote_with_vscode.md).
  - Allow the permission for the execution for this file.
  ```
  $ chmod u+x communication_test_on_ros.py
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

### :o:Exercise(Run the program to exchange messages)

#### Launch the ROS navigation and run the program in ROS Container

  - launch the ROS navigation.
  ```shell
  $ roslaunch oit_stage_ros navigation.launch
  ```

  - Open another terminal emulator and run `communication_test_on_ros.py` like the following.
  ```shell
  $ rosrun oit_pbl_ros_samples communication_test_on_ros.py
  ```

#### Run the program on Windows within about 10 seconds after launching the ROS programs

  - Open the windows terminal (or powershell) and move to `C:\oit\py22_ipbl\code` 
  - Run `communication_test_on_win.py` like the following.
  ```cmd
  C:\\...\code> python ./communication_test_on_win.py
  ```

- You can see the following output in the Windows terminal (or powershell). (Windows side)

```cmd
C:\\...\code> python ./communication_test_on_win.py
{'op': 'publish', 'topic': '/from_ros', 'msg': {'data': 'Hello! this is ROS 1'}}
{'op': 'publish', 'topic': '/from_ros', 'msg': {'data': 'Hello! this is ROS 2'}}
{'op': 'publish', 'topic': '/from_ros', 'msg': {'data': 'Hello! this is ROS 3'}}
{'op': 'publish', 'topic': '/from_ros', 'msg': {'data': 'Hello! this is ROS 4'}}
{'op': 'publish', 'topic': '/from_ros', 'msg': {'data': 'Hello! this is ROS 5'}}
{'op': 'publish', 'topic': '/from_ros', 'msg': {'data': 'Hello! this is ROS 6'}}
{'op': 'publish', 'topic': '/from_ros', 'msg': {'data': 'Hello! this is ROS 7'}}
{'op': 'publish', 'topic': '/from_ros', 'msg': {'data': 'Hello! this is ROS 8'}}
{'op': 'publish', 'topic': '/from_ros', 'msg': {'data': 'Hello! this is ROS 9'}}
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
$ rosrun oit_pbl_ros_samples communication_test_on_ros.py
[INFO] [1653977204.707825, 686.400000]: /communication_test_on_ros:Started
[INFO] [1653977214.698294, 696.400000]: /communication_test_on_ros:Sending message to win(0):Hello! this is ROS 0
[INFO] [1653977215.197496, 696.900000]: /communication_test_on_ros:Sending message to win(1):Hello! this is ROS 1
[INFO] [1653977215.693747, 697.400000]: /communication_test_on_ros:Sending message to win(2):Hello! this is ROS 2
[INFO] [1653977216.203684, 697.900000]: /communication_test_on_ros:Sending message to win(3):Hello! this is ROS 3
[INFO] [1653977216.703777, 698.400000]: /communication_test_on_ros:Sending message to win(4):Hello! this is ROS 4
[INFO] [1653977217.194854, 698.900000]: /communication_test_on_ros:Sending message to win(5):Hello! this is ROS 5
[INFO] [1653977217.696613, 699.400000]: /communication_test_on_ros:Sending message to win(6):Hello! this is ROS 6
[INFO] [1653977218.200533, 699.900000]: /communication_test_on_ros:Sending message to win(7):Hello! this is ROS 7
[INFO] [1653977218.696035, 700.400000]: /communication_test_on_ros:Sending message to win(8):Hello! this is ROS 8
[INFO] [1653977219.199850, 700.900000]: /communication_test_on_ros:Sending message to win(9):Hello! this is ROS 9
[INFO] [1653977229.503096, 711.200000]: /communication_test_on_ros:Receive from win(1):Hello this is Windows 0
[INFO] [1653977231.520796, 713.200000]: /communication_test_on_ros:Receive from win(2):Hello this is Windows 2
[INFO] [1653977232.518434, 714.200000]: /communication_test_on_ros:Receive from win(3):Hello this is Windows 3
[INFO] [1653977236.555830, 718.200000]: /communication_test_on_ros:Receive from win(4):Hello this is Windows 7
[INFO] [1653977238.570894, 720.200000]: /communication_test_on_ros:Receive from win(5):Hello this is Windows 9
[INFO] [1653977258.783111, 740.400000]: /communication_test_on_ros:Exiting
```

### Sequence of the programs

- At first, `communication_test_on_ros.py` in ROS node sends messages to the Windows side. The sending messages from ROS node are like `Hello! this is ROS 0` in this example.
- And `communication_test_on_win` on Windows receives the sent messages. The received messages is outputted like `{'topic': '/from_ros', 'msg': {'data': 'Hello! this is ROS 0'}, 'op': 'publish'}`. This communication process is repeated 10 times.
- After that `communication_test_on_win` on Windows sends messages. And sent messages are recieved by `communication_test_on_ros.py` in ROS node.
- The recieved messages are outputted like `Hello this is Windows 0`.

### Important notice

- Please see the following example output.

  ```shell
  $ rosrun oit_pbl_ros_samples communication_test_on_ros.py
  :
  :
  [INFO] [1626179893.690161, 104.700000]: /communication_test:Receive from win(1):Hello this is Windows 0
  [INFO] [1626179894.692005, 105.700000]: /communication_test:Receive from win(2):Hello this is Windows 1
  [INFO] [1626179896.709420, 107.700000]: /communication_test:Receive from win(3):Hello this is Windows 3
  [INFO] [1626179897.713940, 108.700000]: /communication_test:Receive from win(4):Hello this is Windows 4
  [INFO] [1626179898.722476, 109.700000]: /communication_test:Receive from win(5):Hello this is Windows 5
  :
  :
  [INFO] [1626179902.746700, 113.700000]: /communication_test:Exiting
  ```

  - You'd think where is the message, `Hello this is Windows 2`. We can't know because the message is lost.
  - Exchange communication using `publisher` and `subscriber` over the ROS is not so reliable.
  - You have to consider that to integrate Windows and ROS programs. The most simple way is to send the same message multiple times.

