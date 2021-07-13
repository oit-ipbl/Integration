# Message exchange between Windows program and ROS node

[README](../README.md)

---

## Objectives

Make a windonws program which can publish and receive ROS topic via TCP/IP.

## Prerequisite

You have to finish all of [robots](https://github.com/oit-ipbl/robots) and [image processing](https://github.com/oit-ipbl/image_processing).

## Practice

### Make a Windows side python program

- Download the following 2 files into Windows directory `C:\oit\py21\code`.
  - [ros_utils.py](../ros_utils.py)
  - [rosbridge_tcp.py](../rosbridge_tcp.py)
- Make a python file named `communication_with_ros_sample_01.py` in Windows directory `C:\oit\py21\code` and edit it with VSCode.
  - See [image processing development](https://github.com/oit-ipbl/portal/blob/main/setup/python%2Bvscode.md).

```python
#!/usr/bin/env python
# -*- coding: utf-8 -*-
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
```

### Make a ROS node

Open `~/catkin_ws/src/oit_pbl_ros_samples/` by Visual Studio Code editor, and edit `communication_test.py`. See [Developing inside the ROS container with VSCode](https://github.com/oit-ipbl/portal/blob/main/setup/remote_with_vscode.md).

Type the following template. It's OK copy and paste.

```python
#!/usr/bin/env python
# -*- coding: utf-8 -*-
import math
import os
import rospy
from std_msgs.msg import String
from utils import RosWinMessenger


class CommunicationTestNode(object):
    def __init__(self):
        pass

    def process(self):
        node_name = rospy.get_name()
        rospy.sleep(10)
        rate = rospy.Rate(2)  # Keep loop with 2hz
        # Create a publisher which can publish String type topics.
        to_win_pub = rospy.Publisher("/from_ros", String, queue_size=1)
        # Wrapper class to receive messages from Windows
        messenger = RosWinMessenger(to_win_pub, "/from_windows")

        # Publish messages to Windows
        for i in range(0, 10):
            message = "Hello! this is ROS " + str(i)
            rospy.loginfo("%s:Sending message to win(%d):%s",
                          node_name, i, message)
            # See https://github.com/oit-ipbl/robots/blob/main/basics/basics_01.md#summary-of-talkerpy
            to_win_pub.publish(message) 
            rate.sleep()

        # Receive messages from Windows
        for i in range(0, 10):
            message = messenger.wait_response(timeout=5)
            if message is not None:
                rospy.loginfo("%s:Receive from win(%d):%s",
                              node_name, i, message)


def main():
    script_name = os.path.basename(__file__)
    rospy.init_node(os.path.splitext(script_name)[0])
    rospy.sleep(0.5)  # rospy.Time.now() returns 0, without this sleep.

    node = CommunicationTestNode()
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

After a while run a sample program, `communication_test.py`.

```shell
$ rosrun oit_pbl_ros_samples communication_test.py.py
```

#### Run Windows side program within about 10 seconds

```cmd
C:\oit\py21\code>python ./communication_with_ros_sample_01.py
```

- Windows side console output.

```cmd
 ./communication_with_ros_sample_01.py
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

- ROS side terminal output.

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

- At first, ROS node, `communication_test.py`, sends messages to the Windows side program, `communication_with_ros_sample_01.py`. The messages are like this `Hello! this is ROS 0`
- Windows side, `communication_with_ros_sample_01.py`, outputs the received messages like this `{'topic': '/from_ros', 'msg': {'data': 'Hello! this is ROS 0'}, 'op': 'publish'}`. This commnication process is repeated 10 times.
- After that the Windows side program, `communication_with_ros_sample_01.py`, sends messages and ROS node receieves them. The exchanged messages are like this, `Hello this is Windows 0`.

### Important notice

See this output.

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

Where is `Hello this is Windows 2` ? I don't know. It's missing.
ROS style commnication which use publisher and subscriber is not so reliable.
You have to consider this fact to integrate Windows and ROS programs. The most simple way is send same message multiple times.
