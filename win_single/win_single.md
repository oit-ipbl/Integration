# Message exchange between Windows program and ROS node

[README](../README.md)

---

## Objectives

WindowsからTCP/IPでROSトピックをパブリッシュする。また、ROSからパブリッシュしたトピックをWindowsで受信する。

## Prerequisite

You have to finish all of [robots](https://github.com/oit-ipbl/robots) and [image processing](https://github.com/oit-ipbl/image_processing).

## Practice

### Make a windows side python program

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
    while time.time() - tm < 40 and received < 10:
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

    try:
        ros_bridge_tcp.terminate()
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
import actionlib
import rospy
from geometry_msgs.msg import Twist
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from std_msgs.msg import String
from tf.transformations import quaternion_from_euler
from geometry_msgs.msg import Quaternion
from utils import navigation, wait_string_message


class CommunicationTestNode(object):
    def __init__(self, topic_name_from_win, topic_name_from_ros, topic_name_cmd_vel, move_base_name="move_base"):
        self.topic_name_from_win = topic_name_from_win
        self.to_win_pub = rospy.Publisher(
            topic_name_from_ros, String, queue_size=1)

    def process(self):
        sleep_time = 1
        node_name = rospy.get_name()
        ac = actionlib.SimpleActionClient(self.move_base_name, MoveBaseAction)
        # Waiting action server for navigation
        while not ac.wait_for_server(rospy.Duration(5)):
            rospy.loginfo(
                "%s:Waiting for the move_base action server to come up", node_name)
        rospy.loginfo("%s:The server %s comes up",
                      node_name, self.move_base_name)
        navigation(ac, 1.15, 2.42, math.radians(90))
        rospy.sleep(5)
        # Send message to windows
        for i in range(0, 10):
            self.to_win_pub.publish("Hello! this is ROS " + str(i))
            rospy.sleep(sleep_time)
        tm = rospy.get_time()
        # Recieve messsage from windows
        for i in range(0, 10):
            message_from_win = wait_string_message(self.topic_name_from_win, 2)
            if message_from_win:
                rospy.loginfo("%s:Receive from win(%d):%s",
                              node_name, i, message_from_win)
            if rospy.get_time() - tm > 20:
                break


def main():
    script_name = os.path.basename(__file__)
    rospy.init_node(os.path.splitext(script_name)[0])
    rospy.sleep(0.5)  # rospy.Time.now() returns 0, without this sleep.

    node = CommunicationTestNode(
        "/from_windows", "/from_ros", "/cmd_vel")
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

#### Run windows side program within about 10 seconds

```cmd
C:\oit\py21\code>python ./communication_with_ros_sample_01.py
```

- Windows side console output.

```cmd
C:\oit\py21\code>python ./communication_with_ros_sample_01.py
{'topic': '/from_ros', 'msg': {'data': 'Hello! this is ROS 0'}, 'op': 'publish'}
{'topic': '/from_ros', 'msg': {'data': 'Hello! this is ROS 1'}, 'op': 'publish'}
{'topic': '/from_ros', 'msg': {'data': 'Hello! this is ROS 2'}, 'op': 'publish'}
{'topic': '/from_ros', 'msg': {'data': 'Hello! this is ROS 3'}, 'op': 'publish'}
{'topic': '/from_ros', 'msg': {'data': 'Hello! this is ROS 4'}, 'op': 'publish'}
{'topic': '/from_ros', 'msg': {'data': 'Hello! this is ROS 5'}, 'op': 'publish'}
{'topic': '/from_ros', 'msg': {'data': 'Hello! this is ROS 6'}, 'op': 'publish'}
{'topic': '/from_ros', 'msg': {'data': 'Hello! this is ROS 7'}, 'op': 'publish'}
{'topic': '/from_ros', 'msg': {'data': 'Hello! this is ROS 8'}, 'op': 'publish'}
{'topic': '/from_ros', 'msg': {'data': 'Hello! this is ROS 9'}, 'op': 'publish'}
Sending ros message: {'op': 'publish', 'topic': '/from_windows', 'msg': {'data': 'Hello this is windows 0'}}
Sending ros message: {'op': 'publish', 'topic': '/from_windows', 'msg': {'data': 'Hello this is windows 1'}}
Sending ros message: {'op': 'publish', 'topic': '/from_windows', 'msg': {'data': 'Hello this is windows 2'}}
Sending ros message: {'op': 'publish', 'topic': '/from_windows', 'msg': {'data': 'Hello this is windows 3'}}
Sending ros message: {'op': 'publish', 'topic': '/from_windows', 'msg': {'data': 'Hello this is windows 4'}}
Sending ros message: {'op': 'publish', 'topic': '/from_windows', 'msg': {'data': 'Hello this is windows 5'}}
Sending ros message: {'op': 'publish', 'topic': '/from_windows', 'msg': {'data': 'Hello this is windows 6'}}
Sending ros message: {'op': 'publish', 'topic': '/from_windows', 'msg': {'data': 'Hello this is windows 7'}}
Sending ros message: {'op': 'publish', 'topic': '/from_windows', 'msg': {'data': 'Hello this is windows 8'}}
Sending ros message: {'op': 'publish', 'topic': '/from_windows', 'msg': {'data': 'Hello this is windows 9'}}
```

- ROS side terminal output.

```shell
$ rosrun oit_pbl_ros_samples communication_test.py 
[INFO] [1626173639.498166, 38.800000]: /communication_with_win_sample_01:Started
[INFO] [1626173639.814206, 39.100000]: /communication_with_win_sample_01:The server move_base comes up
[INFO] [1626173639.815497, 39.100000]: /communication_with_win_sample_01:Sending goal
[INFO] [1626173653.610203, 52.900000]: /communication_with_win_sample_01:Finished: (3)
[INFO] [1626173669.637060, 68.900000]: /communication_with_win_sample_01:Receive from win(0):Hello this is windows 2
[INFO] [1626173670.645778, 69.900000]: /communication_with_win_sample_01:Receive from win(1):Hello this is windows 3
[INFO] [1626173671.645056, 70.900000]: /communication_with_win_sample_01:Receive from win(2):Hello this is windows 4
[INFO] [1626173672.643695, 71.900000]: /communication_with_win_sample_01:Receive from win(3):Hello this is windows 5
[INFO] [1626173673.644215, 72.900000]: /communication_with_win_sample_01:Receive from win(4):Hello this is windows 6
[INFO] [1626173674.653763, 73.900000]: /communication_with_win_sample_01:Receive from win(5):Hello this is windows 7
[INFO] [1626173675.654142, 74.900000]: /communication_with_win_sample_01:Receive from win(6):Hello this is windows 8
[INFO] [1626173676.654058, 75.900000]: /communication_with_win_sample_01:Receive from win(7):Hello this is windows 9
[INFO] [1626173680.670933, 79.900000]: /communication_with_win_sample_01:Exiting
```

### Overview of the programs

- At first, robot will navivgate to point (1.15, 2.42).
- Next, ROS node, `communication_with_win_sample_01.py`, sends messages to windows side program, `communication_with_ros_sample_01.py`. `communication_with_ros_sample_01.py` outputs the received messages like this `{'topic': '/from_ros', 'msg': {'data': 'Hello! this is ROS 0'}, 'op': 'publish'}`.
- After that the Windows side program, `communication_with_ros_sample_01.py`, sends messages and ROS node receieves them.  
