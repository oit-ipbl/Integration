# Message exchange between a ROS node and multi Windows programs

[README](../README.md)

---

## Objectives

Let's Make multiple image processing programs on windows and a ROS node which can communicate with each other via TCP/IP.

This page explains how to communicate between many image processing programs on Windows and the ROS node.

## Prerequisite

You have to finish all of [robots](https://github.com/oit-ipbl/robots), [image processing](https://github.com/oit-ipbl/image_processing), and [Message exchange between a Windows program and a ROS node](win_single/win_single.md)

## Practice1(Single image processing program and a ros node)
### Make a Windows side python program
- Save the following two files into `C:\oit\py21\code` on Windows.
  - If you want to download the files, click the following links and then download from a `Raw` button.
  - [start_on_windows_single.py](./win/start_on_windows_single.py)
    - This program is the module that can call the all image processing programs on Windows. When executed, this program waits for messages from the ROS and calls the image processing program according to the received message.
  - [show_hand_game_win.py](./win/show_hand_game_win.py)
    - This program runs an image processing game while communicating with the ROS.

#### show_hand_game_win.py
- This program runs the image processing game `show hand game` in which the player shows own left or right hand to the camera according to the ROS message. This program uses `mediapipe` to recognize hands.
- You can test only this image processing game without communication with the ROS by using the following command.
  - When executed, the command calls `demo` function in the `show_hand_game_win.py`, and runs demo of `show hand game` without communication with the ROS.
  - We strongly recommend to implement the `demo` function like this even if you create new original image processing games.

```sh
C:\\...\code> python show_hand_game_win.py
```

- This program's `start_game` function is called from `start_on_windows_single.py` when the received ROS message is "[shg]start".
  - `start_game` function communicates with the ROS by sending/receiving message.
  - For example, `message_from_ros = ros_bridge_tcp.wait_response(pub_msg, hand_types, timeout=30)` runs the following three steps
    1. It sends message `pub_msg` to the ROS
    1. It waits up to `timeout=30` seconds for a message that matching the string in `hand_types` to receive. (* `hand_types` is the `list`, and it has any strings) If this 2nd argument is None, it receives every message and stops waiting.
    1. It store the received message in the `message_from_ros` if the message is received
  - It is strongly recommended to prefix the messages sent/received with the prefix corresponding to the image processing game which is communicating with the ROS.
```python
def start_game(topic_name_from_win, ros_bridge_tcp):
    # definition of message types receiving from ros
    hand_types = ["[shg]left", "[shg]right"]

    # Send start message and wait hand type selected by ROS
    pub_msg = {
        "op": "publish",
        "topic": topic_name_from_win,
        "msg": {"data": "[shg]start"}
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
    ros_bridge_tcp.wait_response(pub_msg, ["[shg]result"], timeout=5)
```

#### start_on_windows_single.py
- After confirming that `show_hand_game_win.py` runs normally at alone, Confirm that the game `show_hand_game` called from `start_on_windows_single.py` runs normally while communicating with the ROS, as following command.
  - Now, `start_on_windows_single.py` waits in the infinite loop for a message from the ROS with `while True:`, and exits the infinite loop when the message "[all]end" is received from the ROS.

```sh
C:\\...\code> python start_on_windows_single.py
```

- If you want to add a new image processing program that communicates with ROS (e.g. `show_hand_game_win.py`), you have to edit the following part of `start_on_windows_single.py`.
  - `if message['msg']['data'] == "[shg]start show hand game":` is the conditional expression that will be True when the message `[shg]start show hand game` is received from the ROS.
  - `shg.start_game(topic_name_from_win, ros_bridge_tcp)` calls `start_game` function of the image processing program (i.e. `show_hand_game_win.py`) that communicates with the ROS.
    - `import show_hand_game_win as shg` is also required.


```python
    while True:
        messages = ros_bridge_tcp.wait() #Wait for message from ROS and assign the response into 'messages'
        for message in messages:
            # If message['msg']['data'] is "[shg]start show hand game", then shg.star_game is invoked
            if message['msg']['data'] == "[shg]start":
                shgw.start_game(topic_name_from_win, ros_bridge_tcp)
            if message['msg']['data'] == "[all]end":
                pub_msg = {
                    "op": "publish",
                    "topic": topic_name_from_win,
                    "msg": {"data": "[all]finished"}
                }
                print("Every game has finished!")
                # Send message "Every game has finished!" to ros.
                ros_bridge_tcp.wait_response(pub_msg, ["[all]finished"], timeout=5)
                break
        else:
            continue
        break
```

### Make a ROS node
- Open `~/catkin_ws/` by Visual Studio Code editor, and add the following files into `~/catkin_ws/src/oit_pbl_ros_samples/scripts/`. See [Developing inside the ROS container with VSCode](https://github.com/oit-ipbl/portal/blob/main/setup/remote_with_vscode.md).

- Save the following two files into `~/catkin_ws/src/oit_pbl_ros_samples/scripts/` on ROS.
  - If you want to download the files, click the following links and then download from the `Raw` button.
  - [start_on_ros_single.py](./ros/start_on_ros_single.py)
    - This program runs on the ROS container, and then calls `show_hand_game_ros.py` which send the message to the image processing program on Windows.
  - [show_hand_game_ros.py](./ros/show_hand_game_ros.py)
    - This is the ROS program which communicates with the image processing game (i.e. `show_hand_game_win.py`) on the Windows.

#### show_hand_game_ros.py
- `play_show_hand_game` function of this program communicates with `show_hand_game_win.py` on Windows.
- Although this program is generally called from `start_on_ros_signal.py` as module, you can also run this program alone.
  - In order to run this program alone, the following steps are required on the ROS before running the command.
  - Allow the parmission for execution and run the launch program.
    ```sh
    $ chmod u+x show_hand_game_ros.py
    $ roslaunch oit_stage_ros navigation.launch
    ```
  - Next, run the image processing program on Windows.
    ```sh
    C:\\...\code> python start_on_windows_single.py
    ```
- When run alone by using the following command, this program will run the game `show hand game` while communicating with Windows and exit without stopping the infinite loop on the Windows side.
```sh
$ rosrun oit_pbl_ros_samples show_hand_game_ros.py
```

- The communication process between the ROS and Windows in `show_hand_game_ros.py` is as follows. Please read and understand the comments carefully.

```python
def play_show_hand_game():
    rospy.sleep(3) 
    node_name = rospy.get_name()
    # Prepare to play show hand game
    # Specify topic names to commnicate with show hand game
    to_win_pub = rospy.Publisher("/from_ros", String, queue_size=1)
    messenger = RosWinMessenger(to_win_pub, "/from_windows")
    # Start game sequence
    rospy.loginfo("%s:Try to start show hand game", node_name)

    # Send game start signal to Windows, and wait Windows side response.
    message_from_win = messenger.wait_response(
        "[shg]start", ["[shg]start"], timeout=30)
    if message_from_win:
        rospy.loginfo("%s:Receive from win:%s",
                          node_name, message_from_win)
    else:
        rospy.logerr("%s:Timeout. can't start show hand game on windows", node_name)
        return "[shg]timeout"

    # Select robot's hand_type
    hand_types = ["[shg]left", "[shg]right"]
    hand_type = random.choice(hand_types)
    rospy.loginfo("%s:Robot selects '%s'", node_name, hand_type)

    # Send hand_type chosen by robot to windows show hand game, and wait game result
    message_from_win = messenger.wait_response(
        hand_type, ["[shg]correct", "[shg]wrong"], timeout=30)
    if message_from_win:
        rospy.loginfo("%s:Receive from win:%s",
                    node_name, message_from_win)
    else:
        rospy.logerr(
            "%s:Timeout. can't get show hand game result on windows", node_name)
        return "[shg]timeout"
    rospy.sleep(3)
    return message_from_win            
```

#### start_on_ros_single.py
- After confirming that `show_hand_game_ros.py` runs normally, Confirm that both `start_on_ros_single.py` and `show_hand_game_ros.py` runs normally, as following command.
  - if you did not run the launch program, allow the parmission for execution and run the launch program on other terminal `roslaunch oit_stage_ros navigation.launch
`.
```sh
$ chmod u+x show_hand_game_ros.py
$ rosrun oit_pbl_ros_samples start_on_ros_single.py
```

- If you want to add a new ROS program (e.g. `show_hand_game_ros.py`) that communicates with Windows side, you have to edit the following part of `start_on_ros_single.py` as follows.
  - `result = shgr.play_show_hand_game()` calls `show_hand_game_ros.py` as module.
    - `import show_hand_game_ros as shgr` must be added.

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

### Reference
- The following figure shows the flow of programs when you run `show_hand_game_ros.py` alone on the ROS after running `start_on_windows_single.py` on Windows.
<image src="./communication_win-ros.png">

## Exercise (add another service)
### Windows side
- In Practice, you have implemented the communication process between `show_hand_game_win.py` and ROS (`show_hand_game_ros.py`).
- In this Exercise, you will add the bright and dark game.
- First, download `bright_dark_game_win.py` and save it in your `code` folder (on windows). 
  - If you want to download the file, click the link and then click the `Raw` button.
  - [bright_dark_game_win.py](./win/bright_dark_game_win.py)
    - An image processing program that communicates with ROS
  - This program is a game that brightens or darkens the camera image according to the "bright" and "dark" commands given by the ROS. You can adjust the brightness by pointing the camera at the light or covering the camera with your hand. If you can't switch between bright and dark as you want, try changing the parameters in the `judge_game` function in `bright_dark_game_win.py`.


- Next, run `bright_dark_game_win.py` and check if it works correctly.
  - In some cases, it may not start properly. Then, try restarting the Windows Terminal.

```sh
C:\\...\code> python bright_dark_game_win.py
```
- After confirming that the above program are executed correctly, copy `start_on_windows_single.py`, rename it to `start_on_windows_multi.py` and save it in the `code` folder (on Windows).
- Next, add the following if statement to the next line of `shg.start_game(topic_name_from_win, ros_bridge_tcp)` in `start_on_windows_multi.py` and add the import statement `import bright_dark_game_win as bdg` at the beginning of the program.
  - The following code calls the `start_game` function in `bright_dark_game_win.py`.

```python
            if message['msg']['data'] == "[bdg]start":
                bdg.start_game(topic_name_from_win, ros_bridge_tcp)
```

- Now you have a bright dark game program (on windows side) ready to communicate with the ROS side.

### ROS side

- Download `bright_dark_game_ros.py` and save it in the folder `~/catkin_ws/src/oit_pbl_ros_samples/scripts/` (in the ROS container). 
  - [bright_dark_game_ros.py](./ros/bright_dark_game_ros.py)
    - This is a ROS-side program to communicate with `bright_dark_game_win.py`.
    - Open `~/catkin_ws/` in the ROS container by Visual Studio Code editor, and add the above file into `~/catkin_ws/src/oit_pbl_ros_samples/scripts/ . See [Developing inside the ROS container with VSCode](https://github.com/oit-ipbl/portal/blob/main/setup/remote_with_vscode.md).
- After you have correctly placed `bright_dark_game_ros.py` in the ROS container, run the following command in the ROS container to see if it works correctly.
  - If you did not run the launch program, run the launch program on other terminal `roslaunch oit_stage_ros navigation.launch`


```sh
$ chmod u+x bright_dark_game_ros.py
```

- Next, start the following program on Windows side.

```sh
C:\\...\code> python start_on_windows_multi.py
```

- Next, if you run the following command in the ROS container, the `demo()` function of the `bright_dark_game_ros.py` will be called through ROS, and communication will be established between ROS side and Windows side.
  - If you run only `bright_dark_game_ros.py`, `start_on_windows_multi.py` will not be terminated because the termination process is not sent to Windows side from ROS side. If you want to exit `start_on_windows_multi.py`, send `Ctr+C`.

```sh
$ rosrun oit_pbl_ros_samples bright_dark_game_ros.py 
``` 

- Once you have verified that `bright_dark_game_ros.py` runs correctly, copy `start_on_ros_single.py`, rename it to `start_on_ros_multi.py`, and save it in the `~/catkin_ws/src/oit_ pbl_ros_samples/scripts/` folder (in the ROS container). 
- Then, add a procedure invoking the  `bright_dark_game_ros.py` in `start_on_ros_multi.py`.
  - Add the following code snippet before `end_game()` in `process()` of `start_on_ros_multi.py`.
  - `import bright_dark_game_ros as bdg` is also required.

```python
    print("---bdg---")
    result_bdg = bdg.play_bright_dark_game()
    rospy.sleep(5)
```
- This code fragment shows the process of executing `play_bright_dark_game()` in `bright_dark_game_ros.py` and receiving the result.

- Execute the following command to check if the communication between multiple services on the ROS-side and Windows-side works.
  - If you did not run the launch program, run the launch program on other terminal `roslaunch oit_stage_ros navigation.launch`

- First, start the program on the Windows side.
```sh
C:\\...\code> python start_on_windows_multi.py
```

- Next, run `start_on_ros_multi.py` in the ROS container.
```sh
$ rosrun oit_pbl_ros_samples start_on_ros_multi.py
```
- Make sure that the show hand game and the bright dark game are executed in sequence while communicating between the ROS side and Windows side programs.


## Exercise (game and navigation))

- Add navigation function into Exercise(add another service) programs. See [Robot control 3](https://github.com/oit-ipbl/robots/blob/main/robot_control/robot_control_03.md#robot-control-3).

- In this Exercise, the program on the Windows side is the same as the program in Exercise (Add another service)

### ROS side
- Download `move_robot_ros.py` and save it in the folder `~/catkin_ws/src/oit_pbl_ros_samples/scripts/` (in the ROS congainer). 
  - [move_robot_ros.py](./ros/move_robot_ros.py)
    - A program to move the robot to the specified location.
    - Open `~/catkin_ws/` in the ROS container, by Visual Studio Code editor, and add the file into `~/catkin_ws/src/oit_pbl_ros_samples/scripts/`. See [Developing inside the ROS container with VSCode](https://github.com/oit-ipbl/portal/blob/main/setup/remote_with_vscode.md).

- After you have correctly placed `move_robot_ros.py` in the ROS container, run the following command in the ROS container to see if it works correctly.
- Run the following command in the ROS container
  - If you did not run the launch program, run the launch program on other terminal `roslaunch oit_stage_ros navigation.launch`

```sh
$ chmod u+x move_robot_ros.py
```

- Next, run the following command in the ROS container, and the `demo()` function of `move_robot_ros.py` will be called through ROS, and the robot will move to the specified location on the stage simulator.

```sh
$ rosrun oit_pbl_ros_samples move_robot_ros.py 
``` 

- After checking that `move_robot_ros.py` works correctly, add a procedure to `start_on_ros_multi.py` that calls `move_robot_ros.py`.
  - Add the following code snippet before the line `print("---bdg---")` in the `process()` function of `start_on_ros_multi.py`. This will call `process()` in `move_robot_ros.py` to move the robot.
  - `import move_robot_ros.py as mrobot` is also required.

```python
    print("---nav---")
    rospy.loginfo("%s:Started", rospy.get_name())
    mrobot.process(x=2.0, y=2.5, angle=270)
    rospy.loginfo("%s:Exiting", rospy.get_name())        
    rospy.sleep(10)
```

- Next, add the following code snippet before the line `end_game()`.

```python
    print("---nav---")
    rospy.loginfo("%s:Started", rospy.get_name())
    mrobot.process(x=3.0, y=3.6, angle=90)
    rospy.loginfo("%s:Exiting", rospy.get_name())        
    rospy.sleep(5)
```

- With these code snippets, the robot will move to the specified coordinates after the Show Hand Game and after the Bright Dark Game, respectively.

- Execute the following commands to check if the communication between multiple services in the ROS side and Windows side and robot navigation on the stage simulator works properly.
  - If you did not run the launch program, run the launch program on other terminal `roslaunch oit_stage_ros navigation.launch`

- First, start the program on the Windows side.
```sh
C:\\...\code> python start_on_windows_multi.py
```

- Next, run `start_on_ros_multi.py` in the ROS container.
```sh
$ rosrun oit_pbl_ros_samples start_on_ros_multi.py
```
- Make sure that the show hand game and the bright dark game are executed in sequence while communicating between the ROS side and Windows side programs, and the robot moves to the specified coordinates on the stage simulator.
