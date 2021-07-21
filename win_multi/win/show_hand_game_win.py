#!/usr/bin/env python
# -*- coding: utf-8 -*-
#!!!!!This program must be executed on windows python3 environment!!!!!

import random
import cv2
import time
import mediapipe as mp

device = 0 # camera device number

def getFrameNumber(start:float, fps:int):
    now = time.perf_counter() - start
    frame_now = int(now * 1000 / fps)

    return frame_now


def show_hand(win_msg):
    global device

    cap = cv2.VideoCapture(device)
    fps = cap.get(cv2.CAP_PROP_FPS)
    wt  = cap.get(cv2.CAP_PROP_FRAME_WIDTH)
    ht  = cap.get(cv2.CAP_PROP_FRAME_HEIGHT)

    print("Size:", ht, "x", wt, "/Fps: ", fps)

    mp_hands = mp.solutions.hands
    hands = mp_hands.Hands(min_detection_confidence=0.8, min_tracking_confidence=0.8)

    cv2.namedWindow("show hand game", cv2.WINDOW_NORMAL)
    cnt = 0
    start = time.perf_counter()
    frame_prv = -1
    while cap.isOpened():
        frame_now = getFrameNumber(start, fps)
        if frame_now == frame_prv:
            continue
        frame_prv = frame_now

        ret, frame = cap.read()
        if not ret:
            print("Ignoring empty camera frame")
            continue

        cnt_left = cnt_right = 0

        frame = cv2.cvtColor(cv2.flip(frame, 1), cv2.COLOR_BGR2RGB)
        frame.flags.writeable = False
        results = hands.process(frame)
        frame.flags.writeable = True
        frame = cv2.cvtColor(frame, cv2.COLOR_RGB2BGR)
        mhlandmarks = results.multi_hand_landmarks
        cv2.putText(frame, win_msg, (50, 50), cv2.FONT_HERSHEY_COMPLEX, 1.0, (0,255,0), 2)
        if cnt > 60:
            cv2.putText(frame, "Start!!", (150, 450), cv2.FONT_HERSHEY_COMPLEX, 1.0, (0,0,255), 2)
        else:
            cv2.putText(frame, str(int((60-cnt)/15)), (150, 450), cv2.FONT_HERSHEY_COMPLEX, 1.0, (0,0,255), 2)
        if cnt > 60 and mhlandmarks:
            for i in range(len(mhlandmarks)):
                if results.multi_handedness[i].classification[0].label == 'Left':
                    cnt_left += 1
                if results.multi_handedness[i].classification[0].label == 'Right':
                    cnt_right += 1
            break

        if cnt > 120:
            print("Time over!!")
            break
        
        cv2.imshow("show hand game", frame)
        cnt += 1

        key = cv2.waitKey(15) & 0xFF
        if key == 27:
            break

    cap.release()
    cv2.destroyWindow("show hand game")

    return cnt_left, cnt_right

def judge_game(left, right, ros_hand):
    print("judge_game:left:", left, "\tright:", right)
    result = "[shg]wrong"
    if ros_hand == "[shg]left":
        if right == 0 and left > 0:
            result = "[shg]correct"
    else: # ros_hand == "right"
        if left == 0 and right > 0:
            result = "[shg]correct"

    return result

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


def demo_DIP_game():

    # Decide your hand
    hand_types = ["left", "right"]
    message_from_ros = random.choice(hand_types)
    print("\nReceive from ROS:", message_from_ros, "\n")

    window_message = "show only your " + message_from_ros + " hand!"
    left, right = show_hand(window_message)

    # Judge
    result = judge_game(left, right, message_from_ros)
    print("\nWindows:", result, "\n")


if __name__=="__main__":
    demo_DIP_game()
