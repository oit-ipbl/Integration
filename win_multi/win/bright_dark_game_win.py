#!/usr/bin/env python
# -*- coding: utf-8 -*-
import random
import cv2
import time

device = 0 # camera device number

def getFrameNumber(start:float, fps:int):
    now = time.perf_counter() - start
    frame_now = int(now * 1000 / fps)

    return frame_now


def bright_dark(win_msg):
    global device

    cap = cv2.VideoCapture(device)
    fps = cap.get(cv2.CAP_PROP_FPS)
    wt  = cap.get(cv2.CAP_PROP_FRAME_WIDTH)
    ht  = cap.get(cv2.CAP_PROP_FRAME_HEIGHT)

    print("Size:", ht, "x", wt, "/Fps: ", fps)

    cv2.namedWindow("bright dark game", cv2.WINDOW_NORMAL)
    cnt = 0
    cnt_brightness = 0
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

        gframe = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        brightness = gframe.mean()

        cv2.putText(frame, win_msg, (50, 50), cv2.FONT_HERSHEY_COMPLEX, 1.0, (0,255,0), 2)
        if cnt > 60:
            cv2.putText(frame, "Start!!", (150, 450), cv2.FONT_HERSHEY_COMPLEX, 1.0, (0,0,255), 2)
        else:
            cv2.putText(frame, str(int((60-cnt)/15)), (150, 450), cv2.FONT_HERSHEY_COMPLEX, 1.0, (0,0,255), 2)

        if cnt > 60:
            cnt_brightness += brightness
        
        if cnt > 90: # 90-60 = 30frames
            break
        
        cv2.imshow("bright dark game", frame)
        cnt += 1

        key = cv2.waitKey(15) & 0xFF
        if key == 27:
            break

    cap.release()
    cv2.destroyWindow("bright dark game")

    return cnt_brightness/30 #30frames


def judge_game(brightness, ros_order):
    print("mean brightness:", brightness)
    result = "[bdg]wrong"
    if ros_order == "[bdg]bright":
        if brightness > 180:
            result = "[bdg]correct"
    else: # ros_order == "dark"
        if brightness < 50:
            result = "[bdg]correct"

    return result


def start_game(topic_name_from_win, ros_bridge_tcp):
    ros_orders = ["[bdg]bright", "[bdg]dark"]

    # Send start message and wait ros_orders selected by ROS
    pub_msg = {
        "op": "publish",
        "topic": topic_name_from_win,
        "msg": {"data": "[bdg]OK_start"}
    }
    message_from_ros = ros_bridge_tcp.wait_response(pub_msg, ros_orders, timeout=30)
    if message_from_ros:
        print("\nReceive from ROS:", message_from_ros, "\n")

    # brightness value is returned
    window_message = "Keep your screen " + message_from_ros + "!!"
    brightness = bright_dark(window_message)

    # Judge
    result = judge_game(brightness, message_from_ros)
    pub_msg = {
        "op": "publish",
        "topic": topic_name_from_win,
        "msg": {"data": result}
    }
    # Send game result to ROS.
    print(result)
    # In this program, ros never return to the following wait_response
    ros_bridge_tcp.wait_response(pub_msg, ["[bdg]OK_result"], timeout=5)


def demo():

    ros_orders = ["[bdg]bright", "[bdg]dark"]
    message_from_ros = random.choice(ros_orders)
    print("\nReceive from ROS:", message_from_ros, "\n")

    window_message = "Keep your screen " + message_from_ros + "!!"
    brightness = bright_dark(window_message)

    # Judge
    result = judge_game(brightness, message_from_ros)
    print("\nWindows:", result, "\n")


if __name__=="__main__":
    demo()
