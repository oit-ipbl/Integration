# Integration

## Tello Video

### sample01.py
```python
import cv2
from djitellopy import Tello

tello = Tello()

tello.connect()

tello.streamon()

while True:
    frame = tello.get_frame_read().frame

    cv2.imshow("Tello Video", frame)

    if cv2.waitKey(1) == ord('q'):
        break

tello.streamoff()

cv2.destroyAllWindows()

tello.end()
```

## Tello Rotate

### sample02.py

```python
import cv2
from djitellopy import Tello

tello = Tello()

tello.connect()

tello.streamon()
tello.takeoff()
while True:
    frame = tello.get_frame_read().frame

    cv2.imshow("Tello Video", frame)
    tello.rotate_counter_clockwise(20)

    if cv2.waitKey(1) == ord('q'):
        break

tello.streamoff()
tello.land()

cv2.destroyAllWindows()
```


## Color Picker
The following code displays the rgp and hsv values around the clicked location in Tello.

### sample03.py
```python
import cv2
import numpy as np
from djitellopy import Tello

def mouse_event(event, x, y, flg, prm):
   if event == cv2.EVENT_LBUTTONDOWN:
       img = np.ones((128, 128, 3), np.uint8)
       avbgr = np.array([(np.uint8)(np.average(frame[y-2:y+2, x-2:x+2, 0])),
                         (np.uint8)(np.average(frame[y-2:y+2, x-2:x+2, 1])),
                         (np.uint8)(np.average(frame[y-2:y+2, x-2:x+2, 2]))])
       img[:, :, 0] = img[:, :, 0] * avbgr[0]
       img[:, :, 1] = img[:, :, 1] * avbgr[1]
       img[:, :, 2] = img[:, :, 2] * avbgr[2]
       cv2.imshow('average color', img)
       print('bgr: ' + str(img[1, 1, :]))
       avhsv = cv2.cvtColor(np.array([[avbgr]], np.uint8), cv2.COLOR_BGR2HSV)
       print('hsv: ' + str(avhsv[0, 0, :]))


tello = Tello()
tello.connect()
tello.streamon()

while True:
    frame = tello.get_frame_read().frame
    cv2.imshow("Tello Video", frame)
    cv2.setMouseCallback('Tello Video', mouse_event)
    if cv2.waitKey(1) == ord('q'):
        break

tello.streamoff()
cv2.destroyAllWindows()
tello.end()
```
## Find the red

### sample04.py
```python
from djitellopy import Tello
import cv2

import time
tello = Tello()

tello.connect()

tello.streamon()

cv2.namedWindow("Tracking")

#lower_red = (0, 50, 50)
#upper_red = (15, 200, 200)


while True:
    frame = tello.get_frame_read().frame
    frame = cv2.resize(frame, (720, 480))
    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
    
    #mask = cv2.inRange(hsv, lower_red, upper_red)

    hsv_min = (0, 70, 70)
    hsv_max = (15, 255, 255)
    mask1 = cv2.inRange(hsv, hsv_min, hsv_max)
 
    hsv_min = (160, 70, 70)
    hsv_max = (179, 255, 255)
    mask2 = cv2.inRange(hsv, hsv_min, hsv_max)
 
    mask = mask1 + mask2
    
    contours, _ = cv2.findContours(mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
    
    if contours:
        largest_contour = max(contours, key=cv2.contourArea)
        if cv2.contourArea(largest_contour) > 500: 
            x, y, w, h = cv2.boundingRect(largest_contour)
            cv2.rectangle(frame, (x, y), (x + w, y + h), (0, 0, 255), 2)
            print("Area = {}".format(cv2.contourArea(largest_contour)) )
            print("(x ,y)=({},{})".format(x+w/2,y+h/2))
            #time.sleep(1)
    
    cv2.imshow("Tracking", frame)
    key = cv2.waitKey(1)
    if key == ord('q'):
        break
tello.streamoff()

cv2.destroyAllWindows()
tello.end()
```

## Stop when you see red

### sample05.py

```pyhton
from djitellopy import Tello
import cv2
import time

tello = Tello()
tello.connect()
tello.streamon()

print(f"Battery: {tello.get_battery()}%")
tello.takeoff()

cv2.namedWindow("Tracking")


while True:
    frame = tello.get_frame_read().frame
        
    frame = cv2.resize(frame, (720, 480))
    
    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
    
    hsv_min = (0, 70, 70)
    hsv_max = (15, 255, 255)
    mask1 = cv2.inRange(hsv, hsv_min, hsv_max)
 
    hsv_min = (160, 70, 70)
    hsv_max = (179, 255, 255)
    mask2 = cv2.inRange(hsv, hsv_min, hsv_max)
 
    mask = mask1 + mask2
    
    contours, _ = cv2.findContours(mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
    

    if contours:

        largest_contour = max(contours, key=cv2.contourArea)
        if cv2.contourArea(largest_contour) > 1000: 
            # 輪郭を囲む矩形を描画する
            x, y, w, h = cv2.boundingRect(largest_contour)
            cv2.rectangle(frame, (x, y), (x + w, y + h), (0, 0, 255), 2)
            print(cv2.contourArea(largest_contour) )
            print(x+w/2,y+h/2)
        else:
            tello.rotate_clockwise(30)
    cv2.imshow("Tracking", frame)
    
    key = cv2.waitKey(1)
    if key == ord('q'):
        break

tello.streamoff()

tello.land()

cv2.destroyAllWindows()
```
