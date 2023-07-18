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
## color piker

### sample02.py
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


## Tello Rotate

### sample03.py

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
