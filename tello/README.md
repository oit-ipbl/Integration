# Integration

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

```


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
    tello.rotate_counter_clockwise(10)

    if cv2.waitKey(1) == ord('q'):
        break

tello.streamoff()
tello.land()

cv2.destroyAllWindows()
```
