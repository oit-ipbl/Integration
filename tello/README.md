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
