import numpy as np
import cv2
import time
from picamera2 import Picamera2
picam2 = Picamera2()
picam2.configure(picam2.create_preview_configuration(main={"format": 'XRGB8888', "size": (640, 480)}))
picam2.start()

while True:
    frame = picam2.capture_array()
    frame2 = picam2.capture_array()

    hsv = cv2.cvtColor(frame2, cv2.COLOR_BGR2HSV)
    lower_red = np.array([60, 35, 140])
    upper_red = np.array([180, 255, 255])
    mask = cv2.inRange(hsv, lower_red, upper_red)
    result = cv2.bitwise_and(frame2, frame2, mask = mask)
    
    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    gray = cv2.medianBlur(gray, 5)
    
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break
    
    rows = gray.shape[0]
    circles = cv2.HoughCircles(gray, cv2.HOUGH_GRADIENT, 1, rows / 8,
                               param1=100, param2=30,
                               minRadius=1, maxRadius=100)
    
    
    if circles is not None:
        circles = np.uint16(np.around(circles))
        for i in circles[0, :]:
            center = (i[0], i[1])
            # circle center
            cv2.circle(frame, center, 1, (0, 100, 100), 3)
            # circle outline
            radius = i[2]
            cv2.circle(frame, center, radius, (255, 0, 255), 3)
    
    
    cv2.imshow("detected circles", frame)
    cv2.imshow("color", mask)
    cv2.imshow("red", result)
    time.sleep(.5)

# When everything done, release the capture
cv2.destroyAllWindows()