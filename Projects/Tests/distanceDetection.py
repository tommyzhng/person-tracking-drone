import math
import cv2 as cv
from cvzone.HandTrackingModule import HandDetector

import numpy as np

#Hands
detect = HandDetector(detectionCon=0.8, maxHands=1)

#Find the function
x = [350, 180, 115, 88, 60, 53, 43, 43, 33, 31, 28]
y = [10, 20, 30, 40, 50, 60, 70, 80, 90, 100, 110]
coff = np.polyfit(x, y, 2)


capture = cv.VideoCapture(0, cv.CAP_DSHOW)
capture.set(3, 640)
capture.set(4, 480)

while True:
    #read frame
    success, frame = capture.read()
    frame = cv.flip(frame, 1)
    hands, frame = detect.findHands(frame)

    if hands:
        landmarks = hands[0]["lmList"]
        x1, y1 = landmarks[5][:2]
        x2, y2 = landmarks[17][:2]

        magnitude = int(math.sqrt((abs(x2-x1))**2 + (abs(y2-y1))**2))
        A, B, C = coff
        distanceCM = A*magnitude**2 + B*magnitude + C
        print(distanceCM)


    #Break
    cv.imshow("test", frame)
    if cv.waitKey(1) & 0xFF == ord('q'):
        break
capture.release()
cv.destroyAllWindows()