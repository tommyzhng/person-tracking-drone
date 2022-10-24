import cv2 as cv
from tracker import Tracker


#OpenCV Init
capture = Tracker()
#drone = DroneFunctions()

#Drone
#drone.arm_and_takeoff(5)

#Main Loop
while True:
    frame = capture.read()
    frame = cv.flip(frame, 1)


    cv.imshow("Body Tracking", frame)
    if cv.waitKey(1) & 0xFF == ord('q'):
        break
