import cv2 as cv
from _bodyTracker import BodyDetector
from drone import DroneFunctions

#OpenCV
capture = cv.VideoCapture(0, cv.CAP_DSHOW)
detect = BodyDetector()
#drone = DroneFunctions()

#Drone
#drone.arm_and_takeoff(2)

#Main Loop
while True:
    success, frame = capture.read()
    frame = cv.flip(frame, 1)

    frame, centerHuman, area = detect.trackBody(frame)
    differences = detect.distanceFromCenter(frame, centerHuman)

    #drone
    #drone.move(differences, area)

    cv.imshow("Body Tracking", frame)
    if cv.waitKey(1) & 0xFF == ord('q'):
        break