import cv2 as cv
from tracker import GetVideo, Tracker

#OpenCV Init
capture = GetVideo().start()
tracker = Tracker()
#drone = DroneFunctions()

#Drone
#drone.arm_and_takeoff(5)

#Main Loop
while True:
    frame = capture.frame
    frame, center = tracker.process(frame=frame)
    

    cv.imshow("Body Tracking", frame)
    if cv.waitKey(1) & 0xFF == ord('q'):
        capture.stop()
        break
