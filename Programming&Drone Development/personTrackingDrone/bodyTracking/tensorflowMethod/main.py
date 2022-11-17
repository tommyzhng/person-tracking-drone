import cv2 as cv
from tracker import GetVideo, Tracker
from drone import DroneFunctions, VehicleMode
import time

#OpenCV Init
drone = DroneFunctions()
capture = GetVideo().start()
tracker = Tracker()
#Drone
#drone.arm_and_takeoff(2.5)


time.sleep(2)


#Main Loop
while True:
    frame = capture.frame
    frame, differences, area = tracker.process(frame=frame)
    #drone.move(differences, 0)
    print(f"{round(differences*100, 3)} %")


    if drone.vehicle.mode == VehicleMode("LAND"):
        capture.stop()
        break

    if cv.waitKey(1) & 0xFF == ord('q'):
        capture.stop()
        break
