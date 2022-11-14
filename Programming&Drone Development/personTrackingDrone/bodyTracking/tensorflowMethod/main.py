import cv2 as cv
from tracker import GetVideo, Tracker
from drone import DroneFunctions, VehicleMode
import time

#OpenCV Init
capture = GetVideo().start()
tracker = Tracker()
drone = DroneFunctions()

#Drone
drone.arm_and_takeoff(1.5)
time.sleep(2)

#Main Loop
while True:
    frame = capture.frame
    frame, differences, area = tracker.process(frame=frame)
    drone.move(differences, 0)
    print(f"{round(differences*100, 3)} %")

    cv.imshow("frame", frame)

    if cv.waitKey(1) and drone.vehicle.mode == VehicleMode("LAND"):#& 0xFF == ord('q'): #
        drone.move(0, 0)
        capture.stop()
        break
