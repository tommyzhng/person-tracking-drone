import cv2 as cv
from tracker import GetVideo, Tracker
from drone import DroneFunctions
from dronekit import VehicleMode

#OpenCV Init
capture = GetVideo().start()
tracker = Tracker()
drone = DroneFunctions()

#Drone
drone.arm_and_takeoff(5)

#Main Loop
while True:
    frame = capture.frame
    frame, differences = tracker.process(frame=frame)
    drone.move(differences)

    cv.imshow("frame", frame)

    if cv.waitKey(1) and drone.vehicle.mode == VehicleMode("LAND"):
        drone.move(0)
        capture.stop()
        break
