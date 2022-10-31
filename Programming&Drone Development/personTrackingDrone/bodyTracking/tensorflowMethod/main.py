import cv2 as cv
from tracker import GetVideo, Tracker
from drone import DroneFunctions
from dronekit import VehicleMode

#OpenCV Init
capture = GetVideo().start()
tracker = Tracker()
drone = DroneFunctions()

#Drone
drone.arm_and_takeoff(2)

#Main Loop
while True:
    frame = capture.frame
    frame, differences, area = tracker.process(frame=frame)
    drone.move(differences, area)

    print(area)

    cv.imshow("frame", frame)

    if cv.waitKey(1) and drone.vehicle.mode == VehicleMode("LAND"):#& 0xFF == ord('q'): #
        drone.move(0, 0)
        capture.stop()
        break
