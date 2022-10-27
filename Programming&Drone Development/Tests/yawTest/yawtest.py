import time
import cv2 as cv
from drone import DroneFunctions, VehicleMode

#OpenCV
capture = cv.VideoCapture(0, cv.CAP_DSHOW)
drone = DroneFunctions()

#Drone
drone.arm_and_takeoff(2)
counter = 0
#Main Loop
while True:
    success, frame = capture.read()
    frame = cv.flip(frame, 1)

    drone.yawX(50)
    time.sleep(0.2)

    if counter >= 330:
        drone.yawX(degree = 20, heading = 0, relative=0)
        drone.vehicle.mode = VehicleMode("LAND")
    else:
        counter += 5


    cv.imshow("Body Tracking", frame)
    if cv.waitKey(1) and drone.vehicle.mode == VehicleMode("LAND"):
        break