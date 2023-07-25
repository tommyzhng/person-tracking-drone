import cv2 as cv
from tracker import GetVideo, Tracker
from drone import DroneFunctions, VehicleMode
import time

#OpenCV Init
drone = DroneFunctions()
capture = GetVideo().start()
tracker = Tracker() 
#Drone
drone.arm_and_takeoff(3)
time.sleep(1)

#Main Loop
while True:
    frame = capture.frame
    frame, differences, area = tracker.process(frame=frame)
    drone.move(differences, area)
    print(f"{round(differences*100, 3)} %")
    print(f"Area: {area}")

    cv.imshow("tracker", frame)

    if drone.vehicle.mode == VehicleMode("LAND"):
        drone.move(0,0)
        capture.stop()
        break

    if cv.waitKey(1) & 0xFF == ord('q'):
        drone.move(0,0)
        drone.vehicle.mode = VehicleMode("LAND")
        capture.stop()
        break
