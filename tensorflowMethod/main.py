import cv2 as cv
from tracker import GetVideo, Tracker
from drone import DroneFunctions, VehicleMode
import time
import keyboard

#OpenCV Inits
drone = DroneFunctions(testing=True)
capture = GetVideo().start()
tracker = Tracker() 

start_tracking = False

print("done setup")

#Main Loop
while True:
    if start_tracking:
        frame = capture.frame
        frame, differences, area = tracker.process(frame=frame)
        xDiff, velocity = drone.calculate_movement(differences, area)
        #print(f"{round(differences*100, 3)} %")
        print(f"xDiff: {xDiff}, velocity: {velocity}")
        cv.imshow("tracker", frame)
    if keyboard.is_pressed('s'):
        start_tracking = True

    #if t is pressed, takeoff
    if keyboard.is_pressed('t'):
        drone.arm_and_takeoff(3)
    
    #if l is pressed, land
    if keyboard.is_pressed('l'):
        drone.vehicle.mode = VehicleMode("LAND")

    #if q is pressed, quit
    if cv.waitKey(1) == ord('q'):
        drone.vehicle.mode = VehicleMode("LAND")
        capture.stop()
        break

    time.sleep(0.01)