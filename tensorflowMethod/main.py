import cv2 as cv
from tracker import GetVideo, Tracker
from drone import DroneFunctions, VehicleMode
import time
import keyboard

#OpenCV Init
drone = DroneFunctions(testing=True)
capture = GetVideo().start()
tracker = Tracker() 


#Main Loop
while True:
    frame = capture.frame
    frame, differences, area = tracker.process(frame=frame)
    drone.move(differences, area)
    print(f"{round(differences*100, 3)} %")
    print(f"Area: {area}")
    cv.imshow("tracker", frame)

    #if t is pressed, takeoff
    if keyboard.is_pressed('t'):
        drone.arm_and_takeoff(2)
    
    #if l is pressed, land
    if keyboard.is_pressed('l'):
        drone.vehicle.mode = VehicleMode("LAND")

    #if q is pressed, quit
    if cv.waitKey(1) == ord('q'):
        drone.vehicle.mode = VehicleMode("LAND")
        capture.stop()
        break

    time.sleep(0.01)
