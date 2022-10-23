from dronekit import connect, VehicleMode
from pymavlink import mavutil

import socket
from _handTracker import HandDetector

import time
import cvzone
import cv2 as cv

#Hands
detect = HandDetector(maxHands=1)
#OpenCV
capture = cv.VideoCapture(0, cv.CAP_DSHOW)
fpsReader = cvzone.FPS()
#Drone
cw = True
yaw_value = 0

def arm_and_takeoff(alt):
    while not vehicle.is_armable:
        print("Waiting for initialization")
        time.sleep(1)
    print("Arming motors")
    vehicle.mode = VehicleMode("GUIDED")
    vehicle.armed = True
    while not vehicle.armed:      
        print(" Waiting for arming...")
        time.sleep(1)
    print("Taking off!")
    vehicle.simple_takeoff(alt)
def set_yaw(speed, cw=1, heading = 180):
    if speed == 0:
        heading = 0
    elif speed > 0:
        cw = -1
    else:
        cw = 1
        speed = int(speed / -1)
    msg = vehicle.message_factory.command_long_encode(
        0,0,
        mavutil.mavlink.MAV_CMD_CONDITION_YAW,
        0,
        heading,    #heading passed
        speed,
        cw,
        1,  #1 for relative #0 for absolute
        0,0,0)
    vehicle.send_mavlink(msg)

connectionString = socket.gethostbyname_ex(socket.gethostname())[-1][1] + ":14550"

print(f"Connecting to vehicle on {connectionString}")
vehicle = connect(connectionString)
arm_and_takeoff(10)

while True:
    #Read frame
    success, frame = capture.read()
    frame = cv.resize(frame, (640,480))
    frame = cv.flip(frame, 1)
    fps, frame = fpsReader.update(frame,pos=(50,80),color=(0,255,0),scale=5,thickness=5)

    yaw_Value, frame = detect.trackHands(frame) #detect hands

    #Drone Functions
    set_yaw(int(yaw_Value*180))

    #Break
    cv.imshow("test", frame)
    if cv.waitKey(1) & 0xFF == ord('q'):
        vehicle.mode = VehicleMode("LAND")
        break

capture.release()
cv.destroyAllWindows()