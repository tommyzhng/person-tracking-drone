from dronekit import connect, VehicleMode
from pymavlink import mavutil
import time
import keyboard
import math

import cv2 as cv
import mediapipe as mp

#Hands
mpHands = mp.solutions.hands
mpDraw = mp.solutions.drawing_utils
hands = mpHands.Hands(max_num_hands=1, min_detection_confidence=0.5, min_tracking_confidence=0.5)

#OpenCV
capture = cv.VideoCapture(0, cv.CAP_DSHOW)

#Drone
cw = True
yaw_value = 0

def trackHands(frame, mpDraw, mpHands):
    frame.flags.writeable = False
    frame = cv.cvtColor(frame, cv.COLOR_BGR2RGB)

    result = hands.process(frame)
    frame.flags.writeable = True
    frame = cv.cvtColor(frame, cv.COLOR_RGB2BGR)

    if result.multi_hand_landmarks:
        for landmarks in result.multi_hand_landmarks:
            for index, landmark in enumerate(landmarks.landmark):
                if index == 9:
                    yaw_value = 2*(landmark.x-0.5)
            mpDraw.draw_landmarks(frame, landmarks, mpHands.HAND_CONNECTIONS)
    else:
        yaw_value = 0

    return yaw_value, frame

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

def set_yaw(heading):
    if heading < 0:
        cw = 1
        heading = heading / -1
    else:
        cw = -1
    msg = vehicle.message_factory.command_long_encode(
        0,0,
        mavutil.mavlink.MAV_CMD_CONDITION_YAW,
        0,
        heading,    #heading passed
        0,
        cw,
        1,  #1 for relative #0 for absolute
        0,0,0)
    vehicle.send_mavlink(msg)

connectionString = "192.168.1.4:14550"
print(f"Connecting to vehicle on {connectionString}")
vehicle = connect(connectionString)
arm_and_takeoff(10)

while True:
    #read frame
    success, frame = capture.read()
    frame = cv.resize(frame, (640,480))
    frame = cv.flip(frame, 1)
    yaw_Value, frame = trackHands(frame, mpDraw, mpHands)

    #Drone Functions
    print(yaw_Value*60)
    set_yaw(yaw_Value*60)

    #Break
    cv.imshow("test", frame)
    if cv.waitKey(1) & 0xFF == ord('q'):
        break
capture.release()
cv.destroyAllWindows()