from dronekit import connect, VehicleMode, LocationGlobalRelative, LocationGlobal
from pymavlink import mavutil
import time
import keyboard

#Connect to Vehicle

cw = True
yaw_Val = 10

connectionString = "192.168.246.47:14550"
print(f"Connecting to vehicle on {connectionString}")
vehicle = connect(connectionString)

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
    
def set_yaw(heading, cw):
    if cw == True:
        cw = 1
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

arm_and_takeoff(10)

while True:
    if keyboard.is_pressed("w"):
        cw = True
        set_yaw(yaw_Val, cw)
    if keyboard.is_pressed("s"):
        cw = False
        set_yaw(yaw_Val, cw)
    if keyboard.is_pressed("l"):
        vehicle.mode = "LAND" 
        break
    
    



            





