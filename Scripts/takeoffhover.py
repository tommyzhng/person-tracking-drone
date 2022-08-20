from dronekit import connect, VehicleMode, LocationGlobalRelative, LocationGlobal, Command
from pymavlink import mavutil
import time
import keyboard


#Connect to Vehicle
print("Connecting to vehicle")
vehicle = connect('udp:127.0.0.1:14551')



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
    time.sleep(5)

arm_and_takeoff(10)

while True:
    altitude = vehicle.location.global_relative_frame.alt
    time.sleep(0.5)
    altitude2 = vehicle.location.global_relative_frame.alt
    descentrate = (altitude2 - altitude) * 2
    
    print(descentrate)
    if keyboard.is_pressed('l'):
        vehicle.mode = VehicleMode("LAND")


            





