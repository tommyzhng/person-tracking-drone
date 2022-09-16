#Script for Takeoff & Control with Keyboard

import time
from dronekit import VehicleMode, connect, LocationGlobalRelative, Command, LocationGlobal
from pymavlink import mavutil
import tkinter as tk

connectionString = input("Connect to: ") or "192.168.1.4:14550"
print(f"Connecting to vehicle on {connectionString}")
vehicle = connect(connectionString)

#Parameters:
gndspd = 2 #m/s
arm = 0

def arm_and_takeoff(aTargetAltitude):
        print("Basic pre-arm checks")
        # Don't let the user try to arm until autopilot is ready
        while not vehicle.is_armable:
            print(" Waiting for vehicle to initialise...")
            time.sleep(1)
    
        print("Arming motors")
        # Copter should arm in GUIDED mode
        vehicle.mode = VehicleMode("GUIDED")
        vehicle.armed = True

        while not vehicle.armed:      
            print(" Waiting for arming...")
            time.sleep(1)

        print("Taking off!")
        vehicle.simple_takeoff(aTargetAltitude)

        while True:
            print(" Altitude: ", + vehicle.location.global_relative_frame.alt)      
            if vehicle.location.global_relative_frame.alt >= aTargetAltitude*0.95:
                print("Reached target altitude")
                break
            time.sleep(1)


def set_velocity_body(vehicle, vx, vy, vz):
    
    msg = vehicle.message_factory.set_position_target_local_ned_encode(
        0,
        0, 0,
        mavutil.mavlink.MAV_FRAME_BODY_NED,
        0b0000111111000111,     #BMSK -> 1 = ignore, 0 = read
        0, 0, 0,                #POS
        vx, vy, vz,              #VEL
        0, 0, 0,                #ACCEL
        0, 0)
    vehicle.send_mavlink(msg)
    vehicle.flush()



def key(event):  
    if event.char == event.keysym:
        if event.keysym == 'w':
            set_velocity_body(vehicle, gndspd, 0, 0)
        elif event.keysym == 's':
            set_velocity_body(vehicle, -gndspd, 0, 0)
        elif event.keysym == 'a':
            set_velocity_body(vehicle, 0, -gndspd, 0)
        elif event.keysym == 'd':
            set_velocity_body(vehicle, 0, gndspd, 0)

        elif event.keysym == 'r':
            vehicle.mode = VehicleMode("RTL")
            print("'R is pressed ->> Setting Vehicle to RTL")
        
    
            
#--MAIN
arm_and_takeoff(10)
root = tk.Tk()
print("Control Drone with WASD. Press B to ARM. Press R to return to RTL")
root.bind_all('<Key>', key)
root.mainloop()