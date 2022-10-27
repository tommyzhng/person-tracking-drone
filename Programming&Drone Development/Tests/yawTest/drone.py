from dronekit import connect, VehicleMode
from pymavlink import mavutil

import socket
import time

class DroneFunctions:
    def __init__(self):
        connectionString = socket.gethostbyname_ex(socket.gethostname())[-1][1] + ":14550"
        print(connectionString)
        self.vehicle = connect(connectionString)

    def arm_and_takeoff(self, alt):
        while not self.vehicle.is_armable:
            print("Waiting to arm")
            time.sleep(1)
        print("Arming Motors")
        self.vehicle.mode = VehicleMode("GUIDED")
        self.vehicle.armed = True
        while not self.vehicle.armed:
            print("Waiting to arm")
            time.sleep(1)
        print("Taking off!")
        self.vehicle.simple_takeoff(alt)
        while self.vehicle.location.global_relative_frame.alt < alt - 0.5:
            time.sleep(1)
        
    def yawX(self, degree, relative = 1, heading = 180):
        rotDirect = -1 if degree < 0 else 1
        if heading == 180:
            heading = 0 if degree == 0 else 180
        degree = degree / -1 if rotDirect == -1 else degree

        msg = self.vehicle.message_factory.command_long_encode(
            0,0,
            mavutil.mavlink.MAV_CMD_CONDITION_YAW,
            0,
            heading,
            degree,
            rotDirect,
            relative,
            0,0,0)
        self.vehicle.send_mavlink(msg)


        



