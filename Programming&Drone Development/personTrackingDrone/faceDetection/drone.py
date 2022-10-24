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

    def move(self, differences):
        self.xDiff = differences[0] * 360
        self.yDiff = differences[1]
        self.altitudeY()
        
    def yawX(self, heading = 180):
        rotDirect = -1 if self.xDiff < 0 else 1
        heading = 0 if self.xDiff == 0 else 180
        self.xDiff = self.xDiff / -1 if rotDirect == -1 else self.xDiff

        msg = self.vehicle.message_factory.command_long_encode(
            0,0,
            mavutil.mavlink.MAV_CMD_CONDITION_YAW,
            0,
            heading,
            self.xDiff,
            rotDirect,
            1,
            0,0,0)
        self.vehicle.send_mavlink(msg)

        



