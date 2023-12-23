from dronekit import connect, VehicleMode
from pymavlink import mavutil

import socket
import time

class Drone:       
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

    def calculate_target_acceleration(self):
        v1 = self.vehicle.velocity[2]
        v2 = 0
        d = self.vehicle.location.global_relative_frame.alt - 0.5

        a = ((v2**2)-(v1**2)) / (2*(d))
        #a = (v2 - v1) / 2
        print(f"current v: {v1}, d to ground: {d}, commanded accel: {a}")
        time.sleep(0.2)
        return a

    def send_acceleration(self, freefall = False):
        a = self.calculate_target_acceleration() if not freefall else 10
        msg = self.vehicle.message_factory.set_position_target_local_ned_encode(
            0,
            0, 0,
            mavutil.mavlink.MAV_FRAME_LOCAL_NED,
            0b110000111111,
            0, 0, 0,
            0, 0, 0,
            0, 0, a,
            0, 0)
        self.vehicle.send_mavlink(msg) 


    def yaw_instruction(self):
        if self.vehicle.location.global_relative_frame.alt < 10:
            a = self.calculate_acceleration()
            v = self.vehicle.velocity[2]
        else:
            v = 3.5
            a = 0
        msg = self.vehicle.message_factory.set_position_target_local_ned_encode(
            0,
            0, 0,
            mavutil.mavlink.MAV_FRAME_BODY_NED,
            0b010000000111,
            0, 0, 0, #position
            5, 0, v, #velocity
            0, 0, a, #acceleration
            0, 1.3)    #yaw
        self.vehicle.send_mavlink(msg)
        self.request_data()
        time.sleep(0.2)