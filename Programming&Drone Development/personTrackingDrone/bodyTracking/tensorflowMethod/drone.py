from dronekit import connect, VehicleMode
from pymavlink import mavutil
import time

class DroneFunctions:
    def __init__(self):
        connectionString = "192.168.112.246:14550"
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

    def move(self, xDiff, area):
        velocity = self.fwd_movement(area)
        xDiff = xDiff if xDiff > 0.1 or xDiff < -0.1 else 0
        msg = self.vehicle.message_factory.set_position_target_local_ned_encode(
            0,
            0, 0,
            mavutil.mavlink.MAV_FRAME_BODY_NED,
            0b011111000111,
            0, 0, 0,
            velocity, 0, 0,
            0, 0, 0,
            0, (xDiff/1.5))
        self.vehicle.send_mavlink(msg)
    
    def fwd_movement(self, area):
            if area < 40 and area > 0:
                return 0.2
            elif area > 50:
                return -0.2
            else:
                return 0