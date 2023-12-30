from dronekit import connect, VehicleMode
from pymavlink import mavutil
import time
import socket
from simple_pid import PID

class DroneFunctions:
    def __init__(self, testing = False):
        if testing == True:
            #connectionString = socket.gethostbyname_ex(socket.gethostname())[-1][1] + ":14550"
            connectionString = "192.168.124.246:14550"
        else:
            connectionString = "/dev/ttyAMA0"
        print(connectionString)
        self.vehicle = connect(connectionString, baud=921600)

        self.target_diff = 0
        self.target_area = 40

        self.configure_pid(
            #yaw pid
            0.3, 0.0, 0.05,
            #forward pid 
            (2/(self.target_area*2-self.target_area)) * 0.5, 0.0, 0.0
            )

    def arm_and_takeoff(self, alt):
        while not self.vehicle.is_armable:
            print("Waiting to arm")
            time.sleep(1)
        print("Arming Motors")
        self.vehicle.mode = VehicleMode("GUIDED")
        self.vehicle.armed = True
        while not self.vehicle.armed:
            print("Waiting to arm")
            time.sleep(0.1)
        print("Taking off!")
        self.vehicle.simple_takeoff(alt)
        while self.vehicle.location.global_relative_frame.alt < alt - 0.5:
            print("Altitude: ", self.vehicle.location.global_relative_frame.alt)
            time.sleep(0.01)

    def configure_pid(self, kp_y, ki_y, kd_y, kp_f, ki_f, kd_f):
        MAX_HEADING_CMD = 0.25
        MAX_VELOCITY_CMD = 2
        self.pid_yaw = PID(kp_y, ki_y, kd_y, setpoint=self.target_diff)
        self.pid_yaw.output_limits = (-MAX_HEADING_CMD, MAX_HEADING_CMD)

        self.pid_velocity = PID(kp_f, ki_f, kd_f, setpoint=self.target_area)
        self.pid_velocity.output_limits = (-MAX_VELOCITY_CMD, MAX_VELOCITY_CMD)

    def calculate_movement(self, raw_xDiff, area):
        xDiff = -round(self.pid_yaw(raw_xDiff), 2)
        velocity = round(self.pid_velocity(area), 2) if area != 0 else 0
        self.send_movement(xDiff, velocity)
        return xDiff, velocity

    def send_movement(self, xDiff, velocity):
        msg = self.vehicle.message_factory.set_position_target_local_ned_encode(
            0,
            0, 0,
            mavutil.mavlink.MAV_FRAME_BODY_NED,
            0b011111000111,
            0, 0, 0,
            velocity, 0, 0,
            0, 0, 0,
            0, xDiff)
        self.vehicle.send_mavlink(msg)
        #self.vehicle.flush()