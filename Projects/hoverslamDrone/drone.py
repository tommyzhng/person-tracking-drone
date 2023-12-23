from pymavlink import mavutil
import socket
import time

class Drone:       
    def __init__(self):
        connectionString = socket.gethostbyname_ex(socket.gethostname())[-1][1] + ":14550"
        print(connectionString)
        self.vehicle:mavutil = mavutil.mavlink_connection(connectionString)
        self.vehicle.wait_heartbeat()

    def get_data(self, command):
        return self.vehicle.recv_match(type=command, blocking=True).to_dict()
    
    def request_message_interval(self, message_id: int, frequency_hz: float):
        self.vehicle.mav.command_long_send(
            self.vehicle.target_system, self.vehicle.target_component,
            mavutil.mavlink.MAV_CMD_SET_MESSAGE_INTERVAL, 0,
            message_id, # The MAVLink message ID
            1e6 / frequency_hz, # The interval between two messages in microseconds. Set to -1 to disable and 0 to request default rate.
            0, 0, 0, 0, 
            0, # Target address of message stream. 0: Flight-stack , 1: address of requestor, 2: broadcast.
        )
    
    def set_mode(self, mode:str):
        mode_id = self.vehicle.mode_mapping()[mode]
        self.vehicle.set_mode(mode_id)

    def arm_throttle(self):
        self.vehicle.mav.command_long_send(
            self.vehicle.target_system,
            self.vehicle.target_component,
            mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
            0,
            1, 
            0, 0, 0, 0, 0, 0)
        print("Waiting for the vehicle to arm")
        self.vehicle.motors_armed_wait()
        print("Armed")

    def takeoff(self, alt):
        self.vehicle.mav.command_long_send(
            self.vehicle.target_system,
            self.vehicle.target_component,
            mavutil.mavlink.MAV_CMD_NAV_TAKEOFF,
            0, 
            0, 0, 0, 0, 0, 0, 
            alt)
        print("Taking off")

        while -self.get_data("LOCAL_POSITION_NED").get('z') < alt - 0.5:
            pass

        print("Reached target height")
    
    def calculate_target_acceleration(self):
        v1 = self.vehicle.descent_rate
        v2 = 0
        d = self.vehicle.altitude - 0.5

        a = ((v2**2)-(v1**2)) / (2*(d))
        print(f"current v: {v1}, d to ground: {d}, commanded accel: {a}")
        return a

    def send_acceleration(self, freefall = False):
        a = self.calculate_target_acceleration() if not freefall else 10
        msg = mavutil.mavlink.MAVLink_set_position_target_local_ned_message(
            0,
            0, 0,
            mavutil.mavlink.MAV_FRAME_LOCAL_NED,
            0b110000111111,
            0, 0, 0,
            0, 0, 0,
            0, 0, a,
            0, 0)
        self.vehicle.mav.send(msg) 


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