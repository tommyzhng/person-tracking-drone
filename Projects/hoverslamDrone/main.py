from drone import Drone
from pymavlink import mavutil
import time

drone = Drone()
drone.set_mode("GUIDED")
drone.arm_throttle()
drone.takeoff(50)

#set data rates
drone.request_message_interval(mavutil.mavlink.MAVLINK_MSG_ID_LOCAL_POSITION_NED, 18)
while True:
    drone.vehicle.altitude = -drone.get_data("LOCAL_POSITION_NED").get('z')
    drone.vehicle.descent_rate = -drone.get_data("LOCAL_POSITION_NED").get('vz')
    drone.send_acceleration() if drone.vehicle.altitude <= 45 else drone.send_acceleration(freefall=True)
    if drone.vehicle.altitude <= 2:
       break
    
drone.set_mode("LAND")