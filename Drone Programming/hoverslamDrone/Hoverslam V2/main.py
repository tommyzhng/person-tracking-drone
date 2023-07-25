from drone import Drone, VehicleMode
import time

drone = Drone()

target_alt = 50

drone.arm_and_takeoff(target_alt)
while drone.vehicle.armed == True:
    (drone.send_instruction())
    print(drone.vehicle.velocity[2])
    if drone.vehicle.location.global_relative_frame.alt <= 1.75:
        break
    
drone.vehicle.mode = VehicleMode("LAND")