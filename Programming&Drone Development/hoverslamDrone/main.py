from drone import Drone, VehicleMode
import time

drone = Drone()

drone.arm_and_takeoff(50)
while drone.vehicle.armed == True:
    (drone.send_acceleration() if drone.vehicle.location.global_relative_frame.alt <= 45 
                                            else drone.send_acceleration(freefall=True))
    if drone.vehicle.location.global_relative_frame.alt <= 1:
        break
    
drone.vehicle.mode = VehicleMode("LAND")