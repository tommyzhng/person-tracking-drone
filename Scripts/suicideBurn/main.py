from drone import Drone, VehicleMode
import time

drone = Drone()

drone.arm_and_takeoff(50)
#drone.vehicle.mode = VehicleMode("STABILIZE")
while drone.vehicle.armed == True:
    if drone.vehicle.location.global_relative_frame.alt >= 45:
        drone.send_acceleration(freefall=True)
    else:
        drone.send_acceleration()
        if drone.vehicle.location.global_relative_frame.alt <= 1:
            break
drone.vehicle.mode = VehicleMode("LAND")
        