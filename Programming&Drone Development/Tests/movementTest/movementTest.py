import time
import math
from drone import DroneFunctions, VehicleMode

#OpenCV
drone = DroneFunctions()
#Drone
drone.arm_and_takeoff(2)
start_time = time.time()
#Main Loop
while True:
    drone.move()
    print (f"{int(time.time() - start_time)} eta {int(2*math.pi / 0.2)}")
    if int(time.time() - start_time) == int(2*math.pi / 0.2) + 2:
        drone.vehicle.mode = VehicleMode("LAND")
        break
    elif drone.vehicle.mode == VehicleMode("LAND"):
        break
