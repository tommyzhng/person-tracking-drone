import time
from drone import DroneFunctions, VehicleMode

#OpenCV
drone = DroneFunctions()

#Drone
drone.arm_and_takeoff(2)
land_heading = (drone.vehicle.heading + 180) % 360
#Main Loop
while True:
    print(f" {drone.vehicle.heading} and {land_heading}")
    drone.move()
    if drone.vehicle.heading >= land_heading and drone.vehicle.heading <= land_heading + 10:
        drone.vehicle.mode = VehicleMode("LAND")
        break
