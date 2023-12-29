from threading import Thread
from dronekit import VehicleMode
from sshkeyboard import listen_keyboard

class KeyboardListener():
    def __init__(self, drone) -> None:
        self.drone = drone
        self.start_tracking = False

    def on_press(self, key):
        global start_tracking
        if key == 's':
            self.start_tracking = True
        if key == 't':
            self.drone.arm_and_takeoff(3)
        if key == 'l':
            self.drone.vehicle.mode = VehicleMode("LAND")

    def listen(self):
        listen_keyboard(on_press=self.on_press)

    def start(self):
        Thread(target=self.listen, args=()).start()