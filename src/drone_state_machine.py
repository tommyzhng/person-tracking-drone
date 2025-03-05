import rospy
from drone_stack import DroneStack
from enum import Enum

class States(Enum): # enum states
    # sequential states
    IDLE = "IDLE"
    TAKEOFF = "TAKEOFF"
    TRACKING = "TRACKING"
    LANDING = "LANDING"

    # additional controllable states
    ENU_FRAME_CONTROL = "ENU_FRAME_CONTROL"
    BODY_FRAME_CONTROL = "BODY_FRAME_CONTROL"


class DroneStateMachine():
    def __init__(self, drone_stack: DroneStack):
        self.state = States.IDLE
        self.drone = drone_stack
        
    def switch_state(self, new_state):
        self.state = new_state
        rospy.loginfo("Switching state to: " + new_state.name)

    def run_states(self):
        if self.state == States.IDLE:
            self.idle()
        elif self.state == States.TAKEOFF:
            self.takeoff()
            self.switch_state(States.ENU_FRAME_CONTROL)
        elif self.state == States.TRACKING:
            #self.tracking()
            pass
        elif self.state == States.LANDING:
            self.landing()
        elif self.state == States.ENU_FRAME_CONTROL:
            self.enu_frame_control()
        elif self.state == States.BODY_FRAME_CONTROL:
            #self.body_frame_control()
            pass
        else:
            rospy.logerr("Invalid state: " + self.state.name)
            self.switch_state(States.IDLE)
    
    def idle(self):
        # do nothing
        pass

    def takeoff(self):
        self.drone.send_position_target(0, 0, self.drone.hover_height)
        # send takeoff target
        if not self.drone.drone_state.armed:
            self.drone.arm_drone()
        if not self.drone.drone_state.mode == "OFFBOARD":
            self.drone.send_position_target(0, 0, self.drone.hover_height)
            self.drone.switch_offboard()

        # check if takeoff is complete
        if (abs(self.drone.local_position[2] - self.drone.hover_height)< 0.2):
            #self.switch_state(States.ENU_FRAME_CONTROL)
            pass

    def landing(self):
        # send landing target
        self.drone.switch_flight_mode("AUTO.LAND")
        self.switch_state(States.IDLE)
        # exit program
        rospy.signal_shutdown("Landed")

    def enu_frame_control(self):
        # send position target
        self.drone.send_position_target(self.drone.position_command[0], self.drone.position_command[1], self.drone.position_command[2])

    def body_frame_control(self):
        # send body position target
        self.drone.send_body_position_target(self.drone.position_command[0], self.drone.position_command[1], self.drone.position_command[2])
        pass

    