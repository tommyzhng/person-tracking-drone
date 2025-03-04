import rospy
from drone_stack import DroneStack

class States(): # enum states
    # sequential states
    IDLE = "IDLE"
    TAKEOFF = "TAKEOFF"
    TRACKING = "TRACKING"
    LANDING = "LANDING"

    # additional controllable states
    ENU_FRAME_CONTROL = "ENU_FRAME_CONTROL"
    BODY_FRAME_CONTROL = "BODY_FRAME_CONTROL"

    def __init__(self):
        pass


class DroneStateMachine():
    def __init__(self, drone_stack: DroneStack):
        self.state = States.IDLE
        self.drone = drone_stack
        
    def switch_state(self, new_state):
        self.state = new_state
        rospy.loginfo("Switching state to: " + new_state)

    def run_states(self):
        if self.state == States.IDLE:
            self.idle()
            #self.switch_state(States.TAKEOFF)
        elif self.state == States.TAKEOFF:
            self.takeoff()
        elif self.state == States.TRACKING:
            #self.tracking()
            pass
        elif self.state == States.LANDING:
            self.landing()
        elif self.state == States.ENU_FRAME_CONTROL:
            self.enu_frame_control()
        elif self.state == States.BODY_FRAME_CONTROL:
            self.body_frame_control()
        else:
            rospy.logerr("Invalid state: " + self.state)
            self.switch_state(States.IDLE)
    
    def idle(self):
        # do nothing
        pass

    def takeoff(self):
        # send takeoff target
        self.drone.send_position_target(0, 0, self.drone.hover_height)
        if not self.drone.drone_state.armed:
            self.drone.arm_drone()
            if self.drone.drone_state.mode != "OFFBOARD":
                while not self.drone.switch_offboard():
                    self._rate.sleep()
                    continue

        # check if takeoff is complete
        if (abs(self.drone.local_position[2] - self.drone.hover_height)< 0.2):
            self.switch_state(States.ENU_FRAME_CONTROL)
        pass

    def landing(self):
        # send landing target
        self.drone.switch_flight_mode("LAND")
        self.switch_state(States.IDLE)

    def enu_frame_control(self):
        # send position target
        self.drone.send_position_target(self.drone.position_command[0], self.drone.position_command[1], self.drone.position_command[2])
        pass

    def body_frame_control(self):
        # send body position target
        self.drone.send_body_position_target(self.drone.position_command[0], self.drone.position_command[1], self.drone.position_command[2])
        pass

    