import rospy
from geometry_msgs.msg import PointStamped
from mavros_msgs.msg import PositionTarget, State
from mavros_msgs.srv import CommandBool, SetMode, SetModeRequest, CommandTOL
from nav_msgs.msg import Odometry
import math
import numpy as np
import asyncio


class DroneStack():
    def __init__(self, rate):
        # init some publishers and subscribers

        # subscribe to x and y data
        self.error_sub = rospy.Subscriber('tracking/error', PointStamped, self.error_callback)
        self.state_sub = rospy.Subscriber('mavros/state', State, self.state_callback)
        self.position_sub = rospy.Subscriber('mavros/global_position/local', Odometry, self.position_callback)
        self.position_pub = rospy.Publisher('mavros/setpoint_raw/local', PositionTarget, queue_size=1)
        
        # ROS services
        self._arming_client = rospy.ServiceProxy("mavros/cmd/arming", CommandBool)
        self._set_mode_client = rospy.ServiceProxy("mavros/set_mode", SetMode)
        self._landing_client = rospy.ServiceProxy("mavros/cmd/land", CommandBool)
        self.rate = rospy.Rate(rate)

        # drone info
        self.local_enu = [0,0,0]
        self.hover_height = 2.0
        self.drone_state = State()

        # commands
        self.position_command = [0,0, self.hover_height]
        self._offb_set_mode = SetModeRequest()
        self._offb_set_mode.custom_mode = 'OFFBOARD'
        self.set_mode = SetModeRequest()
        self._last_req = rospy.Time.now()

        # tracking
        self.vel = [0,0,0]
        self.yaw_rate = 0
        self.max_yaw_rate = 0.5 # rad/s
        self.T_Kp = 0.1
        
    def position_callback(self, msg):
        self.local_position = [msg.pose.pose.position.x, msg.pose.pose.position.y, msg.pose.pose.position.z]
        pass

    def state_callback(self, msg):
        self.drone_state = msg


    def error_callback(self, msg):
        # do something with the error data
        self.vel[0] = msg.point.z # forward vel
        self.yaw_rate = np.clip(self.T_Kp * msg.point.x, -self.max_yaw_rate, self.max_yaw_rate) # yaw rate proportional to error
        pass
    
    def arm_drone(self):
        # send arm command to drone
        if not self.drone_state.armed:
            rospy.loginfo("Waiting for manual arm")
            while not self.drone_state.armed:
                self.rate.sleep()
            rospy.loginfo("Drone armed")
            return True
        else:
            rospy.loginfo("Drone already armed")
            return True
            
        

    def switch_flight_mode(self, mode):
        # switch flight mode
        if self.drone_state.mode != mode:
            rospy.loginfo("Switching to " + mode)
            self.set_mode.custom_mode = mode
            self._set_mode_client.call(self.set_mode)
        else:
            rospy.loginfo("Already in " + mode)

    def switch_offboard(self):
        while self.drone_state.mode != "OFFBOARD":
            if (self.drone_state.mode != "OFFBOARD" and (rospy.Time.now() - self._last_req) > rospy.Duration(5.0)):
                self._set_mode_client.call(self._offb_set_mode)
                self._last_req = rospy.Time.now()
        rospy.loginfo("OFFBOARD enabled")
            


    def send_position_target(self, x, y, z):
        msg = PositionTarget()
        msg.header.stamp = rospy.Time.now()
        msg.coordinate_frame = PositionTarget.FRAME_LOCAL_NED
        msg.type_mask = PositionTarget.IGNORE_VX | PositionTarget.IGNORE_VY | PositionTarget.IGNORE_VZ | PositionTarget.IGNORE_AFX | PositionTarget.IGNORE_AFY | PositionTarget.IGNORE_AFZ | PositionTarget.IGNORE_YAW | PositionTarget.IGNORE_YAW_RATE
        msg.position.x = x
        msg.position.y = y
        msg.position.z = z
        self.position_pub.publish(msg)

    def send_body_position_target(self, x, y, z):
        msg = PositionTarget()
        msg.header.stamp = rospy.Time.now()
        msg.coordinate_frame = PositionTarget.FRAME_BODY_NED
        msg.type_mask = PositionTarget.IGNORE_AFX | PositionTarget.IGNORE_AFY | PositionTarget.IGNORE_AFZ | PositionTarget.IGNORE_YAW | PositionTarget.IGNORE_YAW_RATE
        msg.position.x = x
        msg.position.y = y
        msg.position.z = z
        self.position_pub.publish(msg)

    def send_velocity_target(self, dx, dy, dz, dyaw):
        msg = PositionTarget()
        msg.header.stamp = rospy.Time.now()
        msg.coordinate_frame = PositionTarget.FRAME_BODY_NED
        msg.type_mask = PositionTarget.IGNORE_PX |PositionTarget.IGNORE_PY | PositionTarget.IGNORE_PZ | PositionTarget.IGNORE_AFX | PositionTarget.IGNORE_AFY | PositionTarget.IGNORE_AFZ | PositionTarget.IGNORE_YAW
        msg.velocity.x = dx 
        msg.velocity.y = dy
        msg.velocity.z = dz
        msg.yaw_rate = dyaw
        self.position_pub.publish(msg)
    

    

