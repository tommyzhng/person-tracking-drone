import rospy
from geometry_msgs.msg import PointStamped
from mavros_msgs.msg import PositionTarget

class DroneStack():
    def __init__(self):
        # init some publishers and subscribers
        # subscribe to x and y data
        self.error_sub = rospy.Subscriber('tracking/error', PointStamped, self.error_callback)
        self.position_pub = rospy.Publisher('mavros/setpoint_raw/local', PositionTarget, queue_size=1)
        self.forward_vel = 0
        self.yaw_rate = 0

    def error_callback(self, msg):
        # do something with the error data
        pass
    
    def send_position_target(self, dx, dyaw):
        msg = PositionTarget()
        msg.header.stamp = rospy.Time.now()
        msg.coordinate_frame = PositionTarget.FRAME_BODY_NED
        msg.type_mask = PositionTarget.IGNORE_PX |PositionTarget.IGNORE_PY | PositionTarget.IGNORE_PZ | PositionTarget.IGNORE_AFX | PositionTarget.IGNORE_AFY | PositionTarget.IGNORE_AFZ | PositionTarget.IGNORE_YAW
        msg.velocity.x = dx 
        msg.yaw_rate = dyaw
        self.position_pub.publish(msg)
    
    def update_loop(self):
        self.send_position_target(self.forward_vel, self.yaw_rate)
    

