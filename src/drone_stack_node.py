import drone_stack
import rospy

def main():
    rospy.init_node('drone_stack_node')
    rate = rospy.Rate(60)
    flight_stack = drone_stack.DroneStack()

    while not rospy.is_shutdown():
        flight_stack.update_loop()
        rate.sleep()
    