#include <ros/ros.h>
#include <mavros_msgs/PositionTarget.h>
#include <opencv2/opencv.hpp>
#include <eigen3/Eigen/Dense>

class DroneStack
{
public:
    DroneStack(ros::NodeHandle nh);
    ~DroneStack();

private:
    void PIDController();
    Eigen::Vector2d targetCoordinate{0,0};
    Eigen::Vector2d centerCoordinate{0,0};

    // ros
    void sendPositionTarget();
    ros::NodeHandle nh_;
    ros::Publisher positionTargetPub_;
    mavros_msgs::PositionTarget positionTargetMsg_;
};

DroneStack::DroneStack(/* args */)
{
}

DroneStack::~DroneStack()
{
}
