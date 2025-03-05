#include "person_tracking.hpp"

int main(int argc, char** argv)
{
    ros::init(argc, argv, "person_tracking_node");
    ros::NodeHandle nh;
    PersonTracking personTracking(nh);
    while (ros::ok())
    {
        personTracking.Update();
    }
    return 0;
}
