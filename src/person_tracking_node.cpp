#include "person_tracking.hpp"

ros::NodeHandle nh;
PersonTracking personTracking(nh);

while(ros::ok())
{
    personTracking.Process();
    ros::spinOnce();
}
