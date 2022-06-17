#include <ros/ros.h>

int main(int argc, char **argv)
{
    ros::init(argc, argv, "main_mapless_dynamic");
    ros::NodeHandle nh;
    ROS_INFO_STREAM("Turn on: \"maless_dynamic\".\n");

    

    ros::spin();

    return 0;
}
