#include <iostream>
#include <exception>

#include <ros/ros.h>

#include "ros_wrapper.h"

int main(int argc, char **argv)
{
    ros::init(argc, argv, "mapless_dynamic_node");
    ros::NodeHandle nh;
    bool TEST = true;

    ROS_INFO_STREAM("Turn on: \"mapless_dynamic\".\n");

    try{
        std::unique_ptr<ROSWrapper> wrapper;
        wrapper = std::make_unique<ROSWrapper>(nh, TEST);
    }
    catch(std::exception& e){
        std::cout << " ERROR! the error message is [" << e.what() << std::endl;
    }

    ROS_INFO_STREAM("Turn off: \"mapless_dynamic\".\n");

    return 0;
}
