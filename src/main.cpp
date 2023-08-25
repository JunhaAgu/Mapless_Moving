#include <ros/ros.h>

#include <exception>
#include <iostream>

#include "ros_wrapper.h"

int main(int argc, char** argv) {
    ros::init(argc, argv, "mapless_moving_node");
    ros::NodeHandle nh;

    ROS_INFO_STREAM("Turn on: \"mapless_moving\".\n");

    try {
        std::unique_ptr<ROSWrapper> wrapper;
        wrapper = std::make_unique<ROSWrapper>(nh);
    } catch (std::exception& e) {
        std::cout << " ERROR! the error message is [" << e.what() << std::endl;
    }

    ROS_INFO_STREAM("Turn off: \"mapless_moving\".\n");

    return 0;
}
