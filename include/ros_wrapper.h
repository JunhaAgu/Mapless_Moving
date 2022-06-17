#ifndef _ROS_WRAPPER_H_
#define _ROS_WRAPPER_H_

#include <iostream>
#include <vector>
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>

#include "defines.h"

#include "mapless_dynamic.h"

class ROSWrapper{
// Previous variables
private:
    Mask                     mask0;
    sensor_msgs::PointCloud2 p0;
    bool                     is_initialized; // = default : false.

private:
    std::unique_ptr<MaplessDynamic> solver;

public:
    ROSWrapper(); // constructor
    ~ROSWrapper(); // destructor

private:
    void run(); 

    // void callbackLiDAR(PointCloud p_now);    
};

#endif