#ifndef _MAPLESS_DYNAMIC_H_
#define _MAPLESS_DYNAMIC_H_

#include <iostream>

#include <Eigen/Dense>

#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>

#include "defines.h"

class MaplessDynamic{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW;

public:
    // DECLARE PUBLIC VARIABLES 
    // * NOT RECOMMANDED TO MAKE PUBLIC VARIABLES
    
private:
    // DECLARE PRIVATE VARIABLES

public:
    MaplessDynamic(); // constructor
    ~MaplessDynamic(); // destructor

    void TEST(); // algorithm test function with a loaded data

    void solve(        
        /* inputs */ 
        const sensor_msgs::PointCloud2& p0, const sensor_msgs::PointCloud2& p1, const Pose& T01, 
        /* outputs */
        Mask& mask);

// From this, declare your own sub-functions. 
private:
    void getUserSettingParameters();
    // void func1();
    // ...
};

#endif