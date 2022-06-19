#ifndef _DEFINES_H_
#define _DEFINES_H_

#include <iostream>
#include <ros/ros.h>
#include <vector>
#include <Eigen/Dense>

// ROS cv_bridge
#include <cv_bridge/cv_bridge.h>
#include <opencv2/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>

typedef std::vector<bool> Mask;
typedef Eigen::Matrix4d   Pose; 

#endif