#ifndef _ROS_WRAPPER_H_
#define _ROS_WRAPPER_H_

// std headers
#include <iostream>
#include <vector>

// ROS related headers
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>

// User-defined types & timer headers
#include "defines.h"
#include "timer.h"

// Mapless Dynamic header
#include "mapless_dynamic.h"

#include <visualization_msgs/Marker.h>

class ROSWrapper{
/* =========
    MEMBERS
   ========= */
// Previous variables
private:
    Mask                     mask0;
    sensor_msgs::PointCloud2 p0_msg_;
    sensor_msgs::PointCloud2 p1_msg_;
    bool                     is_initialized_; // = default : false.
    pcl::PointCloud<pcl::PointXYZI>::Ptr p0_pcl_;
    pcl::PointCloud<pcl::PointXYZI>::Ptr p1_pcl_;

// ROS nodehandle & subscriber for LiDAR data.
private:
    ros::NodeHandle nh_;
    ros::Subscriber sub_lidar_;
    ros::Publisher marker_pub_;
    ros::Publisher lidar_marker_pub_;
    visualization_msgs::Marker marker_;
    visualization_msgs::Marker lidar_marker_;
    
    std::string     topicname_lidar_;
    bool            rosbag_play_;
    bool            T01_slam_;
    std::string     dataset_name_;
    std::string     data_number_;

    std_msgs::Header cloudHeader_;

// Mapless Dynamic algorithm object. 
private:
    std::unique_ptr<MaplessDynamic> solver_;

/* =========
    METHODS
   ========= */
public:
    ROSWrapper(ros::NodeHandle& nh); // constructor
    ~ROSWrapper(); // destructor

private:
    void run(); 
    
    void getLaunchParameters();
    void callbackLiDAR(const sensor_msgs::PointCloud2ConstPtr& msg);    
    void updatePreviousVariables(pcl::PointCloud<pcl::PointXYZI>::Ptr p0_pcl, pcl::PointCloud<pcl::PointXYZI>::Ptr p1_pcl, const Mask& mask1);
};

#endif