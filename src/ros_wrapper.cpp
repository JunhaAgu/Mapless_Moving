#include "ros_wrapper.h"

ROSWrapper::ROSWrapper()
: is_initialized(false)
{
    // constructor
    ROS_INFO_STREAM("ROSWrapper - constructed.");

    // ROS parameter (launch ).. 받아옴

    // initialization
    solver = std::make_unique<MaplessDynamic>();

    // subscriber
    // sub.subscribe<PointCLoud>("/velodyne/lidar0", ROSWrapper::callbackLiDAR(), this);

    //run!!!    
    this->run();
};

ROSWrapper::~ROSWrapper(){
    //destructor
    ROS_INFO_STREAM("ROSWrapper - deleted.");
};

void ROSWrapper::run(){
    ros::Rate rate(1000);
    ROS_INFO_STREAM("ROSWrapper - 'run()' - run at ["<< 1000 << "] Hz.");
    while(ros::ok()){
        ros::spinOnce();
        rate.sleep();
    }
};