#include "ros_wrapper.h"

ROSWrapper::ROSWrapper(ros::NodeHandle& nh) 
: is_initialized(false), nh_(nh)
{
    // constructor
    ROS_INFO_STREAM("ROSWrapper - constructed.");

    // get ROS parameter (launch )
    this->getLaunchParameters();

    // initialization
    solver_ = std::make_unique<MaplessDynamic>();

    // subscriber
    sub_lidar_ = nh_.subscribe<sensor_msgs::PointCloud2>(
            "/lidar0/velodyne_points", 1, &ROSWrapper::callbackLiDAR, this);
  
    // spin.   
    this->run();
};

ROSWrapper::~ROSWrapper(){
    //destructor
    ROS_INFO_STREAM("ROSWrapper - deleted.");
};

void ROSWrapper::run(){
    int freq_spin = 200; // [Hz]
    ros::Rate rate(freq_spin);
    ROS_INFO_STREAM("ROSWrapper - 'run()' - run at [" << freq_spin << "] Hz.");
    while(ros::ok()){
        ros::spinOnce();
        rate.sleep();
    }
    ROS_INFO_STREAM("ROSWrapper - 'run()' - run ends.");
};

void ROSWrapper::getLaunchParameters(){

};

void ROSWrapper::callbackLiDAR(const sensor_msgs::PointCloud2ConstPtr& msg){
    if( is_initialized ){ // If initialized, 
        // 0. Get the current LiDAR data
        sensor_msgs::PointCloud2 p1 = *msg;

        // 1. Calculate T01 from the SLAM (or Odometry) algorithm
        Pose T01;
        timer::tic();
        // T01 =  vo->solve();
        double dt_slam = timer::toc(); // milliseconds
        ROS_INFO_STREAM("elapsed time for 'SLAM' :" << dt_slam << " [ms]");

        // 2. Solve the Mapless Dynamic algorithm.
        timer::tic();
        Mask mask1;
        solver_->solve(p0, p1, T01, mask1);
        double dt_solver = timer::toc(); // milliseconds
        ROS_INFO_STREAM("elapsed time for 'solver' :" << dt_solver << " [ms]");

        // 3. Update the previous variables
        updatePreviousVariables(p1, mask1);
    }
    else { // If not initialized, 
        is_initialized = true;
        
        // Initialize the first data.
        p0 = *msg;
        mask0.resize(p0.width, true);
    }
};

void ROSWrapper::updatePreviousVariables(const sensor_msgs::PointCloud2& p1, const Mask& mask1){
    p0 = p1;

    mask0.resize(mask1.size());
    std::copy(mask1.begin(), mask1.end(), mask0.begin());
};