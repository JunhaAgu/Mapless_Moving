#include "ros_wrapper.h"

ROSWrapper::ROSWrapper(ros::NodeHandle& nh, bool test_flag) 
: is_initialized(false), nh_(nh), test_flag_(test_flag)
{
    // constructor
    ROS_INFO_STREAM("ROSWrapper - constructed.");

    // get ROS parameter (from '*.launch' file).
    // If parameters are not set, this function throws an exception and terminates the node.
    this->getLaunchParameters();

    // initialization
    solver_ = std::make_unique<MaplessDynamic>(test_flag_);

    // subscriber
    sub_lidar_ = nh_.subscribe<sensor_msgs::PointCloud2>(
            topicname_lidar_, 1, &ROSWrapper::callbackLiDAR, this);
  
    // spin.   
    this->run();
};

ROSWrapper::~ROSWrapper(){
    //destructor
    ROS_INFO_STREAM("ROSWrapper - deleted.");
};

void ROSWrapper::run(){
    int freq_spin = 10; // [Hz]
    ros::Rate rate(freq_spin);
    ROS_INFO_STREAM("ROSWrapper - 'run()' - run at [" << freq_spin << "] Hz.");
    while(ros::ok()){

        solver_->TEST();
        // solver_->solve();
        
        ros::spinOnce();
        rate.sleep();
    }
    ROS_INFO_STREAM("ROSWrapper - 'run()' - run ends.");
};

void ROSWrapper::getLaunchParameters(){
        if(!ros::param::has("~topicname_lidar")) 
            throw std::runtime_error("ROSWrapper - no 'topicname_lidar' is set. You might run the node by 'roslaunch' with parameter settings.\n");
        ros::param::get("~topicname_lidar", topicname_lidar_);
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
        // solver_->solve(p0, p1, T01, mask1);
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