#include "ros_wrapper.h"

ROSWrapper::ROSWrapper(ros::NodeHandle& nh) 
: is_initialized_(false), nh_(nh)
{
    // constructor
    ROS_INFO_STREAM("ROSWrapper - constructed.");

    p0_pcl_ = boost::make_shared<pcl::PointCloud<pcl::PointXYZ>>();
    p1_pcl_ = boost::make_shared<pcl::PointCloud<pcl::PointXYZ>>();

    // get ROS parameter (from '*.launch' file).
    // If parameters are not set, this function throws an exception and terminates the node.
    this->getLaunchParameters();

    // initialization
    solver_ = std::make_unique<MaplessDynamic>(nh_, rosbag_play_, data_number_);

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
    int freq_spin = 100; // [Hz]
    ros::Rate rate(freq_spin);
    ROS_INFO_STREAM("ROSWrapper - 'run()' - run at [" << freq_spin << "] Hz.");
    while(ros::ok()){
        
        if (rosbag_play_ == false)
        {
            ROS_INFO_STREAM("Data is from saved pcd");
            solver_->TEST();
        }
        else
        {}
        
        ros::spinOnce();
        rate.sleep();
    }
    ROS_INFO_STREAM("ROSWrapper - 'run()' - run ends.");
};

void ROSWrapper::getLaunchParameters(){
        if(!ros::param::has("~topicname_lidar")) 
            throw std::runtime_error("ROSWrapper - no 'topicname_lidar' is set. You might run the node by 'roslaunch' with parameter settings.\n");
        ros::param::get("~topicname_lidar", topicname_lidar_);

        ros::param::get("~rosbag_play", rosbag_play_);
        ros::param::get("~T01_slam", T01_slam_);
        ros::param::get("~data_number", data_number_);
};

void ROSWrapper::callbackLiDAR(const sensor_msgs::PointCloud2ConstPtr& msg){

    static int cnt_data = 0;
    std::cout<< "Iter: "<< cnt_data << std::endl;

    if( is_initialized_ ){ // If initialized, 
        // 0. Get the current LiDAR data
        sensor_msgs::PointCloud2 p1_msg_ = *msg;
        pcl::fromROSMsg(p1_msg_, *p1_pcl_);

        // 1. Calculate T01 from the SLAM (or Odometry) algorithm
        Pose T01;
        if (T01_slam_==true)
        {
            timer::tic();
            // T01 =  vo->solve();
            double dt_slam = timer::toc(); // milliseconds
            ROS_INFO_STREAM("elapsed time for 'SLAM' :" << dt_slam << " [ms]");
        }
        else
        {
            T01 = solver_->data_buf_[cnt_data+2]->T_gt_.inverse();
        }

        // 2. Solve the Mapless Dynamic algorithm.
        timer::tic();
        Mask mask1;

        ROS_INFO_STREAM("Data is from rosbag");

        solver_->solve(p0_pcl_, p1_pcl_, T01, mask1, cnt_data);
        double dt_solver = timer::toc(); // milliseconds
        ROS_INFO_STREAM("elapsed time for 'solver' :" << dt_solver << " [ms]");

        // // 3. Update the previous variables
        updatePreviousVariables(p0_pcl_, p1_pcl_, mask1);

        cnt_data += 1;
    }
    else { // If not initialized, 
        is_initialized_ = true;
        
        // Initialize the first data.
        p0_msg_ = *msg;
        pcl::fromROSMsg(p0_msg_, *p0_pcl_);
        // mask0.resize(p0.width, true);

        solver_->CloudFrame_cur_ ->genRangeImages(p0_pcl_, true);

        // mask0_test_.resize(p0_msg_test_.width, true);
        cnt_data = 1;
    }
};

void ROSWrapper::updatePreviousVariables(pcl::PointCloud<pcl::PointXYZ>::Ptr p0_pcl, pcl::PointCloud<pcl::PointXYZ>::Ptr p1_pcl, const Mask& mask1){
    // p0_pcl_ = p1_pcl_;

    p0_pcl->resize(0);
    pcl::copyPointCloud(*p1_pcl, *p0_pcl);
    p1_pcl->resize(0);

    mask0.resize(mask1.size());
    std::copy(mask1.begin(), mask1.end(), mask0.begin());
};