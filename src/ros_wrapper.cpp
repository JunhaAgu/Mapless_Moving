#include "ros_wrapper.h"

ROSWrapper::ROSWrapper(ros::NodeHandle& nh) 
: is_initialized_(false), nh_(nh)
{
    // constructor
    ROS_INFO_STREAM("ROSWrapper - constructed.");

    p0_pcl_ = boost::make_shared<pcl::PointCloud<pcl::PointXYZI>>();
    p1_pcl_ = boost::make_shared<pcl::PointCloud<pcl::PointXYZI>>();

    // get ROS parameter (from '*.launch' file).
    // If parameters are not set, this function throws an exception and terminates the node.
    this->getLaunchParameters();

    // initialization
    solver_ = std::make_unique<MaplessDynamic>(nh_, rosbag_play_, data_number_);

    // subscriber
    sub_lidar_ = nh_.subscribe<sensor_msgs::PointCloud2>(
            topicname_lidar_, 1, &ROSWrapper::callbackLiDAR, this);

    // publish
    marker_pub_ = nh_.advertise<visualization_msgs::Marker>("marker/node", 1);
    lidar_marker_pub_ = nh_.advertise<visualization_msgs::Marker>("lidar_marker/node", 1);
    
    //marker setting
    {
        marker_.header.frame_id = "/map"; // map frame
        marker_.color.a = 1.0; // Don't forget to set the alpha!
        marker_.color.r = 0.0;
        marker_.color.g = 1.0;
        marker_.color.b = 0.0;
        marker_.scale.x = 0.1;
        marker_.scale.y = 0.1;
        marker_.scale.z = 3.0;
        marker_.type = visualization_msgs::Marker::TEXT_VIEW_FACING; //TEXT_VIEW_FACING; SPHERE;
        marker_.id = 0;
        marker_.action = visualization_msgs::Marker::ADD;
        marker_.pose.orientation.x = 0.0;
        marker_.pose.orientation.y = 0.0;
        marker_.pose.orientation.z = 0.0;
        marker_.pose.orientation.w = 1.0;
    }
    //lidar marker setting
    {
        lidar_marker_.header.frame_id = "/map"; // map frame
        lidar_marker_.color.a = 1.0; // Don't forget to set the alpha!
        lidar_marker_.color.r = 0.0;
        lidar_marker_.color.g = 1.0;
        lidar_marker_.color.b = 0.0;
        lidar_marker_.scale.x = 1.0;
        lidar_marker_.scale.y = 1.0;
        lidar_marker_.scale.z = 1.0;
        lidar_marker_.type = visualization_msgs::Marker::CYLINDER; //TEXT_VIEW_FACING; SPHERE;
        lidar_marker_.id = 0;
        lidar_marker_.action = visualization_msgs::Marker::ADD;
        lidar_marker_.pose.orientation.x = 0.0;
        lidar_marker_.pose.orientation.y = 0.0;
        lidar_marker_.pose.orientation.z = 0.0;
        lidar_marker_.pose.orientation.w = 1.0;
    }
  
    // spin.   
    this->run();
};

ROSWrapper::~ROSWrapper(){
    //destructor
    ROS_INFO_STREAM("ROSWrapper - deleted.");
};

void ROSWrapper::run(){
    int freq_spin = 100; // test: 10[Hz], rosbag: <100[Hz]
    ros::Rate rate(freq_spin);
    ROS_INFO_STREAM("ROSWrapper - 'run()' - run at [" << freq_spin << "] Hz.");
    ROS_INFO_STREAM("Rosbag can be started");
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

    cloudHeader_ = msg->header;
    if( is_initialized_ ){ // If initialized, 
        // 0. Get the current LiDAR data
        // p1_msg_ = *msg;
        pcl::fromROSMsg(*msg, *p1_pcl_);

        // 1. Calculate T01 from the SLAM (or Odometry) algorithm
        Pose T01;
        Pose T10;
        if (T01_slam_==true)
        {
            timer::tic();
            // T01 =  vo->solve();
            double dt_slam = timer::toc(); // milliseconds
            ROS_INFO_STREAM("elapsed time for 'SLAM' :" << dt_slam << " [ms]");
        }
        else
        {
            T01 = solver_->data_buf_[cnt_data+1]->T_gt_;
            T10 = solver_->data_buf_[cnt_data+1]->T_gt_.inverse();
            //"00": +3
            //"01": +2
        }

        // 2. Solve the Mapless Dynamic algorithm.
        timer::tic();
        Mask mask1;

        ROS_INFO_STREAM("Data is from rosbag");
        ROS_INFO_STREAM("p0_ size:" << p0_pcl_->size() << " " << "p1_ size:" << p1_pcl_->size() << " ");
        solver_->solve(p0_pcl_, p1_pcl_, T10, mask1, cnt_data, cloudHeader_);
        double dt_solver = timer::toc(); // milliseconds
        ROS_INFO_STREAM("elapsed time for 'solver' :" << dt_solver << " [ms]");

        // // 3. Update the previous variables
        updatePreviousVariables(p0_pcl_, p1_pcl_, mask1);
        
        marker_.header.stamp = ros::Time::now();
        marker_.text = std::to_string(cnt_data);
        marker_.pose.position.x = 0; //T01(0,3);     //x
        marker_.pose.position.y = 0; //T01(1,3);     //y
        marker_.pose.position.z = 10; //T01(2,3) + 10; //z

        lidar_marker_.header.stamp = ros::Time::now();
        lidar_marker_.pose.position.x = 0; //T01(0,3);     //x
        lidar_marker_.pose.position.y = 0; //T01(1,3);     //y
        lidar_marker_.pose.position.z = 0; //T01(2,3); //z

        marker_pub_.publish(marker_);
        lidar_marker_pub_.publish(lidar_marker_);

        cnt_data += 1;
    }
    else { // If not initialized, 
        is_initialized_ = true;
        
        // Initialize the first data.
        p0_msg_ = *msg;
        pcl::fromROSMsg(p0_msg_, *p0_pcl_);
        // mask0.resize(p0.width, true);

        solver_->CloudFrame_cur_ ->genRangeImages(p0_pcl_, true);
        ROS_INFO_STREAM("p0_ size:" << p0_pcl_->size());
        // mask0_test_.resize(p0_msg_test_.width, true);
        cnt_data = 1;
    }
};

void ROSWrapper::updatePreviousVariables(pcl::PointCloud<pcl::PointXYZI>::Ptr p0_pcl, pcl::PointCloud<pcl::PointXYZI>::Ptr p1_pcl, const Mask& mask1){
    // p0_pcl_ = p1_pcl_;

    p0_pcl->resize(0);
    pcl::copyPointCloud(*p1_pcl, *p0_pcl);
    p1_pcl->resize(0);

    mask0.resize(mask1.size());
    std::copy(mask1.begin(), mask1.end(), mask0.begin());
};