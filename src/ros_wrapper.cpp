#include "ros_wrapper.h"

ROSWrapper::ROSWrapper(ros::NodeHandle& nh) 
: is_initialized_(false), nh_(nh)
{
    // constructor
    ROS_INFO_STREAM("ROSWrapper - constructed.");

    p0_pcl_ = boost::make_shared<pcl::PointCloud<pcl::PointXYZI>>();
    p1_pcl_ = boost::make_shared<pcl::PointCloud<pcl::PointXYZI>>();

    // p1_pcl_wtime_ = boost::make_shared<pcl::PointCloud<slam::XYZTPoint>>();
    // std::shared_ptr<pcl::PointCloud<slam::XYZTPoint>> p1_pcl_wtime_ = nullptr;
    p1_pcl_wtime_ = boost::make_shared<pcl::PointCloud<slam::XYZTPoint>>();

    pose_pre_ = Eigen::Matrix4d::Identity();

    // get ROS parameter (from '*.launch' file).
    // If parameters are not set, this function throws an exception and terminates the node.
    this->getLaunchParameters();

    // initialization
    solver_ = std::make_unique<MaplessDynamic>(nh_, rosbag_play_, dataset_name_, data_number_);

    // subscriber
    sub_lidar_ = nh_.subscribe<sensor_msgs::PointCloud2>(
            topicname_lidar_, 100, &ROSWrapper::callbackLiDAR, this);
    sub_pose_ = nh_.subscribe<nav_msgs::Odometry>(
            topicname_pose_, 100, &ROSWrapper::callbackPose, this);
// pubLaserOdometry2 = nh.advertise<> ("/integrated_to_init", 5);
    // publish
    marker_pub_ = nh_.advertise<visualization_msgs::Marker>("marker/node", 1);
    lidar_marker_pub_ = nh_.advertise<visualization_msgs::Marker>("lidar_marker/node", 1);
    
    //marker setting
    {
        marker_.header.frame_id = "map"; // map frame
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
        lidar_marker_.header.frame_id = "map"; // map frame
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
    int freq_spin; // test: 10[Hz], rosbag: <100[Hz]
    if(rosbag_play_ == true)
    {
        freq_spin = 20;
    }
    else //rosbag_play_==false //<-- pcd dataset play
    {
        freq_spin = 10;
    }
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
        ros::param::get("~topicname_pose", topicname_pose_);

        ros::param::get("~rosbag_play", rosbag_play_);
        ros::param::get("~T01_slam", T01_slam_);
        ros::param::get("~dataset_name", dataset_name_);
        ros::param::get("~data_number", data_number_);
};

void ROSWrapper::callbackPose(const nav_msgs::Odometry::ConstPtr& pose_msg){
        static int cnt_pose = 0 ;
        ros::Time time;
        // double roll, pitch, yaw;
        if (first_time_stamp_pose_ == 0.0)
        {
            first_time_stamp_pose_ = pose_msg->header.stamp.sec + pose_msg->header.stamp.nsec*1e-9;
        }
        ROS_INFO_STREAM("cnt pose: " << cnt_pose);
        msg_pose_input_time_ = pose_msg->header.stamp.sec + pose_msg->header.stamp.nsec*1e-9 - first_time_stamp_pose_;
        std::cout << "pose time: " << msg_pose_input_time_ << std::endl;
        geoQuat_ = pose_msg->pose.pose.orientation;
        // tf::Matrix3x3(tf::Quaternion(geoQuat.z, geoQuat.x, geoQuat.y, geoQuat.w)).getRPY(roll, pitch, yaw);
        
        // trans_ << pose_msg->pose.pose.position.z, pose_msg->pose.pose.position.x, pose_msg->pose.pose.position.y;
        trans_ << pose_msg->pose.pose.position.x, pose_msg->pose.pose.position.y, pose_msg->pose.pose.position.z;
        // std::cout <<"trans: " << trans_ <<std::endl;
        // double q0_ = geoQuat.w;
        // double q1_ = geoQuat.z;
        // double q2_ = geoQuat.x;
        // double q3_ = geoQuat.y;
        q0_ = geoQuat_.w;
        q1_ = geoQuat_.x;
        q2_ = geoQuat_.y;
        q3_ = geoQuat_.z;
        // std::cout <<"q0_: " << geoQuat_.w <<std::endl;
        // std::cout <<"q1_: " << geoQuat_.x <<std::endl;
        // std::cout <<"q2_: " << geoQuat_.y <<std::endl;
        // std::cout <<"q3_: " << geoQuat_.z <<std::endl;
        rot_ << 2*(q0_*q0_+q1_*q1_)-1,   2*(q1_*q2_-q0_*q3_),    2*(q1_*q3_+q0_*q2_),
                2*(q1_*q2_+q0_*q3_),    2*(q0_*q0_+q2_*q2_)-1,  2*(q2_*q3_-q0_*q1_),
                2*(q1_*q3_-q0_*q2_),    2*(q2_*q3_+q0_*q1_),    2*(q0_*q0_+q3_*q3_)-1;
        // std::cout << "rot_: " <<std::endl;
        // std::cout << rot_ << std::endl;
        pose_cur_.block(0,0,3,3) = rot_;
        pose_cur_.block(0,3,3,1) = trans_;
        pose_cur_(3,3) = 1;
        // std::cout << "pose_cur_: " << std::endl;
        // std::cout << pose_cur_ <<std::endl;
        // exit(0);

        // T01_ = pose_pre_.inverse()*pose_cur_;
        T10_ = pose_cur_.inverse()*pose_pre_;
        // std::cout << T10_ <<std::endl;
        // transform_[0] = -pitch;
        // transform_[1] = -yaw;
        // transform_[2] = roll;
        // transform_[3] = pose_msg->pose.pose.position.x;
        // transform_[4] = pose_msg->pose.pose.position.y;
        // transform_[5] = pose_msg->pose.pose.position.z;
        pose_pre_ = pose_cur_;
        is_initialized_pose_ = true;

        if (is_initialized_)
        {
            cnt_pose += 1;
        }
}

void ROSWrapper::callbackLiDAR(const sensor_msgs::PointCloud2ConstPtr& msg){
    if (first_time_stamp_pcl_ == 0.0)
    {
        first_time_stamp_pcl_ = msg->header.stamp.sec + msg->header.stamp.nsec*1e-9;
    }

    // if (is_initialized_)
    // {
    //     first_time_stamp_pcl_ = msg_pcl_input_time_;
    //     std::cout << msg_pcl_input_time_ << std::endl;
    //     std::cout << msg_pose_input_time_ << std::endl;
    //     while (msg_pcl_input_time_ < msg_pose_input_time_)
    //     {
    //         // std::cout<< "wait for pose input" <<std::endl;
    //     }
    // }

    static int cnt_data = 0;
    static int cnt_initial = 0;
    static double total_time = 0.0;
    std::cout<< " =========================== START =========================== " << cnt_data << std::endl;
    std::cout<< "Iter: "<< cnt_data << std::endl;
    msg_pcl_input_time_ = msg->header.stamp.sec + msg->header.stamp.nsec*1e-9 - first_time_stamp_pcl_; 
    std::cout << "debug/pc_raw time: " << msg_pcl_input_time_ << std::endl;

    cloudHeader_ = msg->header;
    if( is_initialized_ && is_initialized_pose_ ){ // If initialized, 
        // 0. Get the current LiDAR data
        // p1_msg_ = *msg;
        pcl::fromROSMsg(*msg, *p1_pcl_);
        pcl::fromROSMsg(*msg, *p1_pcl_wtime_);
        // std::cout << "size: p1_pcl_: "<< (*p1_pcl_).size() << ", " <<"size: p1_pcl_wtime_: " << (*p1_pcl_wtime_).size() <<std::endl;

        // 1. Calculate T01 from the SLAM (or Odometry) algorithm
        if (T01_slam_==true)
        {
            timer::tic();
            // T01 =  vo->solve();
            double dt_slam = timer::toc(); // milliseconds
            ROS_INFO_STREAM("elapsed time for 'SLAM' :" << dt_slam << " [ms]");
        }
        else
        {
            T01_ = solver_->data_buf_[cnt_data]->T_gt_; //KITTI: cnt_data
            std::cout << T01_ << std::endl;
            T10_ = T01_.inverse();
            //"00": +3
            //"01": +2
        }

        // 2. Solve the Mapless Dynamic algorithm.
        // timer::tic();
        Mask mask1;

        ROS_INFO_STREAM("Data is from rosbag");
        ROS_INFO_STREAM("p0_ size:" << p0_pcl_->size() << " // " << "p1_ size:" << p1_pcl_->size() << " ");
        // for (int i=0; i<p1_pcl_wtime_->size() ; ++i)
        // {
        //     std::cout << p1_pcl_wtime_->points[i].timestamp<<std::endl;
        // }
        // std::cout << msg->header.stamp.sec<<std::endl;
        // std::cout << msg->header.stamp.nsec<<std::endl;
        // exit(0);
        timer::tic();
        solver_->CloudFrame_next_->genRangeImages(p1_pcl_, true);
        // double dt1 = timer::toc(); // milliseconds
        // ROS_INFO_STREAM("elapsed time for 'genRangeImages' :" << dt1 << " [ms]");

        // timer::tic();
        solver_->solve(p0_pcl_, p1_pcl_, T10_, mask1, cnt_data, cloudHeader_, p1_pcl_wtime_);
        double dt_solver = timer::toc(); // milliseconds
        ROS_INFO_STREAM("elapsed time for 'solver' :" << dt_solver << " [ms]");
        total_time += dt_solver;
        ROS_INFO_STREAM("Average time for 'solver' :" << total_time/cnt_data << " [ms]" << " " << "window :"<<cnt_data);

        // solver_->pub_static_pts_.publish(msg);
        ROS_INFO("pub msg: %d",cnt_data);

        // // 3. Update the previous variables
        updatePreviousVariables(p0_pcl_, p1_pcl_, mask1, p1_pcl_wtime_);
        
        marker_.header.stamp = ros::Time::now();
        marker_.text = std::to_string(cnt_data);
        marker_.pose.position.x = 0; //T01_(0,3);     //x
        marker_.pose.position.y = 0; //T01_(1,3);     //y
        marker_.pose.position.z = 10; //T01_(2,3) + 10; //z

        lidar_marker_.header.stamp = ros::Time::now();
        lidar_marker_.pose.position.x = 0; //T01_(0,3);     //x
        lidar_marker_.pose.position.y = 0; //T01_(1,3);     //y
        lidar_marker_.pose.position.z = 0; //T01_(2,3); //z

        marker_pub_.publish(marker_);
        lidar_marker_pub_.publish(lidar_marker_);

        cnt_data += 1;
    }
    else if (cnt_initial>1)  // If not initialized, 130
    {    is_initialized_ = true;
        
        // Initialize the first data.
        p0_msg_ = *msg;
        pcl::fromROSMsg(p0_msg_, *p0_pcl_);
        // mask0.resize(p0.width, true);
        if(is_initialized_pose_==true)
        {
            solver_->CloudFrame_cur_ ->genRangeImages(p0_pcl_, true);
            ROS_INFO_STREAM("p0_ size:" << p0_pcl_->size());
        }
        // mask0_test_.resize(p0_msg_test_.width, true);
        cnt_data += 1;

        // pcl::toROSMsg(*p0_pcl_, pcl_msg_);
        // pcl_msg_.header.frame_id = "map";
        // pcl_msg_.header.stamp = cloudHeader_.stamp;
        // solver_->pub_static_pts_.publish(pcl_msg_);
        
        solver_->pub_static_pts_.publish(msg);
        ROS_INFO("pub msg: %d",cnt_data);
    }
    else
    {
        // p0_msg_ = *msg;
        // pcl::fromROSMsg(p0_msg_, *p0_pcl_);
        // pcl::toROSMsg(*p0_pcl_, pcl_msg_);
        // pcl_msg_.header.frame_id = "map";
        // pcl_msg_.header.stamp = cloudHeader_.stamp;
        // solver_->pub_static_pts_.publish(pcl_msg_);
        
        solver_->pub_static_pts_.publish(msg);
        ROS_INFO("pub msg: %d",cnt_data);

        msg_pcl_input_time_ = 0;
    }
    cnt_initial += 1;
    if (T01_slam_== false)
    {
        is_initialized_pose_ = true;
    }
};

void ROSWrapper::updatePreviousVariables(pcl::PointCloud<pcl::PointXYZI>::Ptr p0_pcl, pcl::PointCloud<pcl::PointXYZI>::Ptr p1_pcl, const Mask& mask1,
                                        CloudMessageT::Ptr p1_pcl_wtime){
    // p0_pcl_ = p1_pcl_;

    p0_pcl->resize(0);
    pcl::copyPointCloud(*p1_pcl, *p0_pcl);
    p1_pcl->resize(0);
    p1_pcl_wtime->resize(0);

    mask0.resize(mask1.size());
    std::copy(mask1.begin(), mask1.end(), mask0.begin());
};