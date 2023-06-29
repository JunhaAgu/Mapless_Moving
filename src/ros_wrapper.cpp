#include "ros_wrapper.h"

ROSWrapper::ROSWrapper(ros::NodeHandle& nh) : is_initialized_(false), nh_(nh) {
    // constructor
    ROS_INFO_STREAM("ROSWrapper - constructed.");

    p0_pcl_wtime_ = boost::make_shared<PointCloudwithTime>();
    p1_pcl_wtime_ = boost::make_shared<PointCloudwithTime>();

    pose_pre_ = Eigen::Matrix4d::Identity();

    // get ROS parameter (from '*.launch' file).
    // If parameters are not set, this function throws an exception and
    // terminates the node.
    this->getLaunchParameters();

    // initialization
    solver_ = std::make_unique<MaplessDynamic>(
        nh_, rosbag_play_, dataset_kitti_dir_, dataset_carla_dir_,
        dataset_name_, data_number_);

    // subscriber
    sub_lidar_ = nh_.subscribe<sensor_msgs::PointCloud2>(
        topicname_lidar_, 100, &ROSWrapper::callbackLiDAR, this);
    sub_pose_ = nh_.subscribe<nav_msgs::Odometry>(
        topicname_pose_, 100, &ROSWrapper::callbackPose, this);

    // publisher
    pub_marker_ = nh_.advertise<visualization_msgs::Marker>("marker/node", 1);
    pub_lidar_marker_ =
        nh_.advertise<visualization_msgs::Marker>("lidar_marker/node", 1);

    // marker setting
    {
        marker_.header.frame_id = "map";  // map frame
        marker_.color.a = 1.0;            // Don't forget to set the alpha!
        marker_.color.r = 0.0;
        marker_.color.g = 1.0;
        marker_.color.b = 0.0;
        marker_.scale.x = 0.1;
        marker_.scale.y = 0.1;
        marker_.scale.z = 3.0;
        marker_.type =
            visualization_msgs::Marker::TEXT_VIEW_FACING;  // TEXT_VIEW_FACING;
                                                           // SPHERE;
        marker_.id = 0;
        marker_.action = visualization_msgs::Marker::ADD;
        marker_.pose.orientation.x = 0.0;
        marker_.pose.orientation.y = 0.0;
        marker_.pose.orientation.z = 0.0;
        marker_.pose.orientation.w = 1.0;
    }
    // lidar marker setting
    {
        lidar_marker_.header.frame_id = "map";  // map frame
        lidar_marker_.color.a = 1.0;  // Don't forget to set the alpha!
        lidar_marker_.color.r = 0.0;
        lidar_marker_.color.g = 1.0;
        lidar_marker_.color.b = 0.0;
        lidar_marker_.scale.x = 1.0;
        lidar_marker_.scale.y = 1.0;
        lidar_marker_.scale.z = 1.0;
        lidar_marker_.type =
            visualization_msgs::Marker::CYLINDER;  // TEXT_VIEW_FACING; SPHERE;
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

ROSWrapper::~ROSWrapper() {
    // destructor
    ROS_INFO_STREAM("ROSWrapper - deleted.");
};

void ROSWrapper::run() {
    int freq_spin;  // test: 10[Hz], rosbag: <100[Hz]
    if (rosbag_play_ == true) {
        freq_spin = 20;
    } else  // rosbag_play_==false //<-- pcd dataset play
    {
        freq_spin = 10;
    }
    ros::Rate rate(freq_spin);
    ROS_INFO_STREAM("ROSWrapper - 'run()' - run at [" << freq_spin << "] Hz.");
    ROS_INFO_STREAM("Rosbag can be started");
    while (ros::ok()) {
        if (rosbag_play_ == false) {
            std::cout << " " << std::endl;
            ROS_INFO_STREAM("Data is from saved pcd"
                            << " - "
                            << "run at [" << freq_spin << "] Hz.");
            solver_->TEST();
        } else {
        }

        ros::spinOnce();
        rate.sleep();
    }
    ROS_INFO_STREAM("ROSWrapper - 'run()' - run ends.");
};

void ROSWrapper::getLaunchParameters() {
    if (!ros::param::has("~topicname_lidar"))
        throw std::runtime_error(
            "ROSWrapper - no 'topicname_lidar' is set. You might run the node "
            "by 'roslaunch' with parameter settings.\n");
    ros::param::get("~topicname_lidar", topicname_lidar_);
    ros::param::get("~topicname_pose", topicname_pose_);

    ros::param::get("~rosbag_play", rosbag_play_);
    ros::param::get("~T01_slam", T01_slam_);
    ros::param::get("~dataset_name", dataset_name_);
    ros::param::get("~data_number", data_number_);

    ros::param::get("~dataset_kitti_dir", dataset_kitti_dir_);
    ros::param::get("~dataset_carla_dir", dataset_carla_dir_);
};

void ROSWrapper::callbackPose(const nav_msgs::Odometry::ConstPtr& pose_msg) {
    static int cnt_pose = 0;
    ros::Time time;
    // double roll, pitch, yaw;
    if (first_timestamp_pose_msg_ == 0.0) {
        first_timestamp_pose_msg_ =
            pose_msg->header.stamp.sec + pose_msg->header.stamp.nsec * 1e-9;
    }
    ROS_INFO_STREAM("cnt pose: " << cnt_pose);
    timestamp_pose_msg_ = pose_msg->header.stamp.sec +
                          pose_msg->header.stamp.nsec * 1e-9 -
                          first_timestamp_pose_msg_;
    std::cout << "pose time: " << timestamp_pose_msg_ << std::endl;
    geoQuat_ = pose_msg->pose.pose.orientation;
    // tf::Matrix3x3(tf::Quaternion(geoQuat.z, geoQuat.x, geoQuat.y,
    // geoQuat.w)).getRPY(roll, pitch, yaw);
    trans_ << pose_msg->pose.pose.position.x, pose_msg->pose.pose.position.y,
        pose_msg->pose.pose.position.z;

    q0_ = geoQuat_.w;
    q1_ = geoQuat_.x;
    q2_ = geoQuat_.y;
    q3_ = geoQuat_.z;
    rot_ << 2 * (q0_ * q0_ + q1_ * q1_) - 1, 2 * (q1_ * q2_ - q0_ * q3_),
        2 * (q1_ * q3_ + q0_ * q2_), 2 * (q1_ * q2_ + q0_ * q3_),
        2 * (q0_ * q0_ + q2_ * q2_) - 1, 2 * (q2_ * q3_ - q0_ * q1_),
        2 * (q1_ * q3_ - q0_ * q2_), 2 * (q2_ * q3_ + q0_ * q1_),
        2 * (q0_ * q0_ + q3_ * q3_) - 1;
    pose_cur_.block(0, 0, 3, 3) = rot_;
    pose_cur_.block(0, 3, 3, 1) = trans_;
    pose_cur_(3, 3) = 1;

    T10_ = pose_cur_.inverse() * pose_pre_;

    pose_pre_ = pose_cur_;
    is_initialized_pose_ = true;

    if (is_initialized_)  // for mappless dynamic
    {
        cnt_pose += 1;
    }
}

void ROSWrapper::callbackLiDAR(const sensor_msgs::PointCloud2ConstPtr& msg) {
    if (first_timestamp_pcl_msg_ == 0.0) {
        first_timestamp_pcl_msg_ =
            msg->header.stamp.sec + msg->header.stamp.nsec * 1e-9;
    }

    static int cnt_pcl = 0;
    static int cnt_initial = 0;
    static double total_time = 0.0;

    static double total_time_B = 0.0;

    std::cout << " ====================== START ====================== "
              << std::endl;
    std::cout << "Iter: " << cnt_pcl << std::endl;
    timestamp_pcl_msg_ = msg->header.stamp.sec + msg->header.stamp.nsec * 1e-9 -
                         first_timestamp_pcl_msg_;
    std::cout << "pcl(debug/pc_raw) time: " << timestamp_pcl_msg_ << std::endl;

    cloudHeader_ = msg->header;
    if (is_initialized_ && is_initialized_pose_) {  // If initialized,
        // 0. Get the next LiDAR data
        pcl::fromROSMsg(*msg, *p1_pcl_wtime_);

        // 1. Calculate T01 from the SLAM (or Odometry) algorithm
        if (T01_slam_ == true) {
        } else {
            T01_ = solver_->data_buf_[cnt_pcl]->T_gt_;  // KITTI: cnt_pcl
            std::cout << T01_ << std::endl;
            T10_ = T01_.inverse();
            //"00": +3
            //"01": +2
        }

        // 2. Solve the Mapless Dynamic algorithm.

        Mask mask1;

        ROS_INFO_STREAM("Data is from rosbag");
        ROS_INFO_STREAM("p0_ size:" << p0_pcl_wtime_->size());
        ROS_INFO_STREAM("p1_ size:" << p1_pcl_wtime_->size());

        timer::tic();
        solver_->CloudFrame_next_->genRangeImages(p1_pcl_wtime_, true);
        double dt1 = timer::toc();  // milliseconds
        total_time_B += dt1;
        ROS_INFO_STREAM("Average time for 'B' :" << total_time_B / cnt_pcl
                                                 << " [ms]"
                                                 << " "
                                                 << "window :" << cnt_pcl);
        // ROS_INFO_STREAM("elapsed time for 'genRangeImages' :" << dt1 << "
        // [ms]");

        solver_->solve(p0_pcl_wtime_, p1_pcl_wtime_, T10_, mask1, cnt_pcl,
                       cloudHeader_);
        // double dt_solver = timer::toc(); // milliseconds
        // ROS_INFO_STREAM("elapsed time for 'solver' :" << dt_solver << "
        // [ms]"); total_time += dt_solver; ROS_INFO_STREAM("Average time for
        // 'solver' :" << total_time/cnt_pcl << " [ms]" << " " << "window
        // :"<<cnt_pcl);

        ROS_INFO("pub msg: %d", cnt_pcl);

        // // 3. Update the previous variables
        updatePreviousVariables(p0_pcl_wtime_, p1_pcl_wtime_, mask1);

        marker_.header.stamp = ros::Time::now();
        marker_.text = std::to_string(cnt_pcl);
        marker_.pose.position.x = 0;   // T01_(0,3);     //x
        marker_.pose.position.y = 0;   // T01_(1,3);     //y
        marker_.pose.position.z = 10;  // T01_(2,3) + 10; //z

        lidar_marker_.header.stamp = ros::Time::now();
        lidar_marker_.pose.position.x = 0;  // T01_(0,3);     //x
        lidar_marker_.pose.position.y = 0;  // T01_(1,3);     //y
        lidar_marker_.pose.position.z = 0;  // T01_(2,3); //z

        pub_marker_.publish(marker_);
        pub_lidar_marker_.publish(lidar_marker_);

        cnt_pcl += 1;
    } else if (cnt_initial > 1)  // If not initialized, 129 or 1
    {
        is_initialized_ = true;

        // Initialize the first data.
        pcl::fromROSMsg(*msg, *p0_pcl_wtime_);

        if (is_initialized_pose_ == true) {
            solver_->CloudFrame_cur_->genRangeImages(p0_pcl_wtime_, true);
            ROS_INFO_STREAM("p0_ size:" << p0_pcl_wtime_->size());
        }

        cnt_pcl += 1;

        solver_->pub_static_pts_.publish(msg);
        ROS_INFO("pub msg: %d", cnt_pcl);
    } else {
        solver_->pub_static_pts_.publish(msg);  // publish all pcl
        ROS_INFO("Mapless Dynamic: Not started yet");
        ROS_INFO("pub msg: %d", cnt_pcl);

        timestamp_pcl_msg_ = 0.0;
    }
    cnt_initial += 1;
    if (T01_slam_ == false) {
        is_initialized_pose_ = true;
    }
};

void ROSWrapper::updatePreviousVariables(PointCloudwithTime::Ptr p0_pcl_wtime,
                                         PointCloudwithTime::Ptr p1_pcl_wtime,
                                         const Mask& mask1) {
    p0_pcl_wtime->resize(0);
    pcl::copyPointCloud(*p1_pcl_wtime, *p0_pcl_wtime);
    p1_pcl_wtime->resize(0);
};