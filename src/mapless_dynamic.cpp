#include "mapless_dynamic.h"
#include <string>
#include <fstream>

#include <user_param.h>

MaplessDynamic::MaplessDynamic(ros::NodeHandle& nh, bool test_flag)
: nh_(nh), test_flag_(test_flag), is_initialized_test_(false)
 {
    // constructor
    ROS_INFO_STREAM("MaplessDynamic - constructed.");
    // IMPLEMENT YOUR CODE FROM THIS LINE.

    pub_dynamic_pts_ = nh_.advertise<sensor_msgs::PointCloud2>("/dynamic_pts",1);
    pub_static_pts_  = nh_.advertise<sensor_msgs::PointCloud2>("/static_pts",1);
    
    // Class UserParam
    std::unique_ptr<UserParam> UserParam_;
    UserParam_ = std::make_unique<UserParam>();
    UserParam_->getUserSettingParameters();

    this->getUserSettingParameters();
    
    // Class initialization
    CloudFrame_     = std::make_unique<CloudFrame>(UserParam_);
    SegmentGround_  = std::make_unique<SegmentGround>(UserParam_);
    dRCalc_         = std::make_unique<dRCalc>(UserParam_);
    PclWarp_        = std::make_unique<PclWarp>(UserParam_);
    ObjectExt_      = std::make_unique<ObjectExt>(UserParam_);

    // allocation for solver
    accumulated_dRdt_       = cv::Mat::zeros(img_height_, img_width_, CV_32FC1);
    accumulated_dRdt_score_ = cv::Mat::zeros(img_height_, img_width_, CV_32FC1);
    residual_               = cv::Mat::zeros(img_height_, img_width_, CV_32FC1);

    str_cur_            = new StrRhoPts();
    str_next_           = new StrRhoPts();
    str_cur_warped_     = new StrRhoPts();
    // str_warpPointcloud_ = new StrRhoPts();

    str_cur_->rho.reserve(500000);
    str_cur_->phi.reserve(500000);
    str_cur_->theta.reserve(500000);
    str_cur_->img_rho   = cv::Mat::zeros(img_height_, img_width_, CV_32FC1);
    str_cur_->img_index = cv::Mat::zeros(img_height_, img_width_, CV_32SC1);

    str_cur_->img_x     = cv::Mat::zeros(img_height_, img_width_, CV_32FC1);
    str_cur_->img_y     = cv::Mat::zeros(img_height_, img_width_, CV_32FC1);
    str_cur_->img_z     = cv::Mat::zeros(img_height_, img_width_, CV_32FC1);

    str_cur_->pts_per_pixel_n.resize(img_height_*img_width_);
    str_cur_->pts_per_pixel_index.resize(img_height_*img_width_);
    str_cur_->pts_per_pixel_rho.resize(img_height_ * img_width_);
    str_cur_->pts_per_pixel_index_valid.resize(img_height_ * img_width_);
    for (int i=0; i<img_height_*img_width_; ++i)
    {
        str_cur_->pts_per_pixel_index[i].reserve(5000);
    }
    for (int i=0; i<img_height_*img_width_; ++i)
    {
        str_cur_->pts_per_pixel_rho[i].reserve(5000);
    }
    for (int i=0; i<img_height_*img_width_; ++i)
    {
        str_cur_->pts_per_pixel_index_valid[i].reserve(5000);
    }
    str_cur_->pts        = boost::make_shared<pcl::PointCloud<pcl::PointXYZ>>();
    str_cur_->ptsInImage = boost::make_shared<pcl::PointCloud<pcl::PointXYZ>>();
    str_cur_->img_restore_mask      = cv::Mat::zeros(img_height_, img_width_, CV_32SC1);
    str_cur_->img_restore_warp_mask = cv::Mat::zeros(img_height_, img_width_, CV_32SC1);

    str_next_->rho.reserve(500000);
    str_next_->phi.reserve(500000);
    str_next_->theta.reserve(500000);
    str_next_->img_rho   = cv::Mat::zeros(img_height_, img_width_, CV_32FC1);
    str_next_->img_index = cv::Mat::zeros(img_height_, img_width_, CV_32SC1);

    str_next_->img_x     = cv::Mat::zeros(img_height_, img_width_, CV_32FC1);
    str_next_->img_y     = cv::Mat::zeros(img_height_, img_width_, CV_32FC1);
    str_next_->img_z     = cv::Mat::zeros(img_height_, img_width_, CV_32FC1);

    str_next_->pts_per_pixel_n.resize(img_height_*img_width_);
    str_next_->pts_per_pixel_index.resize(img_height_*img_width_);
    str_next_->pts_per_pixel_rho.resize(img_height_ * img_width_);
    str_next_->pts_per_pixel_index_valid.resize(img_height_ * img_width_);
    for (int i=0; i<img_height_*img_width_; ++i)
    {
        str_next_->pts_per_pixel_index[i].reserve(5000);
    }
    for (int i=0; i<img_height_*img_width_; ++i)
    {
        str_next_->pts_per_pixel_rho[i].reserve(5000);
    }
    for (int i=0; i<img_height_*img_width_; ++i)
    {
        str_next_->pts_per_pixel_index_valid[i].reserve(5000);
    }
    str_next_->pts        = boost::make_shared<pcl::PointCloud<pcl::PointXYZ>>();
    str_next_->ptsInImage = boost::make_shared<pcl::PointCloud<pcl::PointXYZ>>();
    str_next_->img_restore_mask      = cv::Mat::zeros(img_height_, img_width_, CV_32SC1);
    str_next_->img_restore_warp_mask = cv::Mat::zeros(img_height_, img_width_, CV_32SC1);
    
    groundPtsIdx_next_ = cv::Mat::zeros(img_height_, img_width_, CV_8UC1);

    str_cur_warped_->rho.reserve(500000);
    str_cur_warped_->phi.reserve(500000);
    str_cur_warped_->theta.reserve(500000);
    str_cur_warped_->img_rho   = cv::Mat::zeros(img_height_, img_width_, CV_32FC1);
    str_cur_warped_->img_index = cv::Mat::zeros(img_height_, img_width_, CV_32SC1);

    str_cur_warped_->img_x = cv::Mat::zeros(img_height_, img_width_, CV_32FC1);
    str_cur_warped_->img_y = cv::Mat::zeros(img_height_, img_width_, CV_32FC1);
    str_cur_warped_->img_z = cv::Mat::zeros(img_height_, img_width_, CV_32FC1);

    str_cur_warped_->pts_per_pixel_n.resize(img_height_*img_width_);
    str_cur_warped_->pts_per_pixel_index.resize(img_height_*img_width_);
    str_cur_warped_->pts_per_pixel_rho.resize(img_height_ * img_width_);
    str_cur_warped_->pts_per_pixel_index_valid.resize(img_height_ * img_width_);
    for (int i=0; i<img_height_*img_width_; ++i)
    {
        str_cur_warped_->pts_per_pixel_index[i].reserve(5000);
    }
    for (int i=0; i<img_height_*img_width_; ++i)
    {
        str_cur_warped_->pts_per_pixel_rho[i].reserve(5000);
    }
    for (int i=0; i<img_height_*img_width_; ++i)
    {
        str_cur_warped_->pts_per_pixel_index_valid[i].reserve(5000);
    }
    str_cur_warped_->pts        = boost::make_shared<pcl::PointCloud<pcl::PointXYZ>>();
    str_cur_warped_->ptsInImage = boost::make_shared<pcl::PointCloud<pcl::PointXYZ>>();
    str_cur_warped_->img_restore_mask = cv::Mat::zeros(img_height_, img_width_, CV_32SC1);
    str_cur_warped_->img_restore_warp_mask = cv::Mat::zeros(img_height_, img_width_, CV_32SC1);

    // str_warpPointcloud_->rho.reserve(500000);
    // str_warpPointcloud_->phi.reserve(500000);
    // str_warpPointcloud_->theta.reserve(500000);
    // str_warpPointcloud_->img_rho   = cv::Mat::zeros(img_height_, img_width_, CV_32FC1);
    // str_warpPointcloud_->img_index = cv::Mat::zeros(img_height_, img_width_, CV_32SC1);

    // str_warpPointcloud_->img_x = cv::Mat::zeros(img_height_, img_width_, CV_32FC1);
    // str_warpPointcloud_->img_y = cv::Mat::zeros(img_height_, img_width_, CV_32FC1);
    // str_warpPointcloud_->img_z = cv::Mat::zeros(img_height_, img_width_, CV_32FC1);

    // str_warpPointcloud_->pts_per_pixel_n.resize(img_height_ * img_width_);
    // str_warpPointcloud_->pts_per_pixel_index.resize(img_height_ * img_width_);
    // str_warpPointcloud_->pts_per_pixel_rho.resize(img_height_ * img_width_);
    // str_warpPointcloud_->pts_per_pixel_index_valid.resize(img_height_ * img_width_);
    // for (int i=0; i<img_height_*img_width_; ++i)
    // {
    //     str_warpPointcloud_->pts_per_pixel_index[i].reserve(5000);
    // }
    // for (int i=0; i<img_height_*img_width_; ++i)
    // {
    //     str_warpPointcloud_->pts_per_pixel_rho[i].reserve(5000);
    // }
    // for (int i=0; i<img_height_*img_width_; ++i)
    // {
    //     str_warpPointcloud_->pts_per_pixel_index_valid[i].reserve(5000);
    // }
    // str_warpPointcloud_->pts        = boost::make_shared<pcl::PointCloud<pcl::PointXYZ>>();
    // str_warpPointcloud_->ptsInImage = boost::make_shared<pcl::PointCloud<pcl::PointXYZ>>();
    // str_warpPointcloud_->img_restore_mask = cv::Mat::zeros(img_height_, img_width_, CV_32SC1);
    // str_warpPointcloud_->img_restore_warp_mask = cv::Mat::zeros(img_height_, img_width_, CV_32SC1);

    // velo_cur_.reserve(500000);
    // velo_xyz_.reserve(500000);
    ptr_cur_pts_warped_ = boost::make_shared<pcl::PointCloud<pcl::PointXYZ>>();
    // pts_warpewd_        = boost::make_shared<pcl::PointCloud<pcl::PointXYZ>>();

    object_row.reserve(100000);
    object_col.reserve(100000);
    object_rho_roi.reserve(100000);

    filled_object_row.reserve(100000);
    filled_object_col.reserve(100000);
    filled_object_rho_roi.reserve(100000);

    rho_zero_filled_value_row.reserve(100000);
    rho_zero_filled_value_col.reserve(100000);
    rho_zero_filled_value_rho_roi.reserve(100000);

    max_his_filled_object_rho_roi.reserve(100000);

    disconti_row.reserve(100000);
    disconti_col.reserve(100000);

    if (test_flag_){
        this->loadTestData();
    }
    // END YOUR CODE
};


MaplessDynamic::~MaplessDynamic() {
    // destructor
    ROS_INFO_STREAM("MaplessDynamic - deleted.");
    // IMPLEMENT YOUR CODE FROM THIS LINE.

    delete str_cur_;
    delete str_next_;
    delete str_cur_warped_;
    // delete str_warpPointcloud_;

    // END YOUR CODE
};


void MaplessDynamic::TEST(){
    static int cnt_data = 0;
    std::cout<< data_buf_.size() << std::endl;
    std::cout<< "Test iter: "<< cnt_data << std::endl;
    // exit(0);
    // The test function.
    // IMPLEMENT YOUR CODE FROM THIS LINE.

    // p0 = get!
    // p1 = get!
    // T01 = get!

    // this->algorithm(p0,p1,T01);
    
    // END YOUR CODE

    if( is_initialized_test_ ){ // If initialized, 
        // 0. Get the current LiDAR data
        p1_msg_test_ = *(data_buf_[cnt_data]->pcl_msg_);
        pcl::fromROSMsg(p1_msg_test_, p1_pcl_test_);

        // if (cnt_data == 3)
        // {
        //     std::cout << p1_pcl_test_.size() << std::endl;
        //     sensor_msgs::PointCloud2 converted_msg_d;
        //     pcl::toROSMsg(p1_pcl_test_, converted_msg_d);
        //     converted_msg_d.header.frame_id = "map";
        //     pub_dynamic_pts_.publish(converted_msg_d);

        //     std::cout << p0_pcl_test_.size() << std::endl;
        //     sensor_msgs::PointCloud2 converted_msg_s;
        //     pcl::toROSMsg(p0_pcl_test_, converted_msg_s);
        //     converted_msg_s.header.frame_id = "map";
        //     pub_static_pts_.publish(converted_msg_s);
        //     exit(0);
        // }

        // 1. Calculate T01 from the SLAM (or Odometry) algorithm
        Pose T01;
        timer::tic();
        // T01 =  vo->solve();
        T01 = data_buf_[cnt_data]->T_gt_.inverse();
        double dt_slam = timer::toc(); // milliseconds
        ROS_INFO_STREAM("elapsed time for 'SLAM' :" << dt_slam << " [ms]");

        // 2. Solve the Mapless Dynamic algorithm.
        // timer::tic();
        Mask mask1;
        this->solve(p0_pcl_test_, p1_pcl_test_, T01, mask1, cnt_data);
        // double dt_solver = timer::toc(); // milliseconds
        // ROS_INFO_STREAM("elapsed time for 'solver' :" << dt_solver << " [ms]");

        // 3. Update the previous variables
        // updatePreviousVariables(p1, mask1);
    }
    else { // If not initialized, 
        is_initialized_test_ = true;
        
        // Initialize the first data.
        p0_msg_test_ = *(data_buf_[0]->pcl_msg_);
        pcl::fromROSMsg(p0_msg_test_, p0_pcl_test_);
        
        CloudFrame_->genRangeImages(p0_pcl_test_, str_cur_, 1);

        mask0_test_.resize(p0_msg_test_.width, true);
    }
    cnt_data += 1;
    // std::cout << cnt_data << std::endl;
};



void MaplessDynamic::loadTestData(){
    std::string data_num = "07";
    std::string dataset_dir = "/home/junhakim/KITTI_odometry/";
    float pose_arr[12];
    Pose T_tmp;
    Pose T_warp;
    T_warp << 0, -1, 0, 0,
        0, 0, -1, 0,
        1, 0, 0, 0,
        0, 0, 0, 1;
    
    int start_num;
    int final_num;
    if (data_num == "00"){
        start_num = 4390 + 00;
        final_num = 4530 + 1; // 1 ~ 141+2 75
    }
    else if (data_num == "01"){
        start_num = 150 + 00;
        final_num = 250 + 1; // 1 ~ 101+2
    }
    else if (data_num == "02"){
        start_num = 860 + 00;
        final_num = 950 + 1; // 1 ~ 91+2
    }
    else if (data_num == "05"){
        start_num = 2350 + 144+41;
        final_num = 2670 + 1; // 1 ~ 321+2
        //         start_num = 2350+269; final_num = 2670+1; // 1 ~ 321+2
    }
    else if (data_num == "07"){
        start_num = 630 + 00;
        final_num = 820 + 1; // 1 ~ 190+2
        //         start_num = 630+140; final_num = 820+1; // 1 ~ 190+2
    }
    start_num = start_num - 1;
    final_num = final_num - 1;
    
    // read Association --> n_data_
    std::string pose_dir = dataset_dir + "data_odometry_poses/dataset/poses/" + data_num + ".txt";
    std::ifstream posefile;
    posefile.open(pose_dir.c_str());
    std::string s;
    int cnt_line = 0;

    std::cout << "Test data directory: " << pose_dir << std::endl;
    while (getline(posefile, s, '\n'))
    {
        ++cnt_line;
    }
    posefile.close();
    n_data_ = cnt_line;
    // std::cout << cnt_line << std::endl;
    
    // allocation
    data_buf_.reserve(n_data_); //resize error...
    all_pose_.reserve(n_data_); //resize error...
    all_T_gt_.reserve(n_data_);
    for(int i=0; i<n_data_; ++i){
        all_pose_[i].reserve(12);
        data_buf_.push_back(new TestData);

        data_buf_[i]->pcl_ = boost::make_shared<pcl::PointCloud<pcl::PointXYZI>>();
        data_buf_[i]->pcl_msg_ = (new sensor_msgs::PointCloud2);
    }

    // read pose file
    posefile.open(pose_dir.c_str());
    cnt_line = 0;
    while (!posefile.eof())
    {
        getline(posefile, s);
        if (!s.empty())
        {
            std::stringstream ss;
            ss << s;
            for (int i = 0; i < 12; ++i)
            {
                ss >> pose_arr[i];
                all_pose_[cnt_line].push_back(pose_arr[i]);
                // std::cout << pose_arr[i] << std::endl;
            }
            ++cnt_line;
        }
    }
    int n_valid_data = final_num - start_num + 1;

    for (int i=0; i<n_valid_data; ++i)
    {
        valid_data_.push_back(start_num+i);
    }

    for (int ii = 0; ii < n_valid_data; ++ii)
    {
        int i = valid_data_[ii];
        T_tmp = Pose::Zero(4, 4);
        T_tmp(3, 3) = 1;
        T_tmp.block<1, 4>(0, 0) << all_pose_[i][0], all_pose_[i][1], all_pose_[i][2], all_pose_[i][3];
        T_tmp.block<1, 4>(1, 0) << all_pose_[i][4], all_pose_[i][5], all_pose_[i][6], all_pose_[i][7];
        T_tmp.block<1, 4>(2, 0) << all_pose_[i][8], all_pose_[i][9], all_pose_[i][10], all_pose_[i][11];

        if (test_data_type_ == "KITTI")
        {
            all_T_gt_[i] = T_tmp * T_warp;
        }
        else if (test_data_type_ == "CARLA")
        {
            all_T_gt_[i] = T_tmp;
        }
        
    }
    for (int ii = 0; ii < n_valid_data; ++ii)
    {
        int i = valid_data_[ii];
        if (test_data_type_ == "KITTI")
        {
            if (ii == 0)
            {
                data_buf_[ii]->T_gt_ = all_T_gt_[i].inverse() * all_T_gt_[i];
            }
            else
            {
                data_buf_[ii]->T_gt_ = all_T_gt_[i - 1].inverse() * all_T_gt_[i];
            }

            // std::cout << data_buf_[ii]->T_gt_ << std::endl;
        }
        else if (test_data_type_ == "CARLA")
        {
            if (ii == 0){}
            else
            {
                data_buf_[ii-1]->T_gt_ = all_T_gt_[i-1].inverse() * all_T_gt_[i-1];
            }
        }
    }

    // LiDAR data (bin) read
    std::string bin_path;
    bin_path = dataset_dir + "data_odometry_velodyne/dataset/sequences/" + data_num + "/velodyne/";
    read_filelists(bin_path, file_lists_, "bin");
    sort_filelists(file_lists_, "bin");

    #pragma omp parallel num_threads(8)
    #pragma omp parallel for
    for (int i = valid_data_[0]; i < valid_data_[valid_data_.size()-1]+1; ++i)
    {   
        std::string bin_file = bin_path + file_lists_[i];
        // std::cout << i << std::endl;
        readKittiPclBinData(bin_file, i);
    }

    int v_factor = 1;
    // vertical angle resolusion for KITTI or CARLA
    if (test_data_type_ == "KITTI")
    {
        float inter_top = (2.5 - (-8.0)) / (32 / v_factor - 1);
        for (int i = 0; i < 32 / v_factor; ++i)
        {
            if (i == 31)
            {
                v_angle_.push_back(-8.0);
            }
            else
            {
                v_angle_.push_back(2.5 - inter_top * i);
            }
        }
        float inter_bottom = (-8.50 - (-23.8)) / (32 / v_factor - 1);
        for (int i = 0; i < 32 / v_factor; ++i)
        {

            if (i == 31)
            {
                v_angle_.push_back(-23.8);
            }
            else
            {
                v_angle_.push_back(-8.50 - inter_bottom * i);
            }
        }
    }
    else if (test_data_type_ == "CARLA")
    {
        float inter = (2.0 - (-24.8)) / (64);
        for (int i = 0; i < 64 / v_factor; ++i)
        {
            if (i == 63)
            {
                v_angle_.push_back(-24.8);
            }
            else
            {
                v_angle_.push_back(2.0 - inter * i);
            }
        }
    }
    // for (int i=0; i<v_angle_.size(); ++i)
    // std::cout << v_angle_[i] <<std::endl;
    // exit(0);
}

void MaplessDynamic::solve(
    /* inputs */ //const sensor_msgs::PointCloud2& p1
    pcl::PointCloud<pcl::PointXYZ>& p0, pcl::PointCloud<pcl::PointXYZ>& p1, const Pose& T01, 
    /* outputs */ 
    Mask& mask1, int cnt_data)
{
    // IMPLEMENT YOUR ALGORITHM FROM THIS LINE.

    // do something...

    // END YOUR ALGORITHM

    // if (cnt_data == 3)
    // {
    //     std::cout << p0.size()<<std::endl;
    //     sensor_msgs::PointCloud2 converted_msg_d;
    //     pcl::toROSMsg(p0, converted_msg_d);
    //     converted_msg_d.header.frame_id = "map";
    //     pub_dynamic_pts_.publish(converted_msg_d);

    //     sensor_msgs::PointCloud2 converted_msg_s;
    //     pcl::toROSMsg(p1, converted_msg_s);
    //     converted_msg_s.header.frame_id = "map";
    //     pub_static_pts_.publish(converted_msg_s);
    //     exit(0);
    // }

    // timer::tic();
    // double dt_slam = timer::toc(); // milliseconds
    // ROS_INFO_STREAM("elapsed time for 'compensateCurRhoZeroWarp' :" << dt_slam << " [ms]");

    // pointcloud input, p1
    // p0 is already processed in initial step
    timer::tic();
    CloudFrame_->genRangeImages(p1, str_next_, 1);
    // genRangeImages(p1, str_next_, 1);
    // CloudFilter cloud_filter = CloudFilter(p1, str_next_ ,1);

    double dt_toc1 = timer::toc(); // milliseconds
    ROS_INFO_STREAM("elapsed time for 'genRangeImages' :" << dt_toc1 << " [ms]");

    // Warp pcl represented in current frame to next frame
    T_next2cur_ = T01;
    
    // Segment ground
    timer::tic();
    SegmentGround_->fastsegmentGround(str_next_, groundPtsIdx_next_);
    double dt_toc2 = timer::toc(); // milliseconds
    ROS_INFO_STREAM("elapsed time for 'segmentSGround' :" << dt_toc2 << " [ms]");

    //// Occlusion accumulation ////
    // Compute the occlusion dRdt
    
    timer::tic();
    dRCalc_->dR_warpPointcloud(str_next_, str_cur_, p0, T_next2cur_, cnt_data, str_cur_warped_, residual_, velo_cur_, ptr_cur_pts_warped_, CloudFrame_);
    double dt_toc3 = timer::toc(); // milliseconds
    ROS_INFO_STREAM("elapsed time for 'dR_warpPointcloud' :" << dt_toc3 << " [ms]");
    // str_next_->state();

    timer::tic();
    // warp the occlusion accumulation map
    PclWarp_->warpPointcloud(str_cur_, T_next2cur_, accumulated_dRdt_, cnt_data, CloudFrame_);
    PclWarp_->initializeStructAndPcl();
    PclWarp_->warpPointcloud(str_cur_, T_next2cur_, accumulated_dRdt_score_, cnt_data, CloudFrame_);
    double dt_toc4 = timer::toc(); // milliseconds
    ROS_INFO_STREAM("elapsed time for 'warpPointcloud' :" << dt_toc4 << " [ms]");
    
    //     if (cnt_data == 2)
    // {
    //     cv::imshow("before d", accumulated_dRdt_);
    //     countZerofloat(accumulated_dRdt_);
    //     cv::imshow("before  k", accumulated_dRdt_score_);
    //     countZerofloat(accumulated_dRdt_score_);
    // }
    timer::tic();
    // filter out outliers
    ObjectExt_->filterOutAccumdR(str_next_, str_cur_warped_, accumulated_dRdt_, accumulated_dRdt_score_, residual_);
    double dt_toc5 = timer::toc(); // milliseconds
    ROS_INFO_STREAM("elapsed time for 'filterOutAccumdR' :" << dt_toc5 << " [ms]");
    // if (cnt_data == 2)
    // {
    //     cv::imshow("d", accumulated_dRdt_);
    //     countZerofloat(accumulated_dRdt_);
    //     cv::imshow("k", accumulated_dRdt_score_);
    //     countZerofloat(accumulated_dRdt_score_);
    //     cv::waitKey(0);
    //     exit(0);
    // }
    timer::tic();
    // Extract object candidate via connected components in 2-D binary image
    ObjectExt_->extractObjectCandidate(accumulated_dRdt_, str_next_);
    double dt_toc6 = timer::toc(); // milliseconds
    ROS_INFO_STREAM("elapsed time for 'extractObjectCandidate' :" <<  dt_toc6 << " [ms]");

    //// update object_mask
    //object_mask = accumulated_dRdt>0;
    timer::tic();
    // Fast Segment
    ObjectExt_->checkSegment(accumulated_dRdt_, str_next_, groundPtsIdx_next_);
    double dt_toc7 = timer::toc(); // milliseconds
    ROS_INFO_STREAM("elapsed time for 'checkSegment' :" <<  dt_toc7 << " [ms]");

    //// update object_mask
    //object_mask = accumulated_dRdt>0;
    timer::tic();
    ObjectExt_->updateScore(accumulated_dRdt_, accumulated_dRdt_score_);
    double dt_toc8 = timer::toc(); // milliseconds
    ROS_INFO_STREAM("elapsed time for 'updateScore' :" <<  dt_toc8 << " [ms]");

    timer::tic();
    plugImageZeroHoles(accumulated_dRdt_, accumulated_dRdt_score_, str_next_, groundPtsIdx_next_, object_threshold_);
    double dt_toc9 = timer::toc(); // milliseconds
    ROS_INFO_STREAM("elapsed time for 'plugImageZeroHoles' :" <<  dt_toc9 << " [ms]");

    timer::tic();
    float* ptr_accumulated_dRdt_ = accumulated_dRdt_.ptr<float>(0);
    float* ptr_accumulated_dRdt_score_ = accumulated_dRdt_score_.ptr<float>(0);
    float* ptr_next_img_rho = str_next_->img_rho.ptr<float>(0);

    float* ptr_dRdt = residual_.ptr<float>(0);

    for (int i = 0; i < img_height_; ++i)
    {
        int i_ncols = i * img_width_;
        for (int j = 0; j < img_width_; ++j)
        {
            if (*(ptr_accumulated_dRdt_ + i_ncols + j)!=0 && *(ptr_next_img_rho + i_ncols + j)!=0)
            {
            }
            else
            {
                *(ptr_accumulated_dRdt_ + i_ncols + j) = 0.0;
            }
        }
    }

    bool flag_exist_non_zero = false;

    for (int i = 0; i < img_height_; ++i)
    {
        int i_ncols = i * img_width_;
        for (int j = 0; j < img_width_; ++j)
        {
            if (*(ptr_accumulated_dRdt_ + i_ncols + j)!=0)
            {
                flag_exist_non_zero = true;
                break;
            }
        }
    }

    if (flag_exist_non_zero == false)
    {
        for (int i = 0; i < img_height_; ++i)
        {
            int i_ncols = i * img_width_;
            for (int j = 0; j < img_width_; ++j)
            {
                if ((*(ptr_dRdt + i_ncols + j) < 0.0) && *(ptr_dRdt + i_ncols + j) > -0.1 * *(ptr_next_img_rho + i_ncols + j) && (*(ptr_accumulated_dRdt_score_ + i_ncols + j) > 1.0))
                {
                    *(ptr_accumulated_dRdt_ + i_ncols + j) = -*(ptr_dRdt + i_ncols + j);
                }
                else{}
            }
        }
        ObjectExt_->checkSegment(accumulated_dRdt_, str_next_, groundPtsIdx_next_);
    }

    // cv::imshow("accumulated_dRdt", accumulated_dRdt_);
    // cv::waitKey(0);
    // exit(0);
    pcl::PointCloud<pcl::PointXYZ> pcl_dynamic;
    pcl::PointCloud<pcl::PointXYZ> pcl_static;
    float* ptr_next_img_x = str_next_->img_x.ptr<float>(0);
    float* ptr_next_img_y = str_next_->img_y.ptr<float>(0);
    float* ptr_next_img_z = str_next_->img_z.ptr<float>(0);

    // for (int i = 0; i < img_height_; ++i)
    // {
    //     int i_ncols = i * img_width_;
    //     for (int j = 0; j < img_width_; ++j)
    //     {
    //         if (*(ptr_accumulated_dRdt_ + i_ncols + j)!=0)
    //         {
    //             pcl_dynamic.push_back(
    //                 pcl::PointXYZ(*(ptr_next_img_x + i_ncols + j), *(ptr_next_img_y + i_ncols + j), *(ptr_next_img_z + i_ncols + j)));
    //         }
    //         else
    //         {
    //             pcl_static.push_back(
    //                 pcl::PointXYZ(*(ptr_next_img_x + i_ncols + j), *(ptr_next_img_y + i_ncols + j), *(ptr_next_img_z + i_ncols + j)));
    //         }
    //     }
    // }

    for (int i = 0; i < img_height_; ++i)
    {
        int i_ncols = i * img_width_;
        for (int j = 0; j < img_width_; ++j)
        {
            if (*(ptr_accumulated_dRdt_ + i_ncols + j)!=0)
            {
                if (str_next_->pts_per_pixel_index_valid[i_ncols + j].size() != 0)
                {
                    for (int k = 0; k < str_next_->pts_per_pixel_index_valid[i_ncols + j].size(); ++k)
                    {
                        pcl_dynamic.push_back(
                            pcl::PointXYZ(p1[str_next_->pts_per_pixel_index_valid[i_ncols + j][k]]));
                    }
                }
            }
            else
            {
                if (str_next_->pts_per_pixel_index_valid[i_ncols + j].size() != 0)
                {
                    for (int k = 0; k < str_next_->pts_per_pixel_index_valid[i_ncols + j].size(); ++k)
                    {
                        pcl_static.push_back(
                            pcl::PointXYZ(p1[str_next_->pts_per_pixel_index_valid[i_ncols + j][k]]));
                    }
                }
            }
        }
    }
    double dt_toc10 = timer::toc(); // milliseconds
    ROS_INFO_STREAM("elapsed time for 'segmentWholePts' :" <<  dt_toc10 << " [ms]");

    timer::tic();
    //// visualization ////
    // dynamic //
    sensor_msgs::PointCloud2 converted_msg_d;
    pcl::toROSMsg(pcl_dynamic, converted_msg_d);
    converted_msg_d.header.frame_id = "map";
    pub_dynamic_pts_.publish(converted_msg_d);

    sensor_msgs::PointCloud2 converted_msg_s;
    pcl::toROSMsg(pcl_static, converted_msg_s);
    converted_msg_s.header.frame_id = "map";
    pub_static_pts_.publish(converted_msg_s);
    // if (cnt_data == 3)
    // {exit(0);}
    double dt_toc11 = timer::toc(); // milliseconds
    ROS_INFO_STREAM("elapsed time for 'publish' :" <<  dt_toc11 << " [ms]");

    timer::tic();
    //// update for next iteration
    for (int i = 0; i < img_height_; ++i)
    {
        int i_ncols = i * img_width_;
        for (int j = 0; j < img_width_; ++j)
        {
            if (*(ptr_accumulated_dRdt_ + i_ncols + j)==0)
            {
                *(ptr_accumulated_dRdt_score_ + i_ncols + j) = 0;
            }
        }
    }

    copyStruct(str_next_, str_cur_, p1, p0, cnt_data);
    double dt_toc12 = timer::toc(); // milliseconds
    ROS_INFO_STREAM("elapsed time for 'copyRemove' :" <<  dt_toc12 << " [ms]");
};

// void MaplessDynamic::initializeStructAndPcl()
// {
//     {
//         str_warpPointcloud_->rho.resize(0);
//         str_warpPointcloud_->phi.resize(0);
//         str_warpPointcloud_->theta.resize(0);
//         str_warpPointcloud_->img_rho               = cv::Mat::zeros(img_height_, img_width_, CV_32FC1);
//         str_warpPointcloud_->img_index             = cv::Mat::zeros(img_height_, img_width_, CV_32SC1);
//         str_warpPointcloud_->img_x                 = cv::Mat::zeros(img_height_, img_width_, CV_32FC1);
//         str_warpPointcloud_->img_y                 = cv::Mat::zeros(img_height_, img_width_, CV_32FC1);
//         str_warpPointcloud_->img_z                 = cv::Mat::zeros(img_height_, img_width_, CV_32FC1);
//         str_warpPointcloud_->img_restore_mask      = cv::Mat::zeros(img_height_, img_width_, CV_32SC1);
//         str_warpPointcloud_->img_restore_warp_mask = cv::Mat::zeros(img_height_, img_width_, CV_32SC1);

//         for (int i=0; i<img_height_*img_width_; ++i)
//         {
//             str_warpPointcloud_->pts_per_pixel_n[i] = 0;
//         }

//         for (int i=0; i<img_height_*img_width_; ++i)
//         {
//             if (str_warpPointcloud_->pts_per_pixel_index[i].size() != 0)
//             {
//                 str_warpPointcloud_->pts_per_pixel_index[i].resize(0);
//             }
//         }

//         for (int i=0; i<img_height_*img_width_; ++i)
//         {
//             if (str_warpPointcloud_->pts_per_pixel_rho[i].size() != 0)
//             {
//                 str_warpPointcloud_->pts_per_pixel_rho[i].resize(0);
//             }
//         }

//         for (int i=0; i<img_height_*img_width_; ++i)
//         {
//             if (str_warpPointcloud_->pts_per_pixel_index_valid[i].size() != 0)
//             {
//                 str_warpPointcloud_->pts_per_pixel_index_valid[i].resize(0);
//             }
//         }
//     }

//     velo_xyz_.resize(0);
//     pts_warpewd_->resize(0);
// }

void MaplessDynamic::copyStruct(StrRhoPts* str_next, StrRhoPts* str_cur, pcl::PointCloud<pcl::PointXYZ>& p1 ,pcl::PointCloud<pcl::PointXYZ>& p0, int cnt_data)
{
    // memcpy(str_cur, str_next, sizeof(struct StrRhoPts));
    // CHK
    {   str_cur->pts = str_next->pts;
        str_cur->ptsInImage = str_next->ptsInImage;
        
        str_cur->rho.resize(0);
        str_cur->phi.resize(0);
        str_cur->theta.resize(0);
        std::copy(str_next->rho.begin(),str_next->rho.end(), str_cur->rho.begin());
        std::copy(str_next->phi.begin(),str_next->phi.end(), str_cur->phi.begin());
        std::copy(str_next->theta.begin(),str_next->theta.end(), str_cur->theta.begin());

        str_next->img_rho.copyTo(str_cur->img_rho);
        str_next->img_index.copyTo(str_cur->img_index);
        str_next->img_x.copyTo(str_cur->img_x);
        str_next->img_y.copyTo(str_cur->img_y);
        str_next->img_z.copyTo(str_cur->img_z);

        std::copy(str_next->pts_per_pixel_n.begin(), str_next->pts_per_pixel_n.end(), str_cur->pts_per_pixel_n.begin());

        for (int i=0; i<img_height_*img_width_; ++i)
        {
            if (str_cur->pts_per_pixel_index[i].size() != 0)
            {
                str_cur->pts_per_pixel_index[i].resize(0);
                std::copy(str_next->pts_per_pixel_index[i].begin(), str_next->pts_per_pixel_index[i].end(), str_cur->pts_per_pixel_index[i].begin());
            }
        }

        for (int i=0; i<img_height_*img_width_; ++i)
        {
            if (str_cur->pts_per_pixel_rho[i].size() != 0)
            {
                str_cur->pts_per_pixel_rho[i].resize(0);
                std::copy(str_next->pts_per_pixel_rho[i].begin(), str_next->pts_per_pixel_rho[i].end(), str_cur->pts_per_pixel_rho[i].begin());
            }
        }

        for (int i=0; i<img_height_*img_width_; ++i)
        {
            if (str_cur->pts_per_pixel_index_valid[i].size() != 0)
            {
                str_cur->pts_per_pixel_index_valid[i].resize(0);
                std::copy(str_next->pts_per_pixel_index_valid[i].begin(), str_next->pts_per_pixel_index_valid[i].end(), str_cur->pts_per_pixel_index_valid[i].begin());
            }
        }
        str_next->img_restore_mask.copyTo(str_cur->img_restore_mask);
        str_next->img_restore_warp_mask.copyTo(str_cur->img_restore_warp_mask);
    }


    // str_next->reset();
    {
        str_next->rho.resize(0);
        str_next->phi.resize(0);
        str_next->theta.resize(0);
        str_next->img_rho               = cv::Mat::zeros(img_height_, img_width_, CV_32FC1);
        str_next->img_index             = cv::Mat::zeros(img_height_, img_width_, CV_32SC1);
        str_next->img_x                 = cv::Mat::zeros(img_height_, img_width_, CV_32FC1);
        str_next->img_y                 = cv::Mat::zeros(img_height_, img_width_, CV_32FC1);
        str_next->img_z                 = cv::Mat::zeros(img_height_, img_width_, CV_32FC1);
        str_next->img_restore_mask      = cv::Mat::zeros(img_height_, img_width_, CV_32SC1);
        str_next->img_restore_warp_mask = cv::Mat::zeros(img_height_, img_width_, CV_32SC1);

        for (int i=0; i<img_height_*img_width_; ++i)
        {
            str_next->pts_per_pixel_n[i] = 0;
        }

        for (int i=0; i<img_height_*img_width_; ++i)
        {
            if (str_next->pts_per_pixel_index[i].size() != 0)
            {
                str_next->pts_per_pixel_index[i].resize(0);
            }
        }

        for (int i=0; i<img_height_*img_width_; ++i)
        {
            if (str_next->pts_per_pixel_rho[i].size() != 0)
            {
                str_next->pts_per_pixel_rho[i].resize(0);
            }
        }

        for (int i=0; i<img_height_*img_width_; ++i)
        {
            if (str_next->pts_per_pixel_index_valid[i].size() != 0)
            {
                str_next->pts_per_pixel_index_valid[i].resize(0);
            }
        }
    }

    // memcpy(&p0_pcl_test_, &p1, sizeof(pcl::PointCloud<pcl::PointXYZ>));
    // p0_pcl_test_ = p1;
    // if (cnt_data == 2)
    // {
    //     std::cout << p1.size() << std::endl;
    //     sensor_msgs::PointCloud2 converted_msg_d;
    //     pcl::toROSMsg(p1, converted_msg_d);
    //     converted_msg_d.header.frame_id = "map";
    //     pub_dynamic_pts_.publish(converted_msg_d);
    //     exit(0);
    // }

    p0_pcl_test_.resize(0);
    pcl::copyPointCloud(p1, p0_pcl_test_);
    p1.resize(0);

    // if (cnt_data == 2)
    // {
    //     std::cout << p0_pcl_test_.size() << std::endl;
    //     sensor_msgs::PointCloud2 converted_msg_s;
    //     pcl::toROSMsg(p0_pcl_test_, converted_msg_s);
    //     converted_msg_s.header.frame_id = "map";
    //     pub_static_pts_.publish(converted_msg_s);
        
    // }

    // str_cur_warped_->reset();
    {
        str_cur_warped_->rho.resize(0);
        str_cur_warped_->phi.resize(0);
        str_cur_warped_->theta.resize(0);
        str_cur_warped_->img_rho               = cv::Mat::zeros(img_height_, img_width_, CV_32FC1);
        str_cur_warped_->img_index             = cv::Mat::zeros(img_height_, img_width_, CV_32SC1);
        str_cur_warped_->img_x                 = cv::Mat::zeros(img_height_, img_width_, CV_32FC1);
        str_cur_warped_->img_y                 = cv::Mat::zeros(img_height_, img_width_, CV_32FC1);
        str_cur_warped_->img_z                 = cv::Mat::zeros(img_height_, img_width_, CV_32FC1);
        str_cur_warped_->img_restore_mask      = cv::Mat::zeros(img_height_, img_width_, CV_32SC1);
        str_cur_warped_->img_restore_warp_mask = cv::Mat::zeros(img_height_, img_width_, CV_32SC1);

        for (int i=0; i<img_height_*img_width_; ++i)
        {
            str_cur_warped_->pts_per_pixel_n[i] = 0;
        }

        for (int i=0; i<img_height_*img_width_; ++i)
        {
            if (str_cur_warped_->pts_per_pixel_index[i].size() != 0)
            {
                str_cur_warped_->pts_per_pixel_index[i].resize(0);
            }
        }

        for (int i=0; i<img_height_*img_width_; ++i)
        {
            if (str_cur_warped_->pts_per_pixel_rho[i].size() != 0)
            {
                str_cur_warped_->pts_per_pixel_rho[i].resize(0);
            }
        }

        for (int i=0; i<img_height_*img_width_; ++i)
        {
            if (str_cur_warped_->pts_per_pixel_index_valid[i].size() != 0)
            {
                str_cur_warped_->pts_per_pixel_index_valid[i].resize(0);
            }
        }
    }

    // str_warpPointcloud_->reset();
    {
        PclWarp_->str_warpPointcloud_->rho.resize(0);
        PclWarp_->str_warpPointcloud_->phi.resize(0);
        PclWarp_->str_warpPointcloud_->theta.resize(0);
        PclWarp_->str_warpPointcloud_->img_rho               = cv::Mat::zeros(img_height_, img_width_, CV_32FC1);
        PclWarp_->str_warpPointcloud_->img_index             = cv::Mat::zeros(img_height_, img_width_, CV_32SC1);
        PclWarp_->str_warpPointcloud_->img_x                 = cv::Mat::zeros(img_height_, img_width_, CV_32FC1);
        PclWarp_->str_warpPointcloud_->img_y                 = cv::Mat::zeros(img_height_, img_width_, CV_32FC1);
        PclWarp_->str_warpPointcloud_->img_z                 = cv::Mat::zeros(img_height_, img_width_, CV_32FC1);
        PclWarp_->str_warpPointcloud_->img_restore_mask      = cv::Mat::zeros(img_height_, img_width_, CV_32SC1);
        PclWarp_->str_warpPointcloud_->img_restore_warp_mask = cv::Mat::zeros(img_height_, img_width_, CV_32SC1);

        for (int i=0; i<img_height_*img_width_; ++i)
        {
            PclWarp_->str_warpPointcloud_->pts_per_pixel_n[i] = 0;
        }

        for (int i=0; i<img_height_*img_width_; ++i)
        {
            if (PclWarp_->str_warpPointcloud_->pts_per_pixel_index[i].size() != 0)
            {
                PclWarp_->str_warpPointcloud_->pts_per_pixel_index[i].resize(0);
            }
        }

        for (int i=0; i<img_height_*img_width_; ++i)
        {
            if (PclWarp_->str_warpPointcloud_->pts_per_pixel_rho[i].size() != 0)
            {
                PclWarp_->str_warpPointcloud_->pts_per_pixel_rho[i].resize(0);
            }
        }

        for (int i=0; i<img_height_*img_width_; ++i)
        {
            if (PclWarp_->str_warpPointcloud_->pts_per_pixel_index_valid[i].size() != 0)
            {
                PclWarp_->str_warpPointcloud_->pts_per_pixel_index_valid[i].resize(0);
            }
        }
    }
  
    groundPtsIdx_next_ = cv::Mat::zeros(img_height_, img_width_, CV_8UC1);

    velo_cur_.resize(0);
    ptr_cur_pts_warped_->resize(0);
    
    residual_ = cv::Mat::zeros(img_height_, img_width_, CV_32FC1);

    PclWarp_->velo_xyz_.resize(0);
    PclWarp_->pts_warpewd_->resize(0);

    // std::cout << ptr_cur_pts_warped_->width << std::endl;
    // std::cout << pts_warpewd_->width << std::endl;
    // exit(0);
}

void MaplessDynamic::getUserSettingParameters(){
    // IMPLEMENT YOUR CODE FROM THIS LINE.
    
    img_height_ = 64 / 1;
    img_width_ = 4500 / 5 + 1;
    // object_threshold_ = 30;

    // alpha_ = 0.3;
    // beta_ = 0.1;

    // initialize
    score_cnt_ = 0;

    // for test
    test_data_type_ = "KITTI";
    
    // END YOUR CODE   
};

void MaplessDynamic::calculateGTpose(int cnt_line)
{
    Pose T_tmp = Pose::Zero(4,4);
    std::vector<float> pose_tmp = all_pose_[cnt_line];
    // T_tmp.block<1,4>(0,0) << pose_tmp[0], pose_tmp[1], pose_tmp[2], pose_tmp[3];
    // T_tmp.block<1,4>(1,0) << pose_tmp[4], pose_tmp[5], pose_tmp[6], pose_tmp[7];
    // T_tmp.block<1,4>(2,0) << pose_tmp[8], pose_tmp[9], pose_tmp[10], pose_tmp[11];
}




//////////// TO READ KITTI BIN FILES ////////////

void MaplessDynamic::read_filelists(const std::string& dir_path,std::vector<std::string>& out_filelsits,std::string type)
{
    struct dirent *ptr;
    DIR *dir;
    dir = opendir(dir_path.c_str());
    out_filelsits.clear();
    while ((ptr = readdir(dir)) != NULL){
        std::string tmp_file = ptr->d_name;
        if (tmp_file[0] == '.')continue;
        if (type.size() <= 0){
            out_filelsits.push_back(ptr->d_name);
        }else{
            if (tmp_file.size() < type.size())continue;
            std::string tmp_cut_type = tmp_file.substr(tmp_file.size() - type.size(),type.size());
            if (tmp_cut_type == type){
                out_filelsits.push_back(ptr->d_name);
            }
        }
    }
}

bool computePairNum(std::string pair1,std::string pair2)
{
    return pair1 < pair2;
}

void MaplessDynamic::sort_filelists(std::vector<std::string>& filists,std::string type)
{
    if (filists.empty())return;

    std::sort(filists.begin(),filists.end(),computePairNum);
}

void MaplessDynamic::readKittiPclBinData(std::string &in_file, int file_num)
{   
    static int valid_cnt = 0;
    // load point cloud
    std::fstream input(in_file.c_str(), std::ios::in | std::ios::binary);
    if(!input.good()){
        std::cerr << "Could not read file: " << in_file << std::endl;
        exit(EXIT_FAILURE);
    }
    input.seekg(0, std::ios::beg);

    // pcl::PointCloud<pcl::PointXYZI>::Ptr points (new pcl::PointCloud<pcl::PointXYZI>);

    int i;
    for (i=0; input.good() && !input.eof(); ++i) {
        pcl::PointXYZI point;
        input.read((char *) &point.x, 3*sizeof(float));
        input.read((char *) &point.intensity, sizeof(float));
        data_buf_[valid_cnt]->pcl_->push_back(point);
    }
    data_buf_[valid_cnt]->pcl_->resize(i-1);
    pcl::toROSMsg(*data_buf_[valid_cnt]->pcl_, *(data_buf_[valid_cnt]->pcl_msg_));

    // std::cout<< "bin_num: "<<file_num <<" "<<"num_pts: "<<data_buf_[valid_cnt]->pcl_->size() <<std::endl;
    // std::cout<< "bin_num: "<<file_num <<" "<<"final pts: "<<data_buf_[file_num]->pcl_->at((data_buf_[file_num]->pcl_)->size()-2) <<std::endl;
    // std::cout<< "valid_num: "<<valid_cnt <<" "<<"num_pts: "<<data_buf_[valid_cnt]->pcl_msg_->width <<std::endl;
    input.close();
    valid_cnt+=1;
}

// void MaplessDynamic::warpPointcloud(StrRhoPts *str_cur, const Pose &T01, /*output*/ cv::Mat &mat_in, int cnt_data)
// {
//     int n_row = str_next_->img_rho.rows;
//     int n_col = str_next_->img_rho.cols;
//     float *ptr_mat_in = mat_in.ptr<float>(0);
//     float *ptr_img_x = str_cur->img_x.ptr<float>(0);
//     float *ptr_img_y = str_cur->img_y.ptr<float>(0);
//     float *ptr_img_z = str_cur->img_z.ptr<float>(0);
//     float *ptr_img_rho = str_cur->img_rho.ptr<float>(0);
    
//     cv::Mat mat_out = cv::Mat::zeros(img_height_, img_width_, CV_32FC1);
//     int *ptr_warp_img_index = str_warpPointcloud_->img_index.ptr<int>(0);
//     float *ptr_mat_out = mat_out.ptr<float>(0);

//     std::vector<float> I_vec1;
//     I_vec1.reserve(n_row * n_col);
//     bool isempty_flag = 0;

//     for (int i = 0; i < n_row; ++i)
//     {
//         int i_ncols = i * n_col;
//         for (int j = 0; j < n_col; ++j)
//         {            
//             if (*(ptr_mat_in + i_ncols + j) != 0 && *(ptr_img_rho + i_ncols + j) != 0)
//             {
//                 velo_xyz_.push_back(pcl::PointXYZ(*(ptr_img_x + i_ncols + j), *(ptr_img_y + i_ncols + j), *(ptr_img_z + i_ncols + j)));

//                 I_vec1.push_back(*(ptr_mat_in + i_ncols + j));
//                 isempty_flag = 1;
//             }
//         } //end for i
//     }     // end for j

//     if (isempty_flag == 0)
//     {
//         return;
//     }

//     pcl::transformPointCloud(velo_xyz_, *pts_warpewd_, T01);

//     CloudFrame_->genRangeImages(*pts_warpewd_, str_warpPointcloud_, 0);
//     // countZerofloat(str_warpPointcloud_->img_index);

//     // int cnt_debug = 0;

//     for (int i = 0; i < n_row; ++i)
//     {
//         int i_ncols = i * n_col;
//         for (int j = 0; j < n_col; ++j)
//         {            
//             if (*(ptr_warp_img_index + i_ncols + j) != 0)
//             {
//                 *(ptr_mat_out + i_ncols + j) = I_vec1[*(ptr_warp_img_index + i_ncols + j)];
//                 // std::cout<<cnt_debug << " "<< *(ptr_warp_img_index + i_ncols + j)<<std::endl;
//                 // cnt_debug += 1;
//             } // end if
//         } // end for j
//     } //end for i
//     // countZerofloat(mat_in);
//     // if (cnt_data==3)
//     // {
//     //     exit(0);
//     // }

//     // ptr_mat_out = ptr_mat_in;
//     mat_out.copyTo(mat_in);
// }

// void MaplessDynamic::filterOutAccumdR(StrRhoPts* str_next, StrRhoPts* str_cur_warped, cv::Mat& accumulated_dRdt, cv::Mat& accumulated_dRdt_score, cv::Mat& residual)
// {
//     int n_row = str_next->img_rho.rows;
//     int n_col = str_next->img_rho.cols;
//     float coef_accum_w[2] = {0.5, 0.9};
//     float* ptr_accumulated_dRdt = accumulated_dRdt.ptr<float>(0);
//     float* ptr_next_img_rho = str_next->img_rho.ptr<float>(0);
//     float* ptr_cur_warped_img_rho = str_cur_warped->img_rho.ptr<float>(0);
//     float* ptr_residual = residual.ptr<float>(0);

//     // Accumulate the occlusion
//     for (int i = 0; i<n_row; ++i)
//     {
//         int i_ncols = i * n_col;
//         for (int j=0; j<n_col; ++j)
//         {
//             if (*(ptr_next_img_rho + i_ncols + j) < 10.0)
//             {
//                 *(ptr_accumulated_dRdt + i_ncols + j) = coef_accum_w[0] * (*(ptr_accumulated_dRdt + i_ncols + j)) + *(ptr_residual + i_ncols + j);
//             }
//             else // >10
//             {
//                 *(ptr_accumulated_dRdt + i_ncols + j) = coef_accum_w[1] * (*(ptr_accumulated_dRdt + i_ncols + j)) + *(ptr_residual + i_ncols + j);
//             }            
//         }
//     }

//     // Extract candidate for objects
//     for (int i = 0; i<n_row; ++i)
//     {
//         int i_ncols = i * n_col;
//         for (int j = 0; j < n_col; ++j)
//         {
//             if ( (*(ptr_next_img_rho + i_ncols + j) > 40) 
//             || (*(ptr_accumulated_dRdt + i_ncols + j) < alpha_ * (*(ptr_next_img_rho + i_ncols + j)))
//             ||  (*(ptr_next_img_rho + i_ncols + j) == 0) 
//             || (*(ptr_cur_warped_img_rho + i_ncols + j) == 0)
//             || (*(ptr_residual + i_ncols + j) < (- beta_ * (*(ptr_next_img_rho + i_ncols + j)))) )
//             {
//                 *(ptr_accumulated_dRdt + i_ncols + j) = 0;
//             }
//         }
//     }
// }

// void MaplessDynamic::extractObjectCandidate(cv::Mat& accumulated_dRdt, StrRhoPts* str_next, int object_threshold)
// {
//     int n_col = accumulated_dRdt.cols;
//     int n_row = accumulated_dRdt.rows;
//     cv::Mat object_mask = cv::Mat::zeros(img_height_, img_width_, CV_8UC1);
//     uchar* ptr_object_mask = object_mask.ptr<uchar>(0);
//     float* ptr_accumulated_dRdt = accumulated_dRdt.ptr<float>(0);
//     int cnt = 0;
//     float* ptr_next_img_z = str_next->img_z.ptr<float>(0);
//     float* ptr_next_img_rho = str_next->img_rho.ptr<float>(0);

//     cv::Mat object_label = cv::Mat::zeros(img_height_, img_width_, CV_32SC1);

//     for (int i=0; i<n_row; ++i)
//     {
//         int i_ncols = i * n_col;
//         for (int j=0; j<n_col; ++j)
//         {
//             if (*(ptr_accumulated_dRdt + i_ncols + j) > 0)
//             {
//                 *(ptr_object_mask + i_ncols + j) = 255;
//                 cnt += 1;
//             }
//         }
//     }
//     int n_label = cv::connectedComponents(object_mask, object_label, 8);
//     int* ptr_object_label = object_label.ptr<int>(0);

//     if (n_label == 0)
//     {
//         return;
//     }

//     std::vector<int> object_row;
//     std::vector<int> object_col;
//     std::vector<float> object_rho_roi;
//     object_row.reserve(100000);
//     object_col.reserve(100000);
//     object_rho_roi.reserve(100000);

//     std::vector<float> max_his_object_rho_roi;
//     max_his_object_rho_roi.reserve(100000);

//     std::vector<int> disconti_row;
//     std::vector<int> disconti_col;
//     disconti_row.reserve(100000);
//     disconti_col.reserve(100000);

//     std::vector<int> diff_object_area_bw_disconti_row;
//     std::vector<int> diff_object_area_bw_disconti_col;
//     diff_object_area_bw_disconti_row.reserve(100000);
//     diff_object_area_bw_disconti_col.reserve(100000);

//     std::vector<float> diff_z;
//     diff_z.reserve(100000);

//     cv::MatND histogram;
//     for (int object_idx = 0; object_idx < n_label; ++object_idx)
//     {
//         if (object_idx==0) //background
//         {
//             continue;
//         }

//         object_row.resize(0);
//         object_col.resize(0);
//         object_rho_roi.resize(0);
//         max_his_object_rho_roi.resize(0);
//         disconti_row.resize(0);
//         disconti_col.resize(0);
//         diff_object_area_bw_disconti_row.resize(0);
//         diff_object_area_bw_disconti_col.resize(0);
//         diff_z.resize(0);
//         cv::Mat object_rho_mat = cv::Mat::zeros(img_height_, img_width_, CV_32FC1);
//         float *ptr_object_rho_mat = object_rho_mat.ptr<float>(0);

//         for (int i = 0; i < n_row; ++i)
//         {
//             int i_ncols = i * n_col;
//             for (int j = 0; j < n_col; ++j)
//             {
//                 if (*(ptr_object_label + i_ncols + j) == object_idx)
//                 {
//                     object_row.push_back(i);
//                     object_col.push_back(j);
//                     object_rho_roi.push_back(*(ptr_next_img_rho + i_ncols + j));
//                     *(ptr_object_rho_mat + i_ncols + j) = *(ptr_next_img_rho + i_ncols + j);
//                 }
//             }
//         }

//         if (object_row.size() < object_threshold)
//         {
//             for (int i = 0; i < object_row.size(); ++i)
//             {
//                 *(ptr_accumulated_dRdt + object_row[i] * n_col + object_col[i]) = 0.0;
//                 continue;
//             }
//         }
//         else
//         {   
//             std::cout << object_row.size() <<std::endl;
//             float his_range_max = *max_element(object_rho_roi.begin(), object_rho_roi.end());
//             float his_range_min = *min_element(object_rho_roi.begin(), object_rho_roi.end());
//             float his_range[] = {his_range_min, his_range_max};
//             const int* channel_numbers = {0};
//             const float* his_ranges= his_range;
//             int number_bins = 50;
//             // std::cout<<object_rho_roi.size()<<std::endl;
//             // std::cout<<his_range[0]<<std::endl;
//             // std::cout<<his_range[1]<<std::endl;
//             cv::calcHist(&object_rho_mat, 1, channel_numbers, cv::Mat(), histogram, 1, &number_bins, &his_ranges);

//             int max_n = 0;
//             int max_idx = 100;
//             for (int p = 0; p < number_bins; ++p)
//             {
//                 if (max_n < histogram.at<float>(p))
//                 {
//                     max_n = histogram.at<float>(p);
//                     max_idx = p;
//                 }
//             }
//             float his_interval = (his_range_max-his_range_min)/(float)number_bins;
//             float bin_range_min = his_range_min + (float)(max_idx)*his_interval;
//             float bin_range_max = his_range_min + (float)(max_idx+1)*his_interval;
//             float range_min = 0.0;
//             float range_max = 0.0;
//             float max_his_average = 0.0;

//             for (int p = 0; p<object_rho_roi.size(); ++p)
//             {
//                 if (object_rho_roi[p]>bin_range_min && object_rho_roi[p]<bin_range_max)
//                 {
//                     max_his_object_rho_roi.push_back(object_rho_roi[p]);
//                 }
//             }
//             for (int i=0; i<max_his_object_rho_roi.size(); ++i)
//             {
//                 max_his_average += max_his_object_rho_roi[i];
//             }
//             max_his_average = max_his_average/(float)max_his_object_rho_roi.size();

//             if ((max_his_average - 1.0) < 0.0)
//             {
//                 range_min = max_his_average;
//             }
//             else{
//                 range_min = max_his_average - 0.8;
//             }
//             range_max = max_his_average + 1.0;

//             for (int i = 0; i < object_row.size(); ++i)
//             {
//                 if((object_rho_roi[i]<range_min) || (object_rho_roi[i]>range_max))
//                 {
//                     disconti_row.push_back(object_row[i]);
//                     disconti_col.push_back(object_col[i]);
//                 }
//                 else
//                 {
//                     diff_object_area_bw_disconti_row.push_back(object_row[i]);
//                     diff_object_area_bw_disconti_col.push_back(object_col[i]);
//                 }
//             }
//             if((object_row.size()-disconti_row.size()) < object_threshold )
//             {
//                 for (int i = 0; i < object_row.size(); ++i)
//                 {
//                     *(ptr_accumulated_dRdt + object_row[i] * n_col + object_col[i]) = 0;
//                     continue;
//                 }
//             }
//             else
//             {
//                 for (int i = 0; i < disconti_row.size(); ++i)
//                 {
//                     *(ptr_accumulated_dRdt + disconti_row[i] * n_col + disconti_col[i]) = 0;
//                 }
//             }

//             for(int i=0; i<diff_object_area_bw_disconti_row.size(); ++i)
//             {
//                 if (*(ptr_next_img_z+diff_object_area_bw_disconti_row[i]*n_col+diff_object_area_bw_disconti_col[i])!=0)
//                 {
//                     diff_z.push_back(*(ptr_next_img_z + diff_object_area_bw_disconti_row[i] * n_col + diff_object_area_bw_disconti_col[i]));
//                 }
//             }

//             float mean_diff_z = 0.0;
//             for(int i=0; i<diff_z.size(); ++i)
//             {
//                 mean_diff_z += diff_z[i];
//             }
//             mean_diff_z = mean_diff_z/(float)diff_z.size();

//             float std_diff_z = 0.0;
//             for(int i=0; i<diff_z.size(); ++i)
//             {
//                 std_diff_z += (diff_z[i]-mean_diff_z) * (diff_z[i]-mean_diff_z);
//             }
//             std_diff_z = (1.0/((float)diff_z.size()-1.0)*std_diff_z);
//             if (std_diff_z < 0.07*0.07)
//             {
//                 for (int i = 0; i < object_row.size(); ++i)
//                 {
//                     *(ptr_accumulated_dRdt + object_row[i] * n_col + object_col[i]) = 0;
//                 }
//                 // for (int i = 0; i < diff_object_area_bw_disconti_row.size(); ++i)
//                 // {
//                 //     *(ptr_accumulated_dRdt + diff_object_area_bw_disconti_row[i] * n_col + diff_object_area_bw_disconti_col[i]) = 0;
//                 // }
//             }

//             // std::cout<<"   ====================    " <<std::endl;
//             // std::cout<<histogram<<std::endl;
//             // std::cout<<max_idx<<std::endl;
//             // std::cout<<bin_range_min<<" " << bin_range_max << std::endl;
//             // std::cout<<max_his_average<< std::endl;
//             // std::cout<<(float)max_his_object_rho_roi.size() << std::endl;


//         }
//                     // cv::imshow("accumulated_dRdt", accumulated_dRdt);
//             // cv::waitKey(0);
//     } // end for object_idx
//     // cv::imshow("accumulated_dRdt", accumulated_dRdt);
//     // cv::waitKey(0);
//     // exit(0);

// }

// void MaplessDynamic::checkSegment(cv::Mat& accumulated_dRdt, StrRhoPts* str_next, cv::Mat& groundPtsIdx_next)
// {
//     float weight_factor = 0.95;
//     int n_col = str_next->img_rho.cols;
//     int n_row = str_next->img_rho.rows;
//     std::vector<int> idx_row;
//     std::vector<int> idx_col;
//     std::vector<int> check;
//     idx_row.reserve(100000);
//     idx_col.reserve(100000);
//     check.reserve(100000);
//     float* ptr_accumulated_dRdt = accumulated_dRdt.ptr<float>(0);
//     float* ptr_img_rho = str_next->img_rho.ptr<float>(0);
//     uchar* ptr_groundPtsIdx_next = groundPtsIdx_next.ptr<uchar>(0);
//     bool isempty = true;

//     float phi = (float)360.0/(float)n_col;
//     int cnt = 0;
//     int row = 0;
//     int col = 0;
//     float R = 0.0;

//     float d1 = 0.0;
//     float d2 = 0.0;

//     cv::Mat roi_up = cv::Mat::zeros(img_height_, img_width_, CV_8UC1);
//     uchar* ptr_roi_up = roi_up.ptr<uchar>(0);

//     for (int i=0; i<n_row; ++i)
//     {
//         int i_ncols = i * n_col;
//         for (int j = 0; j < n_col; ++j)
//         {
//             if (*(ptr_accumulated_dRdt + i_ncols + j) != 0)
//             {
//                 // std::cout << accumulated_dRdt.at<float>(i,j) << std::endl;
//                 // std::cout<<i<<" " <<j<<std::endl;
//                 idx_row.push_back(i);
//                 idx_col.push_back(j);
//                 check.push_back(0);
//                 isempty = false;
//                 *(ptr_roi_up + i_ncols + j) = 1;
//             }
//         }
//     }

//     if (isempty==true)
//     {
//         return;
//     }

//     while(1)
//     {
//         if(check[cnt]==0)
//         {
//             row = idx_row[cnt];
//             col = idx_col[cnt];
//             R = *(ptr_img_rho + row * n_col + col);
//             int i = 0;
//             int j = 0;
//             int ii = 0;
//             int jj = 0;
//             for (int idx_cw = 1; idx_cw < 5; ++idx_cw)
//             {
//                 if (idx_cw == 1)
//                 {
//                     i = row - 1;
//                     j = col;
//                 }
//                 else if (idx_cw == 2)
//                 {
//                     i = row;
//                     j = col + 1;
//                 }
//                 else if (idx_cw == 3)
//                 {
//                     i = row + 1;
//                     j = col;
//                 }
//                 else if (idx_cw == 4)
//                 {
//                     i = row;
//                     j = col - 1;
//                 }

//                 if ( (i<0) || (i> n_row-1))
//                 {
//                     continue;
//                 }

//                 if ( (j<0) || (j> n_col-1))
//                 {
//                     continue;
//                 }
//                 int i_ncols_j = i * n_col + j;
//                 if ((*(ptr_roi_up + i_ncols_j) == 0) && (*(ptr_img_rho + i_ncols_j) > 0))
//                 {
//                     if (*(ptr_groundPtsIdx_next + i_ncols_j) == 255)
//                     {
//                         continue;
//                     }

//                     d1 = std::max(R, *(ptr_img_rho + i_ncols_j));
//                     d2 = std::min(R, *(ptr_img_rho + i_ncols_j));
//                     if (atan2f(d2 * sinf(phi * D2R), (d1 - d2 * cosf(phi * D2R))) > 10.0 * D2R)
//                     {
//                         idx_row.push_back(i);
//                         idx_col.push_back(j);
//                         check.push_back(0);
//                         *(ptr_roi_up + i_ncols_j) = 1;
//                         *(ptr_accumulated_dRdt + i_ncols_j) = weight_factor * *(ptr_accumulated_dRdt + row * n_col + col);
//                     }
//                 }
//                 else if ((*(ptr_roi_up + i_ncols_j) == 0) && (*(ptr_img_rho + i_ncols_j) == 0))
//                 {
//                     if (*(ptr_groundPtsIdx_next + i_ncols_j) == 255)
//                     {
//                         continue;
//                     }

//                     if (idx_cw == 1)
//                     {
//                         ii = row - 2;
//                         jj = col;
//                     }
//                     else if (idx_cw == 2)
//                     {
//                         ii = row;
//                         jj = col + 2;
//                     }
//                     else if (idx_cw == 3)
//                     {
//                         ii = row + 2;
//                         jj = col;
//                     }
//                     else if (idx_cw == 4)
//                     {
//                         ii = row;
//                         jj = col - 2;
//                     }
//                     if ((ii < 0) || (ii > n_row-1))
//                     {
//                         continue;
//                     }
//                     if ((jj < 0) || (jj > n_col-1))
//                     {
//                         continue;
//                     }
//                     int ii_ncols_jj = ii * n_col + jj;
//                     if (*(ptr_groundPtsIdx_next + ii_ncols_jj) == 255)
//                     {
//                         continue;
//                     }

//                     if (*(ptr_roi_up + ii_ncols_jj) == 1){}
//                     else
//                     {
//                         d1 = std::max(R, *(ptr_img_rho + ii_ncols_jj));
//                         d2 = std::min(R, *(ptr_img_rho + ii_ncols_jj));

//                         if (atan2f(d2 * sinf(2.0 * phi * D2R), (d1 - d2 * cosf(2.0 * phi * D2R))) > 10.0 * D2R)
//                         {
//                             idx_row.push_back(ii);
//                             idx_col.push_back(jj);

//                             check.push_back(0);
//                             *(ptr_roi_up + ii_ncols_jj) = 1;
//                             *(ptr_accumulated_dRdt + ii_ncols_jj) = weight_factor * (*(ptr_accumulated_dRdt + row * n_col + col));
//                             if (idx_cw == 1)
//                             {
//                                 *(ptr_accumulated_dRdt + (ii + 1) * n_col + jj) = weight_factor * (*(ptr_accumulated_dRdt + row * n_col + col));
//                             }
//                             else if (idx_cw == 2)
//                             {
//                                 *(ptr_accumulated_dRdt + ii * n_col + (jj - 1)) = weight_factor * (*(ptr_accumulated_dRdt + row * n_col + col));
//                             }
//                             else if (idx_cw == 3)
//                             {
//                                 *(ptr_accumulated_dRdt + (ii - 1) * n_col + jj) = weight_factor * (*(ptr_accumulated_dRdt + row * n_col + col));
//                             }
//                             else if (idx_cw == 4)
//                             {
//                                 *(ptr_accumulated_dRdt + ii * n_col + (jj + 1)) = weight_factor * (*(ptr_accumulated_dRdt + row * n_col + col));
//                             }
//                             else{}
//                         }
//                     }
//                 }
//             }

//             if (*(ptr_groundPtsIdx_next + row * n_col + col) == 255)
//             {
//                 *(ptr_roi_up + row * n_col + col) = 0;
//             }
//             else{}

//             check[cnt] = 1;
//         }

//         cnt += 1;
//         if (cnt == check.size())
//         {
//             break;
//         }
//             // std::cout << "checkSegment cnt: "<< cnt << std::endl;
//     }

//     // cv::imshow("accumulated_dRdt", accumulated_dRdt);
//     // cv::waitKey(0);
//     // exit(0);
// }

// void MaplessDynamic::updateScore(cv::Mat& accumulated_dRdt, cv::Mat& accumulated_dRdt_score)
// {
//     int n_row = accumulated_dRdt.rows;
//     int n_col = accumulated_dRdt.cols;
//     float* ptr_accumulated_dRdt = accumulated_dRdt.ptr<float>(0);
//     float* ptr_accumulated_dRdt_score = accumulated_dRdt_score.ptr<float>(0);
    
//     for (int i=0; i<n_row; ++i)
//     {
//         int i_ncols = i * n_col;
//         for (int j=0; j<n_col; ++j)
//         {
//             if (*(ptr_accumulated_dRdt + i_ncols + j) != 0)
//             {
//                 *(ptr_accumulated_dRdt_score + i_ncols + j) += 1;
//             }
//         }
//     }

//     for (int i=0; i<n_row; ++i)
//     {
//         int i_ncols = i * n_col;
//         for (int j=0; j<n_col; ++j)
//         {
//             if (*(ptr_accumulated_dRdt_score + i_ncols + j) > 2.0)
//             {
//                 *(ptr_accumulated_dRdt + i_ncols + j) *= 5.0;
//                 if (*(ptr_accumulated_dRdt + i_ncols + j) > 1e3)
//                 {
//                     *(ptr_accumulated_dRdt + i_ncols + j) = 1e3;
//                 }
//                 else{}
//             }
//             else{}
//         }
//     }
// }

void MaplessDynamic::plugImageZeroHoles(cv::Mat& accumulated_dRdt, cv::Mat& accumulated_dRdt_score, StrRhoPts* str_next, cv::Mat& groundPtsIdx_next, int object_threshold)
{
    int n_row = accumulated_dRdt.rows;
    int n_col = accumulated_dRdt.cols;
    float* ptr_accumulated_dRdt         = accumulated_dRdt.ptr<float>(0);
    float* ptr_accumulated_dRdt_score   = accumulated_dRdt_score.ptr<float>(0);
    cv::Mat dRdt_bin        = cv::Mat::zeros(img_height_,img_width_, CV_8UC1);
    cv::Mat dRdt_score_bin  = cv::Mat::zeros(img_height_,img_width_, CV_8UC1);
    uchar* ptr_dRdt_bin         = dRdt_bin.ptr<uchar>(0);
    uchar* ptr_dRdt_score_bin   = dRdt_score_bin.ptr<uchar>(0);

    float* ptr_img_rho = str_next->img_rho.ptr<float>(0);

    for (int i=0; i<n_row; ++i)
    {
        int i_ncols = i * n_col;
        for (int j=0; j<n_col; ++j)
        {            
            if (*(ptr_accumulated_dRdt + i_ncols + j) != 0)
            {
                *(ptr_dRdt_bin + i_ncols + j) = 255;
            }
        }
    }

    for (int i = 0; i < n_row; ++i)
    {
        int i_ncols = i * n_col;
        for (int j = 0; j < n_col; ++j)
        {
            if (*(ptr_accumulated_dRdt_score + i_ncols + j) != 0)
            {
                *(ptr_dRdt_score_bin + i_ncols + j) = 255;
            }
        }
    }
 

    // imfill 
    //invert dRdt_bin
    cv::Mat dRdt_bin_inv = cv::Mat::zeros(img_height_, img_width_, CV_8UC1);
    cv::bitwise_not(dRdt_bin, dRdt_bin_inv);
    uchar *ptr_dRdt_bin_inv = dRdt_bin_inv.ptr<uchar>(0);
    int n_row_minus1_n_col = (n_row - 1) * n_col;
    for (int j = 0; j < n_col; ++j)
    {
        *(ptr_dRdt_bin_inv + n_row_minus1_n_col + j) = 255;
    }
    cv::floodFill(dRdt_bin_inv, cv::Point(0,0), cv::Scalar(0));
    cv::Mat dRdt_bin_filled = (dRdt_bin | dRdt_bin_inv);
    interpAndfill_image(accumulated_dRdt, dRdt_bin_filled);

    cv::Mat dRdt_score_bin_inv = cv::Mat::zeros(img_height_, img_width_, CV_8UC1);
    cv::bitwise_not(dRdt_score_bin, dRdt_score_bin_inv);
    uchar *ptr_dRdt_score_bin_inv = dRdt_score_bin_inv.ptr<uchar>(0);
    for (int j = 0; j < n_col; ++j)
    {
        *(ptr_dRdt_score_bin_inv + n_row_minus1_n_col + j) = 255;
    }
    cv::floodFill(dRdt_score_bin_inv, cv::Point(0,0), cv::Scalar(0));
    cv::Mat dRdt_score_bin_filled = (dRdt_score_bin | dRdt_score_bin_inv);
    interpAndfill_image(accumulated_dRdt_score, dRdt_score_bin_filled);

    // cv::imshow("accumulated_dRdt", dRdt_bin_filled);
    // cv::waitKey(0);
    // exit(0);

    cv::Mat rho_zero_value = cv::Mat::zeros(img_height_, img_width_, CV_8UC1);
    uchar* ptr_rho_zero_value = rho_zero_value.ptr<uchar>(0);

    for (int i=1; i<n_row; ++i)
    {
        int i_ncols = i * n_col;
        for (int j=0; j<n_col; ++j)
        {
            if (*(ptr_img_rho + i_ncols + j) == 0)
            {
                *(ptr_rho_zero_value + i_ncols + j) = 255;
            }
        }
    }

    cv::Mat input_img_mask = cv::Mat::zeros(img_height_, img_width_, CV_8UC1);
    uchar* ptr_input_img_mask = input_img_mask.ptr<uchar>(0);

    for (int i=0; i<n_row; ++i)
    {
        int i_ncols = i * n_col;
        for (int j=0; j<n_col; ++j)
        {
            if (*(ptr_accumulated_dRdt + i_ncols + j) > 0)
            {
                *(ptr_input_img_mask + i_ncols + j) = 255;
            }
        }
    }

    cv::Mat input_img_tmp = accumulated_dRdt.clone();
    float* ptr_input_img_tmp = input_img_tmp.ptr<float>(0);

    // Label objects
    cv::Mat connect_input = (rho_zero_value | input_img_mask);
    cv::Mat object_label = cv::Mat::zeros(img_height_, img_width_, CV_32SC1);

    int n_label = cv::connectedComponents(connect_input, object_label, 8);
    int* ptr_object_label = object_label.ptr<int>(0);

    // std::vector<int> object_row;
    // std::vector<int> object_col;
    // std::vector<float> object_rho_roi;
    // object_row.reserve(100000);
    // object_col.reserve(100000);
    // object_rho_roi.reserve(100000);
    int* ptr_object_row = object_row.data();
    int* ptr_object_col = object_col.data();
    float* ptr_object_rho_roi = object_rho_roi.data();

    cv::Mat zero_candidate = cv::Mat::zeros(img_height_,img_width_, CV_8UC1);
    uchar *ptr_zero_candidate = zero_candidate.ptr<uchar>(0);

    // std::vector<int> filled_object_row;
    // std::vector<int> filled_object_col;
    // std::vector<float> filled_object_rho_roi;
    // filled_object_row.reserve(100000);
    // filled_object_col.reserve(100000);
    // filled_object_rho_roi.reserve(100000);
    int* ptr_filled_object_row = filled_object_row.data();
    int* ptr_filled_object_col = filled_object_col.data();
    float* ptr_filled_object_rho_roi = filled_object_rho_roi.data();


    // std::vector<float> max_his_filled_object_rho_roi;
    max_his_filled_object_rho_roi.reserve(100000);

    // std::vector<int> rho_zero_filled_value_row;
    // std::vector<int> rho_zero_filled_value_col;
    // std::vector<float> rho_zero_filled_value_rho_roi;
    // rho_zero_filled_value_row.reserve(100000);
    // rho_zero_filled_value_col.reserve(100000);
    // rho_zero_filled_value_rho_roi.reserve(100000);
    int* ptr_rho_zero_filled_value_row = rho_zero_filled_value_row.data();
    int* ptr_rho_zero_filled_value_col = rho_zero_filled_value_col.data();
    float* ptr_rho_zero_filled_value_rho_roi = rho_zero_filled_value_rho_roi.data();

    // std::vector<int> disconti_row;
    // std::vector<int> disconti_col;
    // disconti_row.reserve(100000);
    // disconti_col.reserve(100000);

    cv::Mat object_area = cv::Mat::zeros(img_height_, img_width_, CV_8UC1);
    uchar *ptr_object_area = object_area.ptr<uchar>(0);
    cv::Mat object_area_filled = cv::Mat::zeros(img_height_, img_width_, CV_8UC1);
    uchar *ptr_object_area_filled = object_area_filled.ptr<uchar>(0);
    cv::Mat filled_object_rho_mat = cv::Mat::zeros(img_height_, img_width_, CV_32FC1);
    float *ptr_filled_object_rho_mat = filled_object_rho_mat.ptr<float>(0);

    cv::Mat object_area_inv = cv::Mat::zeros(img_height_, img_width_, CV_8UC1);
    uchar *ptr_object_area_inv = object_area_inv.ptr<uchar>(0);
    
    cv::MatND histogram;
    for (int object_idx = 0; object_idx < n_label; ++object_idx)
    {
        if (object_idx==0) //0: background
        {
            continue;
        }

        object_row.resize(0);
        object_col.resize(0);
        object_rho_roi.resize(0);
        filled_object_row.resize(0);
        filled_object_col.resize(0);
        filled_object_rho_roi.resize(0);
        max_his_filled_object_rho_roi.resize(0);
        rho_zero_filled_value_row.resize(0);
        rho_zero_filled_value_col.resize(0);
        rho_zero_filled_value_rho_roi.resize(0);
        disconti_row.resize(0);
        disconti_col.resize(0);
        object_area             = cv::Mat::zeros(img_height_, img_width_, CV_8UC1);
        object_area_filled      = cv::Mat::zeros(img_height_, img_width_, CV_8UC1);
        filled_object_rho_mat   = cv::Mat::zeros(img_height_, img_width_, CV_32FC1);

        for (int i=0; i<n_row; ++i)
        {
            int i_ncols = i * n_col;
            for (int j=0; j<n_col; ++j)
            {
                if (*(ptr_object_label + i_ncols + j) == object_idx)
                {
                    object_row.push_back(i);
                    object_col.push_back(j);
                    object_rho_roi.push_back(*(ptr_img_rho + i_ncols + j));
                    *(ptr_object_area + i_ncols + j) = 255;
                }
            }
        }

        if (object_row.size() < object_threshold)
        {
            for (int i = 0; i < object_row.size(); ++i)
            {
                *(ptr_rho_zero_value + ptr_object_row[i] * n_col + ptr_object_col[i]) = 0;
                *(ptr_accumulated_dRdt + ptr_object_row[i] * n_col + ptr_object_col[i]) = 0.0;
            }
            continue;
        }
        else
        {
            float connect_zero_mean = 0.0;
            int n_connet_zero = 0;

            // cv::Mat object_area_inv = cv::Mat::zeros(img_height_, img_width_, CV_8UC1);
            cv::bitwise_not(object_area, object_area_inv);

            for (int j = 0; j < n_col; ++j)
            {
                *(ptr_object_area_inv + (img_height_-1) * n_col + j) = 255;
            }
            cv::floodFill(object_area_inv, cv::Point(0, 0), cv::Scalar(0));
            object_area_filled = (object_area | object_area_inv);

            for (int i=0; i<n_row; ++i)
            {
                int i_ncols = i * n_col;
                for (int j = 0; j < n_col; ++j)
                {
                    if (*(ptr_object_area_filled + i_ncols + j) != 0 && *(ptr_input_img_mask + i_ncols + j) != 0)
                    {
                        connect_zero_mean += *(ptr_accumulated_dRdt + i_ncols + j);
                        n_connet_zero += 1;
                    }
                }
            }
            if (n_connet_zero > 0)
            {
                connect_zero_mean /= (float)n_connet_zero;

                for (int i = 0; i < n_row; ++i)
                {
                    int i_ncols = i * n_col;
                    for (int j = 0; j < n_col; ++j)
                    {
                        if (*(ptr_object_area_filled + i_ncols + j) != 0 && *(ptr_input_img_mask + i_ncols + j) == 0)
                        {
                            *(ptr_accumulated_dRdt + i_ncols + j) = connect_zero_mean;
                        }
                    }
                }
            }
            else
            {
                // continue;
            }

            for (int i = 0; i < n_row; ++i)
            {
                int i_ncols = i * n_col;
                for (int j = 0; j < n_col; ++j)
                {
                    if (*(ptr_img_rho + i_ncols + j) != 0 && *(ptr_input_img_tmp + i_ncols + j) != 0 && *(ptr_object_area_filled + i_ncols + j) != 0)
                    {
                        filled_object_row.push_back(i);
                        filled_object_col.push_back(j);
                        filled_object_rho_roi.push_back(*(ptr_img_rho + i_ncols + j));
                        *(ptr_filled_object_rho_mat + i_ncols + j) = *(ptr_img_rho + i_ncols + j);
                    }
                    else{}
                }
            }

            if (filled_object_row.size()<1)
            {
                for (int i = 0; i < object_row.size(); ++i)
                {
                    *(ptr_accumulated_dRdt + ptr_object_row[i] * n_col + ptr_object_col[i]) = 0;
                }
                continue;
            }
            else{}

            float his_range_max = *max_element(filled_object_rho_roi.begin(), filled_object_rho_roi.end());
            float his_range_min = *min_element(filled_object_rho_roi.begin(), filled_object_rho_roi.end());
            float his_range[] = {his_range_min, his_range_max};
            const int* channel_numbers = {0};
            const float* his_ranges= his_range;
            int number_bins = 50;
            cv::calcHist(&filled_object_rho_mat, 1, channel_numbers, cv::Mat(), histogram, 1, &number_bins, &his_ranges);

            int max_n = 0;
            int max_idx = 100;
            for (int p = 0; p < number_bins; ++p)
            {
                if (max_n < histogram.at<float>(p))
                {
                    max_n = histogram.at<float>(p);
                    max_idx = p;
                }
            }
            float his_interval = (his_range_max-his_range_min)/(float)number_bins;
            float bin_range_min = his_range_min + (float)(max_idx)*his_interval;
            float bin_range_max = his_range_min + (float)(max_idx+1)*his_interval;
            float range_min = 0.0;
            float range_max = 0.0;

            for (int p = 0; p<filled_object_rho_roi.size(); ++p)
            {
                if (ptr_filled_object_rho_roi[p]>bin_range_min && ptr_filled_object_rho_roi[p]<bin_range_max)
                {
                    max_his_filled_object_rho_roi.push_back(ptr_filled_object_rho_roi[p]);
                }
            }
            float max_his_average = 0.0;
            for (int i=0; i<max_his_filled_object_rho_roi.size(); ++i)
            {
                max_his_average += max_his_filled_object_rho_roi[i];
            }
            max_his_average = max_his_average/(float)max_his_filled_object_rho_roi.size();

            if ((max_his_average - 10.0) < 0.0)
            {
                range_min = 0.0;
            }
            else{
                range_min = max_his_average - 10.0;
            }
            range_max = max_his_average + 10.0;

            //
            for (int i = 0; i < n_row; ++i)
            {
                int i_ncols = i * n_col;
                for (int j = 0; j < n_col; ++j)
                {
                    if (*(ptr_object_area_filled + i_ncols + j) != 0 && *(ptr_img_rho + i_ncols + j) != 0)
                    {
                        rho_zero_filled_value_row.push_back(i);
                        rho_zero_filled_value_col.push_back(j);
                        rho_zero_filled_value_rho_roi.push_back(*(ptr_img_rho + i_ncols + j));
                    }
                }
            }

            for (int i = 0; i < rho_zero_filled_value_row.size(); ++i)
            {
                if ((ptr_rho_zero_filled_value_rho_roi[i] < range_min) || (ptr_rho_zero_filled_value_rho_roi[i] > range_max))
                {
                    if (*(ptr_object_area_filled + ptr_rho_zero_filled_value_row[i] * n_col + ptr_rho_zero_filled_value_col[i]) != 0)
                    {
                        disconti_row.push_back(ptr_rho_zero_filled_value_row[i]);
                        disconti_col.push_back(ptr_rho_zero_filled_value_col[i]);
                    }
                }
            }

            for (int i = 0; i < disconti_row.size(); ++i)
            {
                *(ptr_accumulated_dRdt + disconti_row[i] * n_col + disconti_col[i]) = 0;
            }

        }
    } // end for object_idx
}

void MaplessDynamic::interpAndfill_image(cv::Mat& input_img, cv::Mat& filled_bin)
{
    int n_col = input_img.cols;
    int n_row = input_img.rows;
    std::vector<int> row;
    std::vector<int> col;
    row.reserve(100000);
    col.reserve(100000);

    float* ptr_input_img = input_img.ptr<float>(0);
    uchar* ptr_filled_bin = filled_bin.ptr<uchar>(0);

    cv::Mat input_img_cp = input_img.clone();
    float* ptr_input_img_cp = input_img_cp.ptr<float>(0);

    float left_dir = 0;
    float right_dir = 0;
    float up_dir = 0;
    float down_dir = 0;

    int cnt_left = 1;
    int cnt_right = 1;
    int cnt_up = 1;
    int cnt_down = 1;

    std::vector<float> four_dir(4);

    float min = 0.0 ;

    for (int i=0; i<n_row; ++i)
    {
        int i_ncols = i * n_col;
        for (int j=0; j<n_col; ++j)
        {
            if ((*(ptr_input_img + i_ncols + j) == 0) && (*(ptr_filled_bin + i_ncols + j) > 0))
            {
                row.push_back(i);
                col.push_back(j);
            }
        }
    }

    if (row.size()==0)
    {
        return;
    }

    for (int k=0; k<row.size(); ++k)
    {
        int i = row[k];
        int j = col[k];

        left_dir = 0.0;
        right_dir = 0.0;
        up_dir = 0.0;
        down_dir = 0.0;

        cnt_left = 1;
        cnt_right = 1;
        cnt_up = 1;
        cnt_down = 1;
        // left
        while (left_dir == 0.0)
        {
            if ((j - cnt_left) < 0)
            {
                break;
            }
            left_dir = *(ptr_input_img + i * n_col + (j - cnt_left));
            cnt_left += 1;
        } // end while
        // right
        while (right_dir == 0.0)
        {
            if ((j + cnt_right) > n_col - 1)
            {
                break;
            }
            right_dir = *(ptr_input_img + i * n_col + (j + cnt_right));
            cnt_right += 1;
        } // end while
        // up
        while (up_dir == 0.0)
        {
            if ((i - cnt_up) < 0)
            {
                break;
            }
            up_dir = *(ptr_input_img + (i - cnt_up) * n_col + j);
            cnt_up += 1;
        } // end while
        // down
        while (down_dir == 0.0)
        {
            if ((i + cnt_down) > n_row - 1)
            {
                break;
            }
            down_dir = *(ptr_input_img + (i + cnt_down) * n_col + j);
            cnt_down += 1;
        } // end while
        four_dir[0] = (left_dir);
        four_dir[1] = (right_dir);
        four_dir[2] = (up_dir);
        four_dir[3] = (down_dir);
        min = *min_element(four_dir.begin(), four_dir.end());
        *(ptr_input_img_cp + i * n_col + j) = min;
    }
    input_img_cp.copyTo(input_img);
}

void MaplessDynamic::countZerofloat(cv::Mat& input_mat)
{
    int n_row = input_mat.rows;
    int n_col = input_mat.cols;
    float* ptr_input_mat = input_mat.ptr<float>(0);
    int cnt = 0;
    for (int i = 0; i < n_row; ++i)
    {
        int i_ncols = i * n_col;
        for (int j = 0; j < n_col; ++j)
        {
            if (*(ptr_input_mat + i_ncols + j) != 0)
            {
                cnt += 1;
            }
        }
    }
    std::cout<<"# of non zero: "<<cnt <<std::endl;
}

void MaplessDynamic::countZeroint(cv::Mat& input_mat)
{
    int n_row = input_mat.rows;
    int n_col = input_mat.cols;
    int *ptr_input_mat = input_mat.ptr<int>(0);
    int cnt = 0;
    for (int i = 0; i < n_row; ++i)
    {
        int i_ncols = i * n_col;
        for (int j = 0; j < n_col; ++j)
        {
            if (*(ptr_input_mat + i_ncols + j) != 0)
            {
                cnt += 1;
            }
        }
    }
    std::cout << "# of non zero: " << cnt << std::endl;
}

void MaplessDynamic::countZerouchar(cv::Mat& input_mat)
{
    int n_row = input_mat.rows;
    int n_col = input_mat.cols;
    uchar *ptr_input_mat = input_mat.ptr<uchar>(0);
    int cnt = 0;
    for (int i = 0; i < n_row; ++i)
    {
        int i_ncols = i * n_col;
        for (int j = 0; j < n_col; ++j)
        {
            if (*(ptr_input_mat + i_ncols + j) != 0)
            {
                cnt += 1;
            }
        }
    }
    std::cout << "# of non zero: " << cnt << std::endl;
}