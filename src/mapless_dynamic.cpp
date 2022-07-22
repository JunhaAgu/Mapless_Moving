#include "mapless_dynamic.h"
#include <string>
#include <fstream> 


MaplessDynamic::MaplessDynamic(ros::NodeHandle& nh, bool test_flag)
: nh_(nh), test_flag_(test_flag), is_initialized_test_(false)
 {
    // constructor
    ROS_INFO_STREAM("MaplessDynamic - constructed.");
    // IMPLEMENT YOUR CODE FROM THIS LINE.

    pub_dynamic_pts_ = nh_.advertise<sensor_msgs::PointCloud2>("/dynamic_pts",1);
    pub_static_pts_  = nh_.advertise<sensor_msgs::PointCloud2>("/static_pts",1);

    this->getUserSettingParameters();

    // allocation for solver
    accumulated_dRdt_       = cv::Mat::zeros(img_height_, img_width_, CV_32FC1);
    accumulated_dRdt_score_ = cv::Mat::zeros(img_height_, img_width_, CV_32FC1);
    residual_               = cv::Mat::zeros(img_height_, img_width_, CV_32FC1);

    str_cur_            = new StrRhoPts();
    str_next_           = new StrRhoPts();
    str_cur_warped_     = new StrRhoPts();
    str_warpPointcloud_ = new StrRhoPts();

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

    str_warpPointcloud_->rho.reserve(500000);
    str_warpPointcloud_->phi.reserve(500000);
    str_warpPointcloud_->theta.reserve(500000);
    str_warpPointcloud_->img_rho   = cv::Mat::zeros(img_height_, img_width_, CV_32FC1);
    str_warpPointcloud_->img_index = cv::Mat::zeros(img_height_, img_width_, CV_32SC1);

    str_warpPointcloud_->img_x = cv::Mat::zeros(img_height_, img_width_, CV_32FC1);
    str_warpPointcloud_->img_y = cv::Mat::zeros(img_height_, img_width_, CV_32FC1);
    str_warpPointcloud_->img_z = cv::Mat::zeros(img_height_, img_width_, CV_32FC1);

    str_warpPointcloud_->pts_per_pixel_n.resize(img_height_ * img_width_);
    str_warpPointcloud_->pts_per_pixel_index.resize(img_height_ * img_width_);
    str_warpPointcloud_->pts_per_pixel_rho.resize(img_height_ * img_width_);
    str_warpPointcloud_->pts_per_pixel_index_valid.resize(img_height_ * img_width_);
    for (int i=0; i<img_height_*img_width_; ++i)
    {
        str_warpPointcloud_->pts_per_pixel_index[i].reserve(5000);
    }
    for (int i=0; i<img_height_*img_width_; ++i)
    {
        str_warpPointcloud_->pts_per_pixel_rho[i].reserve(5000);
    }
    for (int i=0; i<img_height_*img_width_; ++i)
    {
        str_warpPointcloud_->pts_per_pixel_index_valid[i].reserve(5000);
    }
    str_warpPointcloud_->pts        = boost::make_shared<pcl::PointCloud<pcl::PointXYZ>>();
    str_warpPointcloud_->ptsInImage = boost::make_shared<pcl::PointCloud<pcl::PointXYZ>>();
    str_warpPointcloud_->img_restore_mask = cv::Mat::zeros(img_height_, img_width_, CV_32SC1);
    str_warpPointcloud_->img_restore_warp_mask = cv::Mat::zeros(img_height_, img_width_, CV_32SC1);

    velo_cur_.reserve(500000);
    velo_xyz_.reserve(500000);
    ptr_cur_pts_warped_ = boost::make_shared<pcl::PointCloud<pcl::PointXYZ>>();
    pts_warpewd_        = boost::make_shared<pcl::PointCloud<pcl::PointXYZ>>();

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
    delete str_warpPointcloud_;

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
        
        this->genRangeImages(p0_pcl_test_, str_cur_);

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
    genRangeImages(p1, str_next_);
    double dt_toc1 = timer::toc(); // milliseconds
    ROS_INFO_STREAM("elapsed time for 'genRangeImages' :" << dt_toc1 << " [ms]");

    // Warp pcl represented in current frame to next frame
    T_next2cur_ = T01;
    
    // Segment ground
    timer::tic();
    fastsegmentGround(str_next_);
    double dt_toc2 = timer::toc(); // milliseconds
    ROS_INFO_STREAM("elapsed time for 'segmentSGround' :" << dt_toc2 << " [ms]");

    //// Occlusion accumulation ////
    // Compute the occlusion dRdt
    
    timer::tic();
    dR_warpPointcloud(str_next_, str_cur_, p0, T_next2cur_, cnt_data, str_cur_warped_, residual_);
    double dt_toc3 = timer::toc(); // milliseconds
    ROS_INFO_STREAM("elapsed time for 'dR_warpPointcloud' :" << dt_toc3 << " [ms]");
    // str_next_->state();

    timer::tic();
    // warp the occlusion accumulation map
    warpPointcloud(str_cur_, T_next2cur_, accumulated_dRdt_, cnt_data);
    initializeStructAndPcl();
    warpPointcloud(str_cur_, T_next2cur_, accumulated_dRdt_score_, cnt_data);
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
    filterOutAccumdR(str_next_, str_cur_warped_, accumulated_dRdt_, accumulated_dRdt_score_, residual_);
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
    extractObjectCandidate(accumulated_dRdt_, str_next_, object_threshold_);
    // double dt_extractObjectCandidate = timer::toc(); // milliseconds
    // ROS_INFO_STREAM("elapsed time for 'dt_extractObjectCandidate' :" << dt_extractObjectCandidate << " [ms]");
    double dt_toc6 = timer::toc(); // milliseconds
    ROS_INFO_STREAM("elapsed time for 'extractObjectCandidate' :" <<  dt_toc6 << " [ms]");

    //// update object_mask
    //object_mask = accumulated_dRdt>0;
    timer::tic();
    // Fast Segment
    checkSegment(accumulated_dRdt_, str_next_, groundPtsIdx_next_);
    double dt_toc7 = timer::toc(); // milliseconds
    ROS_INFO_STREAM("elapsed time for 'checkSegment' :" <<  dt_toc7 << " [ms]");

    //// update object_mask
    //object_mask = accumulated_dRdt>0;
    timer::tic();
    updateScore(accumulated_dRdt_, accumulated_dRdt_score_);
    double dt_toc8 = timer::toc(); // milliseconds
    ROS_INFO_STREAM("elapsed time for 'updateScore' :" <<  dt_toc8 << " [ms]");

    timer::tic();
    plugImageZeroHoles(accumulated_dRdt_, accumulated_dRdt_score_, str_next_, groundPtsIdx_next_, object_threshold_);
    double dt_toc9 = timer::toc(); // milliseconds
    ROS_INFO_STREAM("elapsed time for 'plugImageZeroHoles' :" <<  dt_toc9 << " [ms]");

    timer::tic();
    float* ptr_accumulated_dRdt_ = accumulated_dRdt_.ptr<float>(0);
    float* ptr_accumulated_dRdt_score_ = accumulated_dRdt_score_.ptr<float>(0);
    float *ptr_next_img_rho = str_next_->img_rho.ptr<float>(0);

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

void MaplessDynamic::initializeStructAndPcl()
{
    {
        str_warpPointcloud_->rho.resize(0);
        str_warpPointcloud_->phi.resize(0);
        str_warpPointcloud_->theta.resize(0);
        str_warpPointcloud_->img_rho               = cv::Mat::zeros(img_height_, img_width_, CV_32FC1);
        str_warpPointcloud_->img_index             = cv::Mat::zeros(img_height_, img_width_, CV_32SC1);
        str_warpPointcloud_->img_x                 = cv::Mat::zeros(img_height_, img_width_, CV_32FC1);
        str_warpPointcloud_->img_y                 = cv::Mat::zeros(img_height_, img_width_, CV_32FC1);
        str_warpPointcloud_->img_z                 = cv::Mat::zeros(img_height_, img_width_, CV_32FC1);
        str_warpPointcloud_->img_restore_mask      = cv::Mat::zeros(img_height_, img_width_, CV_32SC1);
        str_warpPointcloud_->img_restore_warp_mask = cv::Mat::zeros(img_height_, img_width_, CV_32SC1);

        for (int i=0; i<img_height_*img_width_; ++i)
        {
            str_warpPointcloud_->pts_per_pixel_n[i] = 0;
        }

        for (int i=0; i<img_height_*img_width_; ++i)
        {
            if (str_warpPointcloud_->pts_per_pixel_index[i].size() != 0)
            {
                str_warpPointcloud_->pts_per_pixel_index[i].resize(0);
            }
        }

        for (int i=0; i<img_height_*img_width_; ++i)
        {
            if (str_warpPointcloud_->pts_per_pixel_rho[i].size() != 0)
            {
                str_warpPointcloud_->pts_per_pixel_rho[i].resize(0);
            }
        }

        for (int i=0; i<img_height_*img_width_; ++i)
        {
            if (str_warpPointcloud_->pts_per_pixel_index_valid[i].size() != 0)
            {
                str_warpPointcloud_->pts_per_pixel_index_valid[i].resize(0);
            }
        }
    }

    velo_xyz_.resize(0);
    pts_warpewd_->resize(0);
}

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
        str_warpPointcloud_->rho.resize(0);
        str_warpPointcloud_->phi.resize(0);
        str_warpPointcloud_->theta.resize(0);
        str_warpPointcloud_->img_rho               = cv::Mat::zeros(img_height_, img_width_, CV_32FC1);
        str_warpPointcloud_->img_index             = cv::Mat::zeros(img_height_, img_width_, CV_32SC1);
        str_warpPointcloud_->img_x                 = cv::Mat::zeros(img_height_, img_width_, CV_32FC1);
        str_warpPointcloud_->img_y                 = cv::Mat::zeros(img_height_, img_width_, CV_32FC1);
        str_warpPointcloud_->img_z                 = cv::Mat::zeros(img_height_, img_width_, CV_32FC1);
        str_warpPointcloud_->img_restore_mask      = cv::Mat::zeros(img_height_, img_width_, CV_32SC1);
        str_warpPointcloud_->img_restore_warp_mask = cv::Mat::zeros(img_height_, img_width_, CV_32SC1);

        for (int i=0; i<img_height_*img_width_; ++i)
        {
            str_warpPointcloud_->pts_per_pixel_n[i] = 0;
        }

        for (int i=0; i<img_height_*img_width_; ++i)
        {
            if (str_warpPointcloud_->pts_per_pixel_index[i].size() != 0)
            {
                str_warpPointcloud_->pts_per_pixel_index[i].resize(0);
            }
        }

        for (int i=0; i<img_height_*img_width_; ++i)
        {
            if (str_warpPointcloud_->pts_per_pixel_rho[i].size() != 0)
            {
                str_warpPointcloud_->pts_per_pixel_rho[i].resize(0);
            }
        }

        for (int i=0; i<img_height_*img_width_; ++i)
        {
            if (str_warpPointcloud_->pts_per_pixel_index_valid[i].size() != 0)
            {
                str_warpPointcloud_->pts_per_pixel_index_valid[i].resize(0);
            }
        }
    }
  
    groundPtsIdx_next_ = cv::Mat::zeros(img_height_, img_width_, CV_8UC1);

    velo_cur_.resize(0);
    ptr_cur_pts_warped_->resize(0);
    
    residual_ = cv::Mat::zeros(img_height_, img_width_, CV_32FC1);

    velo_xyz_.resize(0);
    pts_warpewd_->resize(0);

    // std::cout << ptr_cur_pts_warped_->width << std::endl;
    // std::cout << pts_warpewd_->width << std::endl;
    // exit(0);
}

void MaplessDynamic::getUserSettingParameters(){
    // IMPLEMENT YOUR CODE FROM THIS LINE.

    img_height_ = 64 / 1;
    img_width_ = 4500 / 5 + 1;
    object_threshold_ = 30;

    alpha_ = 0.3;
    beta_ = 0.1;

    paramRANSAC_.iter = 50; //100;
    paramRANSAC_.thr = 0.1;
    paramRANSAC_.a_thr = 0.1;
    paramRANSAC_.b_thr[0] = -0.5;
    paramRANSAC_.b_thr[1] = -1.2;
    paramRANSAC_.min_inlier = 5;

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

void MaplessDynamic::genRangeImages(pcl::PointCloud<pcl::PointXYZ>& pcl_in, StrRhoPts* str_in)
{
    int h_factor = 5;
    int v_factor = 1;
    float azimuth_res = 0.08*(float)h_factor;

    // generate range images
    float az_step = 1.0f/azimuth_res; // 0.08 degrees step.
    int n_radial = 360*az_step+1;
    int n_ring = 64/v_factor;
    int n_pts = pcl_in.size();

    calcuateRho(pcl_in, str_in);
    makeRangeImageAndPtsPerPixel(str_in, n_pts, n_ring, n_radial, az_step);
    // countZerofloat(str_in->img_rho);
    interpRangeImage(str_in, n_ring, n_radial);
    // countZerofloat(str_in->img_rho);
    interpPts(pcl_in, str_in, n_ring, n_radial);
    // countZerofloat(str_in->img_rho);

    // int cnt1 = 0;
    // int cnt2 = 0;
    // int cnt3 = 0;
    // int cnt4 = 0;
    // float* ptr_img_rho = str_in->img_rho.ptr<float>(0);
    // float *ptr_x = str_in->img_x.ptr<float>(0);
    // float* ptr_y = str_in->img_y.ptr<float>(0);
    // float* ptr_z = str_in->img_z.ptr<float>(0);

    // for (int i = 0; i < n_ring; ++i)
    // {
    //     for (int j = 0; j < n_radial; ++j)
    //     {
    //         if (*(ptr_img_rho + i * n_radial + j) != 0)
    //         {cnt1 +=1;}
    //         if (*(ptr_x + i * n_radial + j) != 0)
    //         {cnt2 +=1;}
    //         if (*(ptr_y + i * n_radial + j) != 0)
    //         {cnt3 +=1;}
    //         if (*(ptr_z + i * n_radial + j) != 0)
    //         {cnt4 +=1;}
    //     }
    // }
    // std::cout << cnt1 <<std::endl;
    // std::cout << cnt2 <<std::endl;
    // std::cout << cnt3 <<std::endl;
    // std::cout << cnt4 <<std::endl;
    // exit(0);
}

void MaplessDynamic::calcuateRho(pcl::PointCloud<pcl::PointXYZ>& pcl_in, StrRhoPts* str_in)
{
    timer::tic();

    float twopi         = 2.0*M_PI;
    float offset_theta  = M_PI;

    int n_pts = pcl_in.size();
    float invrhocos = 0.0;
    float cospsi = 0.0;
    float sinpsi = 0.0;

    for (int i = 0; i < n_pts; ++i)
    {
        str_in->rho.push_back(NORM(pcl_in.points[i].x, pcl_in.points[i].y, pcl_in.points[i].z));
        str_in->phi.push_back(asin(pcl_in.points[i].z / str_in->rho[i]));
        invrhocos = (float)1.0 / (str_in->rho[i] * cos(str_in->phi[i]));

        cospsi = pcl_in.points[i].x * invrhocos;
        if (cospsi > 1)
        {
            cospsi = (float)1.0;
        }
        else if (cospsi < -1)
        {
            cospsi = -(float)1.0;
        }

        sinpsi = pcl_in.points[i].y * invrhocos;

        if (cospsi >= 0)
        {
            if (sinpsi >= 0) // 1 quadrant
            {
                str_in->theta.push_back(acos(cospsi) + offset_theta);
            }
            else // 4 quadrant
            {
                str_in->theta.push_back(twopi - acos(cospsi) + offset_theta);
            }
        }
        else
        {
            if (sinpsi >= 0) // 2 quadrant
            {
                str_in->theta.push_back(M_PI - acos(-cospsi) + offset_theta);
            }
            else // 3 quadrant
            {
                str_in->theta.push_back(M_PI + acos(-cospsi) + offset_theta);
            }
        }

        if (str_in->theta[i] >= twopi)
        {
            str_in->theta[i] = str_in->theta[i] - twopi;
        }
        // std::cout << str_in->rho[i] << " " << str_in->phi[i] << " " << str_in->theta[i]<< std::endl;
    }
    double dt_slam = timer::toc(); // milliseconds
    ROS_INFO_STREAM("elapsed time for 'calcuateRho' :" << dt_slam << " [ms]");
}

void MaplessDynamic::makeRangeImageAndPtsPerPixel(StrRhoPts *str_in, int n_pts, int n_ring, int n_radial, float az_step)
{
    int i_row = 0;
    int i_col = 0;
    float* ptr_img_rho = str_in->img_rho.ptr<float>(0);
    int* ptr_img_index = str_in->img_index.ptr<int>(0);
    int n_row = str_in->img_rho.rows;
    int n_col = str_in->img_rho.cols;
    
    for (int i = 0; i < n_pts; ++i)
    {   
        // std::cout << str_in->rho[i] << " " << str_in->phi[i] << " " << str_in->theta[i]<< std::endl;
        for (int kk = 0; kk < n_ring; ++kk)
        {
            if (v_angle_[kk] < (str_in->phi[i] * R2D))
            {
                i_row = kk;
                break;
            }
            else{}
            if (kk == (n_ring-1))
            {
                i_row = n_ring - 1;
            }
            else{}
        }
        i_col = roundf(str_in->theta[i]*az_step*R2D);

        if ( (i_row > n_ring-1) || (i_row < 0) )
        {
            continue;
        }
        
        int i_row_ncols_i_col = i_row * n_col + i_col;
        if (*(ptr_img_rho + i_row_ncols_i_col) == 0) //(str_in->img_rho.at<float>(i_row,i_col) == 0)
        {   
            *(ptr_img_rho + i_row_ncols_i_col) = str_in->rho[i];
            *(ptr_img_index + i_row_ncols_i_col) = i;
        }
        else if (*(ptr_img_rho + i_row_ncols_i_col) > str_in->rho[i])
        {
            *(ptr_img_rho + i_row_ncols_i_col) = str_in->rho[i];
            *(ptr_img_index + i_row_ncols_i_col) = i;
        }
        else{}

        str_in->pts_per_pixel_n[i_row_ncols_i_col] += 1;
        str_in->pts_per_pixel_index[i_row_ncols_i_col].push_back(i);
        str_in->pts_per_pixel_rho[i_row_ncols_i_col].push_back(str_in->rho[i]);
    } // end for

    // for(int a=0; a < str_in->pts_per_pixel_rho.size(); ++a)
    // {
    //     // std::cout << str_in->pts_per_pixel_n[a] << std::endl;
    //     for (int b =0; b< str_in->pts_per_pixel_rho[a].size(); ++b)
    //     {
    //         std::cout << str_in->pts_per_pixel_rho[a][b] << " " ;
    //     }
    //     std::cout << std::endl;
    // }
    // exit(0);
}

void MaplessDynamic::interpRangeImage(StrRhoPts* str_in, int n_ring, int n_radial)
{
    cv::Mat img_rho_new = str_in->img_rho.clone();
    float* ptr_img_rho_new = img_rho_new.ptr<float>(0);

    float* ptr_img_rho = str_in->img_rho.ptr<float>(0);
    int* ptr_img_index = str_in->img_index.ptr<int>(0);
    int* ptr_img_restore_mask = str_in->img_restore_mask.ptr<int>(0);
    int n_col = str_in->img_rho.cols;
    int n_row = str_in->img_rho.rows;

    for (int i = 27; i < 32; i++)
    {
        int i_ncols = i * n_col;
        for (int j = 0 + 2; j < (n_radial - 2); ++j)
        {            
            if (*(ptr_img_rho + i_ncols + j) == 0)
            {
                if ((*(ptr_img_rho + (i - 1) * n_col + j) != 0) && (*(ptr_img_rho + (i + 1) * n_col + j) != 0))
                {
                    if (fabsf32(*(ptr_img_rho + (i - 1) * n_col + j) - *(ptr_img_rho + (i + 1) * n_col + j)) < 0.1)
                    {
                        *(ptr_img_restore_mask + i_ncols + j) = 1;
                        *(ptr_img_rho_new + i_ncols + j) = (*(ptr_img_rho + (i - 1) * n_col + j) + *(ptr_img_rho + (i + 1) * n_col + j)) * 0.5;
                    }
                    else
                    {
                        *(ptr_img_restore_mask + i_ncols + j) = 10;
                        *(ptr_img_rho_new + i_ncols + j) = std::max(*(ptr_img_rho + (i - 1) * n_col + j), *(ptr_img_rho + (i + 1) * n_col + j));
                    }
                }
                else if ((*(ptr_img_rho + (i - 1) * n_col + j) != 0) && (*(ptr_img_rho + (i + 2) * n_col + j) != 0))
                {
                    if (fabsf32(*(ptr_img_rho + (i - 1) * n_col + j) - *(ptr_img_rho + (i + 2) * n_col + j)) < 0.1)
                    {
                        *(ptr_img_restore_mask + i_ncols + j) = 2;
                        *(ptr_img_restore_mask + (i + 1) * n_col + j) = 3;
                        *(ptr_img_rho_new + i_ncols + j) = *(ptr_img_rho + (i - 1) * n_col + j)*(0.6666667) + *(ptr_img_rho + (i + 2) * n_col + j)*(0.3333333);
                        *(ptr_img_rho_new + (i+1) * n_col + j) = *(ptr_img_rho + (i - 1) * n_col + j)*(0.3333333) + *(ptr_img_rho + (i + 2) * n_col + j)*(0.6666667);
                    }
                    else
                    {
                        *(ptr_img_restore_mask + i_ncols + j) = 20;
                        *(ptr_img_restore_mask + (i + 1) * n_col + j) = 30;
                        *(ptr_img_rho_new + i_ncols + j) = std::max(*(ptr_img_rho + (i - 1) * n_col + j), *(ptr_img_rho + (i + 2) * n_col + j));
                        *(ptr_img_rho_new + (i+1) * n_col + j) = std::max(*(ptr_img_rho + (i - 1) * n_col + j), *(ptr_img_rho + (i + 2) * n_col + j));
                    }
                }
                else{}
            } // end if
            else{}

            if ((*(ptr_img_rho + i_ncols + (j - 1)) != 0) && (*(ptr_img_rho + i_ncols + (j + 1)) != 0))
            {
                if (fabsf32(*(ptr_img_rho + i_ncols + (j - 1)) - *(ptr_img_rho + i_ncols + (j + 1))) < 0.05)
                {
                    *(ptr_img_restore_mask + i_ncols + j) = 4;
                    *(ptr_img_rho_new + i_ncols + j) = (*(ptr_img_rho + i_ncols + (j - 1)) + *(ptr_img_rho + i_ncols + (j + 1))) * 0.5;
                }
                else{}
            }
            else if ((*(ptr_img_rho + i_ncols + (j - 1)) != 0) && (*(ptr_img_rho + i_ncols + (j + 2)) != 0))
            {
                if (fabsf32(*(ptr_img_rho + i_ncols + (j - 1)) - *(ptr_img_rho + i_ncols + (j + 2))) < 0.05)
                {
                    *(ptr_img_restore_mask + i_ncols + j) = 5;
                    *(ptr_img_restore_mask + i_ncols + (j + 1)) = 6;
                    *(ptr_img_rho_new + i_ncols + j) = *(ptr_img_rho + i_ncols + (j - 1)) * (0.6666667) + *(ptr_img_rho + i_ncols + (j + 2)) * (0.3333333);
                    *(ptr_img_rho_new + i_ncols + (j + 1)) = *(ptr_img_rho + i_ncols + (j - 1)) * (0.3333333) + *(ptr_img_rho + i_ncols + (j + 2)) * (0.6666667);
                }
                else{}
            }
            else{}

        } // end col

    } // end row

    img_rho_new.copyTo(str_in->img_rho);
}

void MaplessDynamic::interpPts(pcl::PointCloud<pcl::PointXYZ>& pcl_in, StrRhoPts* str_in, int n_ring, int n_radial)
{
    int n_col = str_in->img_rho.cols;
    int n_row = str_in->img_rho.rows;
    
    float* ptr_img_rho = str_in->img_rho.ptr<float>(0);
    int* ptr_img_index = str_in->img_index.ptr<int>(0);
    float* ptr_img_x = str_in->img_x.ptr<float>(0);
    float* ptr_img_y = str_in->img_y.ptr<float>(0);
    float* ptr_img_z = str_in->img_z.ptr<float>(0);
    int* ptr_img_restore_mask = str_in->img_restore_mask.ptr<int>(0);

    for (int i = 0; i < n_ring; ++i)
    {
        int i_ncols = i * n_col;
        for (int j = 0; j < n_radial; ++j)
        {           
            if (str_in->pts_per_pixel_rho[i_ncols + j].size() > 0)
            {
                
                for (int k = 0; k < (str_in->pts_per_pixel_rho[i_ncols + j].size()); ++k)
                {
                    if (std::abs((str_in->pts_per_pixel_rho[i_ncols + j][k] - *(ptr_img_rho + i_ncols + j))) < 0.5)
                    {
                        str_in->pts_per_pixel_index_valid[i_ncols + j].push_back(str_in->pts_per_pixel_index[i_ncols + j][k]);
                    }
                    else{}
                }
            } // end if
            else{}

            if (*(ptr_img_index + i_ncols + j) != 0)
            {
                *(ptr_img_x + i_ncols + j) = pcl_in[*(ptr_img_index + i_ncols + j)].x;
                *(ptr_img_y + i_ncols + j) = pcl_in[*(ptr_img_index + i_ncols + j)].y;
                *(ptr_img_z + i_ncols + j) = pcl_in[*(ptr_img_index + i_ncols + j)].z;
            }
            else{}

            if (*(ptr_img_restore_mask + i_ncols + j) == 1)
            {
                *(ptr_img_x + i_ncols + j) = 0.5 * (pcl_in[*(ptr_img_index + (i - 1) * n_col + j)].x + pcl_in[*(ptr_img_index + (i + 1) * n_col + j)].x);
                *(ptr_img_y + i_ncols + j) = 0.5 * (pcl_in[*(ptr_img_index + (i - 1) * n_col + j)].y + pcl_in[*(ptr_img_index + (i + 1) * n_col + j)].y);
                *(ptr_img_z + i_ncols + j) = 0.5 * (pcl_in[*(ptr_img_index + (i - 1) * n_col + j)].z + pcl_in[*(ptr_img_index + (i + 1) * n_col + j)].z);
            }
            else if (*(ptr_img_restore_mask + i_ncols + j) == 10)
            {
                if ((*(ptr_img_rho + (i - 1) * n_col + j) > *(ptr_img_rho + (i + 1) * n_col + j)))
                {
                    *(ptr_img_x + i_ncols + j) = pcl_in[*(ptr_img_index + (i-1) * n_col + j)].x;
                    *(ptr_img_y + i_ncols + j) = pcl_in[*(ptr_img_index + (i-1) * n_col + j)].y;
                    *(ptr_img_z + i_ncols + j) = pcl_in[*(ptr_img_index + (i-1) * n_col + j)].z;
                }
                else
                {
                    *(ptr_img_x + i_ncols + j) = pcl_in[*(ptr_img_index + (i+1) * n_col + j)].x;
                    *(ptr_img_y + i_ncols + j) = pcl_in[*(ptr_img_index + (i+1) * n_col + j)].y;
                    *(ptr_img_z + i_ncols + j) = pcl_in[*(ptr_img_index + (i+1) * n_col + j)].z;
                }
            }
            else if (*(ptr_img_restore_mask + i_ncols + j) == 2)
            {
                *(ptr_img_x + i_ncols + j) = (0.6666667) * pcl_in[*(ptr_img_index + (i - 1) * n_col + j)].x + (0.3333333) * pcl_in[*(ptr_img_index + (i + 2) * n_col + j)].x;
                *(ptr_img_y + i_ncols + j) = (0.6666667) * pcl_in[*(ptr_img_index + (i - 1) * n_col + j)].y + (0.3333333) * pcl_in[*(ptr_img_index + (i + 2) * n_col + j)].y;
                *(ptr_img_z + i_ncols + j) = (0.6666667) * pcl_in[*(ptr_img_index + (i - 1) * n_col + j)].z + (0.3333333) * pcl_in[*(ptr_img_index + (i + 2) * n_col + j)].z;
            }
            else if (*(ptr_img_restore_mask + i_ncols + j) == 20)
            {
                if ((*(ptr_img_rho + (i - 1) * n_col + j) > *(ptr_img_rho + (i + 2) * n_col + j)))
                {
                    *(ptr_img_x + i_ncols + j) = pcl_in[*(ptr_img_index + (i-1) * n_col + j)].x;
                    *(ptr_img_y + i_ncols + j) = pcl_in[*(ptr_img_index + (i-1) * n_col + j)].y;
                    *(ptr_img_z + i_ncols + j) = pcl_in[*(ptr_img_index + (i-1) * n_col + j)].z;
                }
                else
                {
                    *(ptr_img_x + i_ncols + j) = pcl_in[*(ptr_img_index + (i+2) * n_col + j)].x;
                    *(ptr_img_y + i_ncols + j) = pcl_in[*(ptr_img_index + (i+2) * n_col + j)].y;
                    *(ptr_img_z + i_ncols + j) = pcl_in[*(ptr_img_index + (i+2) * n_col + j)].z;
                }
            }
            else if (*(ptr_img_restore_mask + i_ncols + j) == 3)
            {
                *(ptr_img_x + i_ncols + j) = (0.3333333) * pcl_in[*(ptr_img_index + (i - 2) * n_col + j)].x + (0.6666667) * pcl_in[*(ptr_img_index + (i + 1) * n_col + j)].x;
                *(ptr_img_y + i_ncols + j) = (0.3333333) * pcl_in[*(ptr_img_index + (i - 2) * n_col + j)].y + (0.6666667) * pcl_in[*(ptr_img_index + (i + 1) * n_col + j)].y;
                *(ptr_img_z + i_ncols + j) = (0.3333333) * pcl_in[*(ptr_img_index + (i - 2) * n_col + j)].z + (0.6666667) * pcl_in[*(ptr_img_index + (i + 1) * n_col + j)].z;
            }
            else if (*(ptr_img_restore_mask + i_ncols + j) == 30)
            {
                if ((*(ptr_img_rho + (i - 2) * n_col + j) > *(ptr_img_rho + (i + 1) * n_col + j)))
                {
                    *(ptr_img_x + i_ncols + j) = pcl_in[*(ptr_img_index + (i-2) * n_col + j)].x;
                    *(ptr_img_y + i_ncols + j) = pcl_in[*(ptr_img_index + (i-2) * n_col + j)].y;
                    *(ptr_img_z + i_ncols + j) = pcl_in[*(ptr_img_index + (i-2) * n_col + j)].z;
                }
                else
                {
                    *(ptr_img_x + i_ncols + j) = pcl_in[*(ptr_img_index + (i+1) * n_col + j)].x;
                    *(ptr_img_y + i_ncols + j) = pcl_in[*(ptr_img_index + (i+1) * n_col + j)].y;
                    *(ptr_img_z + i_ncols + j) = pcl_in[*(ptr_img_index + (i+1) * n_col + j)].z;
                }
            }
            else if (*(ptr_img_restore_mask + i_ncols + j) == 4)
            {
                *(ptr_img_x + i_ncols + j) = 0.5 * (pcl_in[*(ptr_img_index + i_ncols + (j-1))].x + pcl_in[*(ptr_img_index + i_ncols + (j+1))].x);
                *(ptr_img_y + i_ncols + j) = 0.5 * (pcl_in[*(ptr_img_index + i_ncols + (j-1))].y + pcl_in[*(ptr_img_index + i_ncols + (j+1))].y);
                *(ptr_img_z + i_ncols + j) = 0.5 * (pcl_in[*(ptr_img_index + i_ncols + (j-1))].z + pcl_in[*(ptr_img_index + i_ncols + (j+1))].z);
            }
            else if (*(ptr_img_restore_mask + i_ncols + j) == 5)
            {
                *(ptr_img_x + i_ncols + j) = (0.6666667) * pcl_in[*(ptr_img_index + i_ncols + (j-1))].x + (0.3333333)*pcl_in[*(ptr_img_index + i_ncols + (j+2))].x;
                *(ptr_img_y + i_ncols + j) = (0.6666667) * pcl_in[*(ptr_img_index + i_ncols + (j-1))].y + (0.3333333)*pcl_in[*(ptr_img_index + i_ncols + (j+2))].y;
                *(ptr_img_z + i_ncols + j) = (0.6666667) * pcl_in[*(ptr_img_index + i_ncols + (j-1))].z + (0.3333333)*pcl_in[*(ptr_img_index + i_ncols + (j+2))].z;
            }
            else if (*(ptr_img_restore_mask + i_ncols + j) == 6)
            {
                *(ptr_img_x + i_ncols + j) = (0.3333333) * pcl_in[*(ptr_img_index + i_ncols + (j-2))].x + (0.6666667)*pcl_in[*(ptr_img_index + i_ncols + (j+1))].x;
                *(ptr_img_y + i_ncols + j) = (0.3333333) * pcl_in[*(ptr_img_index + i_ncols + (j-2))].y + (0.6666667)*pcl_in[*(ptr_img_index + i_ncols + (j+1))].y;
                *(ptr_img_z + i_ncols + j) = (0.3333333) * pcl_in[*(ptr_img_index + i_ncols + (j-2))].z + (0.6666667)*pcl_in[*(ptr_img_index + i_ncols + (j+1))].z;
            }
            else{}
        }     // end for j
    }         // end for i
}

void MaplessDynamic::dR_warpPointcloud(StrRhoPts* str_next, StrRhoPts* str_cur, pcl::PointCloud<pcl::PointXYZ>& p0, Pose& T01, int cnt_data, StrRhoPts* str_cur_warped, cv::Mat& dRdt)
{
    int n_row = str_next->img_rho.rows;
    int n_col = str_next->img_rho.cols;
    int n_ring = n_row;
    int n_radial = n_col;
    float* ptr_cur_img_x = str_cur->img_x.ptr<float>(0);
    float* ptr_cur_img_y = str_cur->img_y.ptr<float>(0);
    float* ptr_cur_img_z = str_cur->img_z.ptr<float>(0);
    // int cnt = 0;

    // representative pts
    for (int i = 0; i < n_row; ++i)
    {
        int i_ncols = i * n_col;
        for (int j = 0; j < n_col; ++j)
        {
            if ((*(ptr_cur_img_x + i_ncols + j) > 10.0) || (*(ptr_cur_img_x + i_ncols + j) < -10.0) || (*(ptr_cur_img_y + i_ncols + j) > 10.0) || (*(ptr_cur_img_y + i_ncols + j) < -10.0))
            {
                velo_cur_.push_back(pcl::PointXYZ(*(ptr_cur_img_x + i_ncols + j), *(ptr_cur_img_y + i_ncols + j), *(ptr_cur_img_z + i_ncols + j)));
            }
        }
    }
    

    // Far pts are warped by the original pts
    for (int i = 0; i < p0.size(); ++i)
    {
        if( (p0[i].x < 10) && (p0[i].x > -10.0) && (p0[i].y < 10.0) && (p0[i].y > -10.0) )
        {
            velo_cur_.push_back(p0[i]);
        }
    }

    // compensate zero in current rho image for warping
    // timer::tic();
    compensateCurRhoZeroWarp(str_cur, n_ring, n_radial, v_angle_, velo_cur_);
    // double dt_slam = timer::toc(); // milliseconds
    // ROS_INFO_STREAM("elapsed time for 'compensateCurRhoZeroWarp' :" << dt_slam << " [ms]");
    // exit(0);
    pcl::transformPointCloud(velo_cur_, *ptr_cur_pts_warped_, T01);
    
// if (cnt_data == 3)
    // {
        // std::cout << velo_cur_.size()<<std::endl;
        // sensor_msgs::PointCloud2 converted_msg_d;
        // pcl::toROSMsg(velo_cur_, converted_msg_d);
        // converted_msg_d.header.frame_id = "map";
        // pub_dynamic_pts_.publish(converted_msg_d);

        // sensor_msgs::PointCloud2 converted_msg_s;
        // pcl::toROSMsg(*ptr_cur_pts_warped_, converted_msg_s);
        // converted_msg_s.header.frame_id = "map";
        // pub_static_pts_.publish(converted_msg_s);
        // exit(0);
    // }

    // current warped image
    genRangeImages(*ptr_cur_pts_warped_, str_cur_warped);
    // countZerofloat(str_cur_warped->img_rho);
    // fill range image using interpolation
    interpRangeImageMin(str_cur_warped, n_ring, n_radial);
    // countZerofloat(str_cur_warped->img_rho);
    // fill pts corresponding to filled range image (no affect the original pts)
    interpPtsWarp(str_cur_warped, n_ring, n_radial);

    // calculate occlusions
    cv::subtract(str_cur_warped->img_rho, str_next->img_rho, dRdt); 
    // residual_ = str_cur_warped_->img_rho - str_next_->img_rho;    
}

void MaplessDynamic::compensateCurRhoZeroWarp(StrRhoPts* str_cur, int n_ring, int n_radial, std::vector<float>& v_angle, pcl::PointCloud<pcl::PointXYZ>& velo_cur)
{
    int n_row = n_ring;
    int n_col = n_radial;
    float left_dir_rho  = 0;
    float right_dir_rho = 0;
    float up_dir_rho    = 0;
    float down_dir_rho  = 0;
    float* ptr_cur_img_rho = str_cur->img_rho.ptr<float>(0);

    int cnt_left    = 1;
    int cnt_right   = 1;
    int cnt_up      = 1;
    int cnt_down    = 1;

    float min = 0.0 ;
    int min_index = 0;

    std::vector<float> four_dir;
    four_dir.resize(4);

    float new_phi = 0.0;
    float new_theta = 0.0;

    for (int i = 0 + 1; i < n_ring - 1; ++i)
    {
        int i_ncols = i * n_col;
        for (int j = 0 + 1; j < n_radial - 1; ++j)
        {
            // initialization every iteration
            left_dir_rho = 0.0;
            right_dir_rho = 0.0;
            up_dir_rho = 0.0;
            down_dir_rho = 0.0;
            
            if (*(ptr_cur_img_rho + i_ncols + j) == 0)
            {
                    cnt_left = 1;
                    cnt_right = 1;
                    cnt_up = 1;
                    cnt_down = 1;
                    //left
                    while (left_dir_rho == 0.0)
                    {
                        if ((j - cnt_left) < 0)
                        {
                            left_dir_rho = 100.0;
                            break;
                        }
                        left_dir_rho = *(ptr_cur_img_rho + i_ncols + (j - cnt_left));
                        cnt_left += 1;
                    } // end while
                    //right
                    while (right_dir_rho == 0.0)
                    {
                        if ((j + cnt_right) > n_col-1)
                        {
                            right_dir_rho = 100.0;
                            break;
                        }
                        right_dir_rho = *(ptr_cur_img_rho + i_ncols + (j + cnt_right));
                        cnt_right += 1;
                    } // end while
                    //up
                    while (up_dir_rho == 0.0)
                    {
                        if ((i - cnt_up) < 0)
                        {
                            up_dir_rho = 100.0;
                            break;
                        }
                        up_dir_rho = *(ptr_cur_img_rho + (i - cnt_up) * n_col + j);
                        cnt_up += 1;
                    } // end while
                    //down
                    while (down_dir_rho == 0.0)
                    {
                        if ((i + cnt_down) > n_row-1)
                        {
                            down_dir_rho = 100.0;
                            break;
                        }
                        down_dir_rho = *(ptr_cur_img_rho + (i + cnt_down) * n_col + j);
                        cnt_down += 1;
                    } // end while
                    four_dir[0] = (left_dir_rho);
                    four_dir[1] = (right_dir_rho);
                    four_dir[2] = (up_dir_rho);
                    four_dir[3] = (down_dir_rho);
                    min = *min_element(four_dir.begin(), four_dir.end());
                    min_index = min_element(four_dir.begin(), four_dir.end()) - four_dir.begin();

                    if (min<40.0)
                    {
                        new_phi = v_angle[i] * D2R;
                        new_theta = 0.4 * (j+1) *D2R;
                        for (int m = 0; m < 5; ++m)
                        {
                            for (int p = 0; p < 5; ++p)
                            {
                                velo_cur.push_back(pcl::PointXYZ(-min * cosf(new_phi + (float)(m - 2) * 0.2* D2R) * cosf(new_theta + (float)(p - 2) * 0.08 * D2R),
                                                                 -min * cosf(new_phi + (float)(m - 2) * 0.2* D2R) * sinf(new_theta + (float)(p - 2) * 0.08 * D2R),
                                                                  min * sinf(new_phi + (float)(m - 2) * 0.2* D2R)));
                            }
                        }
                    } // end if
                    else{}
            } // end if
            else{}
        } // end for j
    } //end for i
    
}

void MaplessDynamic::interpRangeImageMin(StrRhoPts* str_in, int n_ring, int n_radial)
{
    cv::Mat img_rho_new = str_in->img_rho.clone();
    float *ptr_img_rho = str_in->img_rho.ptr<float>(0);
    int *ptr_img_index = str_in->img_index.ptr<int>(0);
    float *ptr_img_rho_new = img_rho_new.ptr<float>(0);
    int *ptr_img_restore_warp_mask = str_in->img_restore_warp_mask.ptr<int>(0);
    int n_col = str_in->img_rho.cols;
    int n_row = str_in->img_rho.rows;

    for (int i = 0 + 2; i < (n_ring - 2); ++i)
    {
        int i_ncols = i * n_col;
        for (int j = 0 + 2; j < (n_radial - 2); ++j)
        {            
            if (*(ptr_img_rho + i_ncols + j) == 0)
            {
                if (*(ptr_img_rho + (i - 1) * n_col + j) != 0 && *(ptr_img_rho + (i + 1) * n_col + j) != 0)
                {
                    if (fabsf32(*(ptr_img_rho + (i - 1) * n_col + j) - *(ptr_img_rho + (i + 1) * n_col + j)) < 0.1)
                    {
                        *(ptr_img_restore_warp_mask + i_ncols + j) = 1;
                        *(ptr_img_rho_new + i_ncols + j) = (*(ptr_img_rho + (i - 1) * n_col + j) + *(ptr_img_rho + (i + 1) * n_col + j)) * 0.5;
                    }
                    else
                    {
                        *(ptr_img_restore_warp_mask + i_ncols + j) = 10;
                        *(ptr_img_rho_new + i_ncols + j) = std::min(*(ptr_img_rho + (i - 1) * n_col + j), *(ptr_img_rho + (i + 1) * n_col + j));
                    }
                }
                else if (*(ptr_img_rho + (i - 1) * n_col + j) != 0 && *(ptr_img_rho + (i + 2) * n_col + j) != 0)
                {
                    if (fabsf32(*(ptr_img_rho + (i - 1) * n_col + j) - *(ptr_img_rho + (i + 2) * n_col + j)) < 0.1)
                    {
                        *(ptr_img_restore_warp_mask + i_ncols + j) = 2;
                        *(ptr_img_restore_warp_mask + (i + 1) * n_col + j) = 3;
                        *(ptr_img_rho_new + i_ncols + j) = *(ptr_img_rho + (i - 1) * n_col + j) * (0.6666667) + *(ptr_img_rho + (i + 2) * n_col + j) * (0.3333333);
                        *(ptr_img_rho_new + (i + 1) * n_col + j) = *(ptr_img_rho + (i - 1) * n_col + j) * (0.3333333) + *(ptr_img_rho + (i + 2) * n_col + j) * (0.6666667);
                    }
                    else
                    {
                        *(ptr_img_restore_warp_mask + i_ncols + j) = 20;
                        *(ptr_img_restore_warp_mask + (i + 1) * n_col + j) = 30;
                        *(ptr_img_rho_new + i_ncols + j) = std::min(*(ptr_img_rho + (i - 1) * n_col + j), *(ptr_img_rho + (i + 2) * n_col + j));
                        *(ptr_img_rho_new + (i + 1) * n_col + j) = std::min(*(ptr_img_rho + (i - 1) * n_col + j), *(ptr_img_rho + (i + 2) * n_col + j));
                    }
                }

                if (*(ptr_img_rho + i_ncols + (j - 1)) != 0 && *(ptr_img_rho + i_ncols + (j + 1)) != 0)
                {
                    if (fabsf32(*(ptr_img_rho + i_ncols + (j - 1)) - *(ptr_img_rho + i_ncols + (j + 1))) < 0.05)
                    {
                        *(ptr_img_restore_warp_mask + i_ncols + j) = 4;
                        *(ptr_img_rho_new + i_ncols + j) = (*(ptr_img_rho + i_ncols + (j - 1)) + *(ptr_img_rho + i_ncols + (j + 1))) * 0.5;
                    }
                    else{}
                }
                else if (*(ptr_img_rho + i_ncols + (j - 1)) != 0 && *(ptr_img_rho + i_ncols + (j + 2)) != 0)
                {
                    if (fabsf32(*(ptr_img_rho + i_ncols + (j - 1)) - *(ptr_img_rho + i_ncols + (j + 2))) < 0.05)
                    {
                        *(ptr_img_restore_warp_mask + i_ncols + j) = 5;
                        *(ptr_img_restore_warp_mask + i_ncols + (j + 1)) = 6;
                        *(ptr_img_rho_new + i_ncols + j) = *(ptr_img_rho + i_ncols + (j - 1)) * (0.6666667) + *(ptr_img_rho + i_ncols + (j + 2)) * (0.3333333);
                        *(ptr_img_rho_new + i_ncols + (j + 1)) = *(ptr_img_rho + i_ncols + (j - 1)) * (0.3333333) + *(ptr_img_rho + i_ncols + (j + 2)) * (0.6666667);
                    }
                    else{}
                }
            } // end if
            else{}
        } // end col

    } // end row

    img_rho_new.copyTo(str_in->img_rho);
}

void MaplessDynamic::interpPtsWarp(StrRhoPts* str_in, int n_ring, int n_radial)
{
    int n_row = n_ring;
    int n_col = n_radial;
    int* ptr_img_index = str_in->img_index.ptr<int>(0);
    float* ptr_img_rho = str_in->img_rho.ptr<float>(0);
    float* ptr_img_x = str_in->img_x.ptr<float>(0);
    float* ptr_img_y = str_in->img_y.ptr<float>(0);
    float* ptr_img_z = str_in->img_z.ptr<float>(0);
    int* ptr_img_restore_warp_mask = str_in->img_restore_warp_mask.ptr<int>(0);

    for (int i = 0 + 2; i < n_row - 2; ++i)
    {
        int i_ncols = i * n_col;
        for (int j = 0 + 2; j < n_col - 2; ++j)
        {            
            if (*(ptr_img_restore_warp_mask + i_ncols + j) == 1)
            {
                *(ptr_img_x + i_ncols + j) = 0.5 * (*(ptr_img_x + (i - 1) * n_col + j) + *(ptr_img_x + (i + 1) * n_col + j));
                *(ptr_img_y + i_ncols + j) = 0.5 * (*(ptr_img_y + (i - 1) * n_col + j) + *(ptr_img_y + (i + 1) * n_col + j));
                *(ptr_img_z + i_ncols + j) = 0.5 * (*(ptr_img_z + (i - 1) * n_col + j) + *(ptr_img_z + (i + 1) * n_col + j));
            }
            else if (*(ptr_img_restore_warp_mask + i_ncols + j) == 10)
            {
                if ((*(ptr_img_rho + (i - 1) * n_col + j) < *(ptr_img_rho + (i + 1) * n_col + j)))
                {
                    *(ptr_img_x + i_ncols + j) = *(ptr_img_x + (i-1) * n_col + j);
                    *(ptr_img_y + i_ncols + j) = *(ptr_img_y + (i-1) * n_col + j);
                    *(ptr_img_z + i_ncols + j) = *(ptr_img_z + (i-1) * n_col + j);
                }
                else
                {
                    *(ptr_img_x + i_ncols + j) = *(ptr_img_x + (i+1) * n_col + j);
                    *(ptr_img_y + i_ncols + j) = *(ptr_img_y + (i+1) * n_col + j);
                    *(ptr_img_z + i_ncols + j) = *(ptr_img_z + (i+1) * n_col + j);
                }
            }
            else if (*(ptr_img_restore_warp_mask + i_ncols + j) == 2)
            {
                *(ptr_img_x + i_ncols + j) = (0.6666667) * (*(ptr_img_x + (i - 1) * n_col + j)) + (0.3333333) * (*(ptr_img_x + (i + 2) * n_col + j));
                *(ptr_img_y + i_ncols + j) = (0.6666667) * (*(ptr_img_y + (i - 1) * n_col + j)) + (0.3333333) * (*(ptr_img_y + (i + 2) * n_col + j));
                *(ptr_img_z + i_ncols + j) = (0.6666667) * (*(ptr_img_z + (i - 1) * n_col + j)) + (0.3333333) * (*(ptr_img_z + (i + 2) * n_col + j));
            }
            else if (*(ptr_img_restore_warp_mask + i_ncols + j) == 20)
            {
                if ((*(ptr_img_rho + (i - 1) * n_col + j) < *(ptr_img_rho + (i + 2) * n_col + j)))
                {
                    *(ptr_img_x + i_ncols + j) = *(ptr_img_x + (i-1) * n_col + j);
                    *(ptr_img_y + i_ncols + j) = *(ptr_img_y + (i-1) * n_col + j);
                    *(ptr_img_z + i_ncols + j) = *(ptr_img_z + (i-1) * n_col + j);
                }
                else
                {
                    *(ptr_img_x + i_ncols + j) = *(ptr_img_x + (i+2) * n_col + j);
                    *(ptr_img_y + i_ncols + j) = *(ptr_img_y + (i+2) * n_col + j);
                    *(ptr_img_z + i_ncols + j) = *(ptr_img_z + (i+2) * n_col + j);
                }
            }
            else if (*(ptr_img_restore_warp_mask + i_ncols + j) == 3)
            {
                *(ptr_img_x + i_ncols + j) = (0.3333333) * (*(ptr_img_x + (i - 2) * n_col + j)) + (0.6666667) * (*(ptr_img_x + (i + 1) * n_col + j));
                *(ptr_img_y + i_ncols + j) = (0.3333333) * (*(ptr_img_y + (i - 2) * n_col + j)) + (0.6666667) * (*(ptr_img_y + (i + 1) * n_col + j));
                *(ptr_img_z + i_ncols + j) = (0.3333333) * (*(ptr_img_z + (i - 2) * n_col + j)) + (0.6666667) * (*(ptr_img_z + (i + 1) * n_col + j));
            }
            else if (*(ptr_img_restore_warp_mask + i_ncols + j) == 30)
            {
                if ((*(ptr_img_rho + (i - 2) * n_col + j) < *(ptr_img_rho + (i + 1) * n_col + j)))
                {
                    *(ptr_img_x + i_ncols + j) = *(ptr_img_x + (i-2) * n_col + j);
                    *(ptr_img_y + i_ncols + j) = *(ptr_img_y + (i-2) * n_col + j);
                    *(ptr_img_z + i_ncols + j) = *(ptr_img_z + (i-2) * n_col + j);
                }
                else
                {
                    *(ptr_img_x + i_ncols + j) = *(ptr_img_x + (i+1) * n_col + j);
                    *(ptr_img_y + i_ncols + j) = *(ptr_img_y + (i+1) * n_col + j);
                    *(ptr_img_z + i_ncols + j) = *(ptr_img_z + (i+1) * n_col + j);
                }
            }
            else if (*(ptr_img_restore_warp_mask + i_ncols + j) == 4)
            {
                *(ptr_img_x + i_ncols + j) = 0.5 * (*(ptr_img_x + i_ncols + (j - 1)) + (*(ptr_img_x + i_ncols + (j + 1))));
                *(ptr_img_y + i_ncols + j) = 0.5 * (*(ptr_img_y + i_ncols + (j - 1)) + (*(ptr_img_y + i_ncols + (j + 1))));
                *(ptr_img_z + i_ncols + j) = 0.5 * (*(ptr_img_z + i_ncols + (j - 1)) + (*(ptr_img_z + i_ncols + (j + 1))));
            }
            else if (*(ptr_img_restore_warp_mask + i_ncols + j) == 5)
            {
                *(ptr_img_x + i_ncols + j) = (0.6666667) * (*(ptr_img_x + i_ncols + (j - 1))) + (0.3333333) * (*(ptr_img_x + i_ncols + (j + 2)));
                *(ptr_img_y + i_ncols + j) = (0.6666667) * (*(ptr_img_y + i_ncols + (j - 1))) + (0.3333333) * (*(ptr_img_y + i_ncols + (j + 2)));
                *(ptr_img_z + i_ncols + j) = (0.6666667) * (*(ptr_img_z + i_ncols + (j - 1))) + (0.3333333) * (*(ptr_img_z + i_ncols + (j + 2)));
            }
            else if (*(ptr_img_restore_warp_mask + i_ncols + j) == 6)
            {
                *(ptr_img_x + i_ncols + j) = (0.3333333) * (*(ptr_img_x + i_ncols + (j - 2))) + (0.6666667) * (*(ptr_img_x + i_ncols + (j + 1)));
                *(ptr_img_y + i_ncols + j) = (0.3333333) * (*(ptr_img_y + i_ncols + (j - 2))) + (0.6666667) * (*(ptr_img_y + i_ncols + (j + 1)));
                *(ptr_img_z + i_ncols + j) = (0.3333333) * (*(ptr_img_z + i_ncols + (j - 2))) + (0.6666667) * (*(ptr_img_z + i_ncols + (j + 1)));
            }
            else{}
        } // end for j
    } // end for i
}

void MaplessDynamic::warpPointcloud(StrRhoPts *str_cur, const Pose &T01, /*output*/ cv::Mat &mat_in, int cnt_data)
{
    int n_row = str_next_->img_rho.rows;
    int n_col = str_next_->img_rho.cols;
    float *ptr_mat_in = mat_in.ptr<float>(0);
    float *ptr_img_x = str_cur->img_x.ptr<float>(0);
    float *ptr_img_y = str_cur->img_y.ptr<float>(0);
    float *ptr_img_z = str_cur->img_z.ptr<float>(0);
    float *ptr_img_rho = str_cur->img_rho.ptr<float>(0);
    
    cv::Mat mat_out = cv::Mat::zeros(img_height_, img_width_, CV_32FC1);
    int *ptr_warp_img_index = str_warpPointcloud_->img_index.ptr<int>(0);
    float *ptr_mat_out = mat_out.ptr<float>(0);

    std::vector<float> I_vec1;
    I_vec1.reserve(n_row * n_col);
    bool isempty_flag = 0;

    for (int i = 0; i < n_row; ++i)
    {
        int i_ncols = i * n_col;
        for (int j = 0; j < n_col; ++j)
        {            
            if (*(ptr_mat_in + i_ncols + j) != 0 && *(ptr_img_rho + i_ncols + j) != 0)
            {
                velo_xyz_.push_back(pcl::PointXYZ(*(ptr_img_x + i_ncols + j), *(ptr_img_y + i_ncols + j), *(ptr_img_z + i_ncols + j)));

                I_vec1.push_back(*(ptr_mat_in + i_ncols + j));
                isempty_flag = 1;
            }
        } //end for i
    }     // end for j

    if (isempty_flag == 0)
    {
        return;
    }

    pcl::transformPointCloud(velo_xyz_, *pts_warpewd_, T01);

    genRangeImages(*pts_warpewd_, str_warpPointcloud_);
    // countZerofloat(str_warpPointcloud_->img_index);

    // int cnt_debug = 0;

    for (int i = 0; i < n_row; ++i)
    {
        int i_ncols = i * n_col;
        for (int j = 0; j < n_col; ++j)
        {            
            if (*(ptr_warp_img_index + i_ncols + j) != 0)
            {
                *(ptr_mat_out + i_ncols + j) = I_vec1[*(ptr_warp_img_index + i_ncols + j)];
                // std::cout<<cnt_debug << " "<< *(ptr_warp_img_index + i_ncols + j)<<std::endl;
                // cnt_debug += 1;
            } // end if
        } // end for j
    } //end for i
    // countZerofloat(mat_in);
    // if (cnt_data==3)
    // {
    //     exit(0);
    // }

    // ptr_mat_out = ptr_mat_in;
    mat_out.copyTo(mat_in);
}

void MaplessDynamic::filterOutAccumdR(StrRhoPts* str_next, StrRhoPts* str_cur_warped, cv::Mat& accumulated_dRdt, cv::Mat& accumulated_dRdt_score, cv::Mat& residual)
{
    int n_row = str_next->img_rho.rows;
    int n_col = str_next->img_rho.cols;
    float coef_accum_w[2] = {0.5, 0.9};
    float* ptr_accumulated_dRdt = accumulated_dRdt.ptr<float>(0);
    float* ptr_next_img_rho = str_next->img_rho.ptr<float>(0);
    float* ptr_cur_warped_img_rho = str_cur_warped->img_rho.ptr<float>(0);
    float* ptr_residual = residual.ptr<float>(0);

    // Accumulate the occlusion
    for (int i = 0; i<n_row; ++i)
    {
        int i_ncols = i * n_col;
        for (int j=0; j<n_col; ++j)
        {
            if (*(ptr_next_img_rho + i_ncols + j) < 10.0)
            {
                *(ptr_accumulated_dRdt + i_ncols + j) = coef_accum_w[0] * (*(ptr_accumulated_dRdt + i_ncols + j)) + *(ptr_residual + i_ncols + j);
            }
            else // >10
            {
                *(ptr_accumulated_dRdt + i_ncols + j) = coef_accum_w[1] * (*(ptr_accumulated_dRdt + i_ncols + j)) + *(ptr_residual + i_ncols + j);
            }            
        }
    }

    // Extract candidate for objects
    for (int i = 0; i<n_row; ++i)
    {
        int i_ncols = i * n_col;
        for (int j = 0; j < n_col; ++j)
        {
            if ( (*(ptr_next_img_rho + i_ncols + j) > 40) 
            || (*(ptr_accumulated_dRdt + i_ncols + j) < alpha_ * (*(ptr_next_img_rho + i_ncols + j)))
            ||  (*(ptr_next_img_rho + i_ncols + j) == 0) 
            || (*(ptr_cur_warped_img_rho + i_ncols + j) == 0)
            || (*(ptr_residual + i_ncols + j) < (- beta_ * (*(ptr_next_img_rho + i_ncols + j)))) )
            {
                *(ptr_accumulated_dRdt + i_ncols + j) = 0;
            }
        }
    }
}

void MaplessDynamic::extractObjectCandidate(cv::Mat& accumulated_dRdt, StrRhoPts* str_next, int object_threshold)
{
    int n_col = accumulated_dRdt.cols;
    int n_row = accumulated_dRdt.rows;
    cv::Mat object_mask = cv::Mat::zeros(img_height_, img_width_, CV_8UC1);
    uchar* ptr_object_mask = object_mask.ptr<uchar>(0);
    float* ptr_accumulated_dRdt = accumulated_dRdt.ptr<float>(0);
    int cnt = 0;
    float* ptr_next_img_z = str_next->img_z.ptr<float>(0);
    float* ptr_next_img_rho = str_next->img_rho.ptr<float>(0);

    cv::Mat object_label = cv::Mat::zeros(img_height_, img_width_, CV_32SC1);

    for (int i=0; i<n_row; ++i)
    {
        int i_ncols = i * n_col;
        for (int j=0; j<n_col; ++j)
        {
            if (*(ptr_accumulated_dRdt + i_ncols + j) > 0)
            {
                *(ptr_object_mask + i_ncols + j) = 255;
                cnt += 1;
            }
        }
    }
    int n_label = cv::connectedComponents(object_mask, object_label, 8);
    int* ptr_object_label = object_label.ptr<int>(0);

    if (n_label == 0)
    {
        return;
    }

    std::vector<int> object_row;
    std::vector<int> object_col;
    std::vector<float> object_rho_roi;
    object_row.reserve(100000);
    object_col.reserve(100000);
    object_rho_roi.reserve(100000);

    std::vector<float> max_his_object_rho_roi;
    max_his_object_rho_roi.reserve(100000);

    std::vector<int> disconti_row;
    std::vector<int> disconti_col;
    disconti_row.reserve(100000);
    disconti_col.reserve(100000);

    std::vector<int> diff_object_area_bw_disconti_row;
    std::vector<int> diff_object_area_bw_disconti_col;
    diff_object_area_bw_disconti_row.reserve(100000);
    diff_object_area_bw_disconti_col.reserve(100000);

    std::vector<float> diff_z;
    diff_z.reserve(100000);

    cv::MatND histogram;
    for (int object_idx = 0; object_idx < n_label; ++object_idx)
    {
        if (object_idx==0) //background
        {
            continue;
        }

        object_row.resize(0);
        object_col.resize(0);
        object_rho_roi.resize(0);
        max_his_object_rho_roi.resize(0);
        disconti_row.resize(0);
        disconti_col.resize(0);
        diff_object_area_bw_disconti_row.resize(0);
        diff_object_area_bw_disconti_col.resize(0);
        diff_z.resize(0);
        cv::Mat object_rho_mat = cv::Mat::zeros(img_height_, img_width_, CV_32FC1);
        float *ptr_object_rho_mat = object_rho_mat.ptr<float>(0);

        for (int i = 0; i < n_row; ++i)
        {
            int i_ncols = i * n_col;
            for (int j = 0; j < n_col; ++j)
            {
                if (*(ptr_object_label + i_ncols + j) == object_idx)
                {
                    object_row.push_back(i);
                    object_col.push_back(j);
                    object_rho_roi.push_back(*(ptr_next_img_rho + i_ncols + j));
                    *(ptr_object_rho_mat + i_ncols + j) = *(ptr_next_img_rho + i_ncols + j);
                }
            }
        }

        if (object_row.size() < object_threshold)
        {
            for (int i = 0; i < object_row.size(); ++i)
            {
                *(ptr_accumulated_dRdt + object_row[i] * n_col + object_col[i]) = 0.0;
                continue;
            }
        }
        else
        {   
            std::cout << object_row.size() <<std::endl;
            float his_range_max = *max_element(object_rho_roi.begin(), object_rho_roi.end());
            float his_range_min = *min_element(object_rho_roi.begin(), object_rho_roi.end());
            float his_range[] = {his_range_min, his_range_max};
            const int* channel_numbers = {0};
            const float* his_ranges= his_range;
            int number_bins = 50;
            // std::cout<<object_rho_roi.size()<<std::endl;
            // std::cout<<his_range[0]<<std::endl;
            // std::cout<<his_range[1]<<std::endl;
            cv::calcHist(&object_rho_mat, 1, channel_numbers, cv::Mat(), histogram, 1, &number_bins, &his_ranges);

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
            float max_his_average = 0.0;

            for (int p = 0; p<object_rho_roi.size(); ++p)
            {
                if (object_rho_roi[p]>bin_range_min && object_rho_roi[p]<bin_range_max)
                {
                    max_his_object_rho_roi.push_back(object_rho_roi[p]);
                }
            }
            for (int i=0; i<max_his_object_rho_roi.size(); ++i)
            {
                max_his_average += max_his_object_rho_roi[i];
            }
            max_his_average = max_his_average/(float)max_his_object_rho_roi.size();

            if ((max_his_average - 1.0) < 0.0)
            {
                range_min = max_his_average;
            }
            else{
                range_min = max_his_average - 0.8;
            }
            range_max = max_his_average + 1.0;

            for (int i = 0; i < object_row.size(); ++i)
            {
                if((object_rho_roi[i]<range_min) || (object_rho_roi[i]>range_max))
                {
                    disconti_row.push_back(object_row[i]);
                    disconti_col.push_back(object_col[i]);
                }
                else
                {
                    diff_object_area_bw_disconti_row.push_back(object_row[i]);
                    diff_object_area_bw_disconti_col.push_back(object_col[i]);
                }
            }
            if((object_row.size()-disconti_row.size()) < object_threshold )
            {
                for (int i = 0; i < object_row.size(); ++i)
                {
                    *(ptr_accumulated_dRdt + object_row[i] * n_col + object_col[i]) = 0;
                    continue;
                }
            }
            else
            {
                for (int i = 0; i < disconti_row.size(); ++i)
                {
                    *(ptr_accumulated_dRdt + disconti_row[i] * n_col + disconti_col[i]) = 0;
                }
            }

            for(int i=0; i<diff_object_area_bw_disconti_row.size(); ++i)
            {
                if (*(ptr_next_img_z+diff_object_area_bw_disconti_row[i]*n_col+diff_object_area_bw_disconti_col[i])!=0)
                {
                    diff_z.push_back(*(ptr_next_img_z + diff_object_area_bw_disconti_row[i] * n_col + diff_object_area_bw_disconti_col[i]));
                }
            }

            float mean_diff_z = 0.0;
            for(int i=0; i<diff_z.size(); ++i)
            {
                mean_diff_z += diff_z[i];
            }
            mean_diff_z = mean_diff_z/(float)diff_z.size();

            float std_diff_z = 0.0;
            for(int i=0; i<diff_z.size(); ++i)
            {
                std_diff_z += (diff_z[i]-mean_diff_z) * (diff_z[i]-mean_diff_z);
            }
            std_diff_z = (1.0/((float)diff_z.size()-1.0)*std_diff_z);
            if (std_diff_z < 0.07*0.07)
            {
                for (int i = 0; i < object_row.size(); ++i)
                {
                    *(ptr_accumulated_dRdt + object_row[i] * n_col + object_col[i]) = 0;
                }
                // for (int i = 0; i < diff_object_area_bw_disconti_row.size(); ++i)
                // {
                //     *(ptr_accumulated_dRdt + diff_object_area_bw_disconti_row[i] * n_col + diff_object_area_bw_disconti_col[i]) = 0;
                // }
            }

            // std::cout<<"   ====================    " <<std::endl;
            // std::cout<<histogram<<std::endl;
            // std::cout<<max_idx<<std::endl;
            // std::cout<<bin_range_min<<" " << bin_range_max << std::endl;
            // std::cout<<max_his_average<< std::endl;
            // std::cout<<(float)max_his_object_rho_roi.size() << std::endl;


        }
                    // cv::imshow("accumulated_dRdt", accumulated_dRdt);
            // cv::waitKey(0);
    } // end for object_idx
    // cv::imshow("accumulated_dRdt", accumulated_dRdt);
    // cv::waitKey(0);
    // exit(0);

}

void MaplessDynamic::fastsegmentGround(StrRhoPts* str_in)
{
    int n_row = str_in->img_rho.rows;
    int n_col = str_in->img_rho.cols;
    float* ptr_img_z = str_in->img_z.ptr<float>(0);
    float* ptr_img_rho = str_in->img_rho.ptr<float>(0);
    uchar* ptr_groundPtsIdx_next = groundPtsIdx_next_.ptr<uchar>(0);

    int downsample_size = 10;
    int n_col_sample = std::round(n_col/downsample_size);

    cv::Mat pts_z_sample = cv::Mat::zeros(n_row, n_col_sample, CV_32FC1);
    float* ptr_pts_z_sample = pts_z_sample.ptr<float>(0);
    cv::Mat rho_sample   = cv::Mat::zeros(n_row, n_col_sample, CV_32FC1);
    float* ptr_rho_sample = rho_sample.ptr<float>(0);

    std::vector<int> col_range;
    std::vector<float> pts_z_col_range;
    std::vector<float> rho_col_range;
    col_range.reserve(downsample_size);
    pts_z_col_range.reserve(downsample_size);
    rho_col_range.reserve(downsample_size);
    
    float min_z = 0.0;
    int min_idx = 0;

    for (int i=0; i<n_row; ++i)
    {
        int i_ncols = i * n_col;
        int i_ncols_sample = i * n_col_sample;
        for (int j=0; j<n_col_sample; ++j)
        {   
            col_range.resize(0);
            pts_z_col_range.resize(0);
            rho_col_range.resize(0);

            int j_downsample_size = j * downsample_size;
            for (int k=0; k<downsample_size; ++k)
            {
                col_range.push_back(j_downsample_size + k);
                pts_z_col_range.push_back(*(ptr_img_z + i_ncols + j_downsample_size + k));
                rho_col_range.push_back(*(ptr_img_rho + i_ncols + j_downsample_size + k));
            }
            min_z = *min_element(pts_z_col_range.begin(), pts_z_col_range.end());
            min_idx = min_element(pts_z_col_range.begin(), pts_z_col_range.end()) - pts_z_col_range.begin();

            if (min_z == 0)
            {
                continue;
            }
            *(ptr_pts_z_sample + i_ncols_sample + j)   = min_z;
            *(ptr_rho_sample + i_ncols_sample + j)     = *(ptr_img_rho + i_ncols + col_range[0] + min_idx);
            // std::cout << j << " "<<*(ptr_rho_sample + i_ncols + j) << std::endl;
        }
    }

    std::vector<float> points_rho;
    std::vector<float> points_z;
    bool mask_inlier[n_row];
    points_rho.reserve(n_row);
    points_z.reserve(n_row);

    cv::Mat mask_inlier_mat = cv::Mat::zeros(n_row, n_col_sample, CV_32FC1);
    float* ptr_mask_inlier_mat = mask_inlier_mat.ptr<float>(0);
    
    for (int j=0; j<n_col_sample; ++j)
    {
        points_rho.resize(0);
        points_z.resize(0);
        for (int i_mask=0; i_mask<n_row; ++i_mask) {mask_inlier[i_mask] = false;}
        for (int i = 0; i < n_row; ++i)
        {
            points_rho.push_back(*(ptr_rho_sample + i * n_col_sample + j));
            points_z.push_back(*(ptr_pts_z_sample + i * n_col_sample + j));
            // std::cout << (*(ptr_rho_sample + i * n_col_sample + j)) << " " << (*(ptr_pts_z_sample + i * n_col_sample + j)) <<std::endl;
        }
        
        ransacLine(points_rho, points_z, /*output*/ mask_inlier, j);


        for (int i=0; i<n_row; ++i)
        {
            if (mask_inlier[i]==true)
            {
                *(ptr_mask_inlier_mat + i * n_col_sample + j) = 255;
            }
        }
    }
    // std::cout << mask_inlier_mat <<std::endl;
    // exit(0);
    
    // cv::imshow("mat", mask_inlier_mat);
    // cv::waitKey(0);

    float rep_z_value = 0.0;
    std::vector<bool> bin_ground_mask;
    bin_ground_mask.reserve(downsample_size);
    for (int i = 0; i < n_row; ++i)
    {
        int i_ncols = i * n_col;
        for (int j = 0; j < n_col_sample; ++j)
        {
            col_range.resize(0);
            pts_z_col_range.resize(0);
            bin_ground_mask.resize(0);
            int j_downsample_size = j * downsample_size;
            if (*(ptr_mask_inlier_mat + i * n_col_sample + j) == 255)
            {
                rep_z_value = *(ptr_pts_z_sample + i * n_col_sample + j);
                if (rep_z_value == 0)
                {
                    continue;
                }
                else{}

                for (int k = 0; k < downsample_size; ++k)
                {
                    col_range.push_back(j_downsample_size + k);
                    pts_z_col_range.push_back(*(ptr_img_z + i_ncols + j_downsample_size + k));
                    if ((pts_z_col_range[k] != 0) && (pts_z_col_range[k] < (rep_z_value + 0.05)))
                    {
                        bin_ground_mask.push_back(true);
                        *(ptr_groundPtsIdx_next + i_ncols + col_range[k]) = 255;
                    }
                    else
                    {
                        bin_ground_mask.push_back(false);
                        *(ptr_groundPtsIdx_next + i_ncols + col_range[k]) = 0;
                    }                    
                }

                // for (int ii = 0 ; ii<col_range.size(); ++ii)
                // {
                //     if (bin_ground_mask[ii] == true)
                //     {
                //         *(ptr_groundPtsIdx_next + i_ncols + col_range[ii]) = 255;
                //     }
                //     else
                //     {
                //         *(ptr_groundPtsIdx_next + i_ncols + col_range[ii]) = 0;
                //     }
                // }
            }
        }
    }
    // cv::imshow("groundPtsIdx_next", groundPtsIdx_next_);
    // cv::waitKey(0);
}

void MaplessDynamic::ransacLine(std::vector<float>& points_rho, std::vector<float>& points_z, /*output*/ bool mask_inlier[], int num_seg)
{

    float* ptr_points_rho = points_rho.data();
    float* ptr_points_z = points_z.data();

    std::vector<float> points_rho_dup;
    points_rho_dup.resize(points_rho.size());
    std::copy(points_rho.begin(), points_rho.end(), points_rho_dup.begin());
    float* ptr_points_rho_dup = points_rho_dup.data();
    
    std::vector<float> points_z_dup;
    points_z_dup.resize(points_rho.size());
    std::copy(points_z.begin(), points_z.end(), points_z_dup.begin());
    float* ptr_points_z_dup = points_z_dup.data();

    int max_range = 100;
    int n_bin_per_seg = points_rho.size();

    bool mask_temp[n_bin_per_seg];
    bool flag_nnz_mask_temp = false;

    std::vector<float> points_rho_sort_temp;
    std::vector<float> points_z_sort_temp;
    points_rho_sort_temp.reserve(n_bin_per_seg);
    points_z_sort_temp.reserve(n_bin_per_seg);

    std::vector<int> idx_non_zero_mask_temp;
    idx_non_zero_mask_temp.reserve(n_bin_per_seg);
    int* ptr_idx_non_zero_mask_temp = idx_non_zero_mask_temp.data();

    std::vector<int> valid_points_idx;
    valid_points_idx.reserve(max_range);
    int* ptr_valid_points_idx = valid_points_idx.data();
    
    for (int i=0; i<max_range; ++i)
    {
        for (int i_mask = 0; i_mask < n_bin_per_seg; ++i_mask)
        {
            mask_temp[i_mask] = false;
        }
        flag_nnz_mask_temp = false;
        points_rho_sort_temp.resize(0);
        points_z_sort_temp.resize(0);
        idx_non_zero_mask_temp.resize(0);
        float min_z = 0.0;
        int min_idx = 0;

        for (int j=0; j<n_bin_per_seg; ++j)
        {
            if (ptr_points_rho[j] > i && ptr_points_rho[j] < i + 1.0)
            {
                mask_temp[j] = true;
                flag_nnz_mask_temp = true;

                points_rho_sort_temp.push_back(ptr_points_rho[j]);
                points_z_sort_temp.push_back(ptr_points_z[j]);
                idx_non_zero_mask_temp.push_back(j);
            }
            else{}
        }
        if (flag_nnz_mask_temp == true)
        {
            min_z = *min_element(points_z_sort_temp.begin(), points_z_sort_temp.end());
            min_idx = min_element(points_z_sort_temp.begin(), points_z_sort_temp.end()) - points_z_sort_temp.begin();
            if (min_z > -1.0)
            {
                continue;
            }
            // std::cout << min_idx << std::endl;
            // std::cout << idx_non_zero_mask_temp[min_idx] << std::endl;
            // exit(0);
            valid_points_idx.push_back(ptr_idx_non_zero_mask_temp[min_idx]);
        }
    }
    // for (int i=0; i<valid_points_idx.size(); ++i)
    // {
    //     std::cout << valid_points_idx[i] << std::endl;
    // }
    // exit(0);

    bool non_zero_points_mask[n_bin_per_seg];
    for (int i=0; i<n_bin_per_seg; ++i){non_zero_points_mask[i] = false;}
    for (int i=0; i<valid_points_idx.size(); ++i)
    {
        non_zero_points_mask[ptr_valid_points_idx[i]] = true;
    }

    // std::vector<bool> mask_inlier;
    // mask_inlier.resize(non_zero_points_mask.size(), false);

    std::vector<float> points_valid_rho;
    points_valid_rho.reserve(points_rho.size());
    float* ptr_points_valid_rho = points_valid_rho.data();

    std::vector<float> points_valid_z;
    points_valid_z.reserve(points_rho.size());
    float *ptr_points_valid_z = points_valid_z.data();

    for (int i=0; i<valid_points_idx.size(); ++i)
    {
        points_valid_rho.push_back(ptr_points_rho[ptr_valid_points_idx[i]]);
        points_valid_z.push_back(ptr_points_z[ptr_valid_points_idx[i]]);
    }

    // for (int i=0; i<valid_points_idx.size(); ++i)
    // {
    //     std::cout << points_valid_rho[i] << std::endl;
    // }
    // for (int i=0; i<valid_points_idx.size(); ++i)
    // {
    //     std::cout << points_valid_z[i] << std::endl;
    // }
    
    if (points_valid_rho.size() < 2)
    {
        // ROS_INFO_STREAM("Not enough pts for RANSAC");
        // std::cout << "number of segment: " << num_seg << std::endl; 
        return;
    }

    int iter = paramRANSAC_.iter;
    float thr = paramRANSAC_.thr;
    float a_thr = paramRANSAC_.a_thr; // abs
    float b_thr[2] = {paramRANSAC_.b_thr[0], paramRANSAC_.b_thr[1]}; // under
    int mini_inlier = paramRANSAC_.min_inlier;
    int n_sample = 2;
    int n_pts_valid = points_valid_rho.size();

    bool id_good_fit[iter];
    for (int i=0; i<iter; ++i)
    {
        id_good_fit[i] = false;
    }

    bool ini_inlier[iter][n_pts_valid];
    for (int i = 0; i < iter; ++i)
    {
        for (int j = 0; j < n_pts_valid; ++j)
        {
            ini_inlier[i][j] = false;
        }
    }
    bool mask[iter][n_pts_valid];
    for (int i = 0; i < iter; ++i)
    {
        for (int j = 0; j < n_pts_valid; ++j)
        {
            mask[i][j] = false;
        }
    }
    float residual[iter][n_pts_valid];
    for (int i = 0; i < iter; ++i)
    {
        for (int j = 0; j < n_pts_valid; ++j)
        {
            residual[i][j] = 0.0;
        }
    }
    // std::vector<int> inlier_cnt(iter, 0);
    int inlier_cnt[iter];
    for (int i = 0; i < iter; ++i)
    {
        inlier_cnt[i] = 0;
    }

    std::vector<pcl::PointCloud<pcl::PointXY>> inlier;
    inlier.resize(iter);
    for (int i=0; i<iter; ++i)
    {
        inlier[i].reserve(points_rho.size());
    }

    float line_A[iter];
    for (int i=0; i<iter; ++i)
    {
        line_A[i] = 0.0;
    }
    float line_B[iter];
    for (int i=0; i<iter; ++i)
    {
        line_B[i] = 0.0;
    }

    int n1 = 0;
    int n2 = 0;
    float x1 = 0.0, x2 = 0.0, y1 = 0.0, y2 = 0.0;

    std::random_device rd;
    std::mt19937 gen(rd());
    std::uniform_int_distribution<int> dis(0, n_pts_valid - 1);

    for (int m=0; m<iter; ++m)
    {
        inlier_cnt[m] = 1;

        while(1)
        { 
            n1 = dis(gen);
            n2 = dis(gen);
            if (n1 != n2)
            {
                break;
            }
        }

        x1 = ptr_points_valid_rho[n1];
        y1 = ptr_points_valid_z[n1];
        x2 = ptr_points_valid_rho[n2];
        y2 = ptr_points_valid_z[n2];

        line_A[m] = (y2 - y1) / (x2 - x1);
        line_B[m] = -line_A[m] * x1 + y1;

        if (line_A[m] < 0.0)
        {
            if (line_A[m] < -a_thr || line_B[m] > b_thr[0])
            {
                continue;
            }
            else{}
        }
        else{}

        if (line_A[m] > 0.0)
        {
            if (line_A[m] > a_thr || line_B[m] > b_thr[1])
            {
                continue;
            }
            else{}
        }
        else{}

        for (int i=0; i<n_pts_valid; ++i)
        {
            residual[m][i] = std::abs(line_A[m]*ptr_points_valid_rho[i] - ptr_points_valid_z[i] + line_B[m])
                            / std::sqrt(line_A[m]*line_A[m] + 1.0);
             
            if (residual[m][i] < thr)
            {
                ini_inlier[m][i] = true;
                mask[m][i] = true;
            }
        }


        pcl::PointXY point_xy;

        for (int j=0; j<n_pts_valid; ++j)
        {
            if (ini_inlier[m][j]==true)
            {
                point_xy.x = ptr_points_valid_rho[j];
                point_xy.y = ptr_points_valid_z[j];
                inlier[m].push_back(point_xy);
                inlier_cnt[m] += 1;
            }
        }
        if ((inlier_cnt[m] - 1) > mini_inlier)
        {
            id_good_fit[m] = true;
        }
    }


    int max_inlier_cnt = 0;
    std::vector<int> max_inlier_cnt_index;
    max_inlier_cnt_index.reserve(100);

    max_inlier_cnt = *std::max_element(inlier_cnt, inlier_cnt + iter);

    for (int i=0; i<iter; ++i)
    {
        if (inlier_cnt[i] == max_inlier_cnt)
        {
            max_inlier_cnt_index.push_back(i);
        }
    }

    float mini_pre = 1e3;


    if (max_inlier_cnt_index.size()==0)
    {
        return;
    }
    
    int id_mini = 0;
    int max_inlier_cnt_index_1 = 0;
    if (max_inlier_cnt_index.size()>1)
    {
        int n_candi = max_inlier_cnt_index.size();
        for (int i_candi = 0; i_candi < n_candi; ++i_candi)
        {
            float mean_residual = 0.0;
            for (int k=0; k<n_pts_valid; ++k)
            {
                mean_residual += residual[max_inlier_cnt_index[i_candi]][k];
            }

            mean_residual /= n_pts_valid;

            float mini = std::min(mean_residual, mini_pre);
            
            if (mini < mini_pre)
            {
                id_mini = i_candi;
            }
            mini_pre = mini;
        }
        max_inlier_cnt_index_1 = max_inlier_cnt_index[id_mini];
    }
    else
    {
        max_inlier_cnt_index_1 = max_inlier_cnt_index[0];
    }
    int best_n_inlier = inlier_cnt[max_inlier_cnt_index_1] - 1;

    // if (best_n_inlier < 3)
    // {
    //     // ROS_INFO_STREAM("inlier pts < 3");
    //     // std::cout << "number of segment: " << num_seg << std::endl; 
    //     // return;
    // }
    if (best_n_inlier == 0)
    {
        // ROS_INFO_STREAM("# of inlier pts = 0");
        return;
    }

    Eigen::MatrixXf A = Eigen::MatrixXf::Zero(best_n_inlier,3);
    for (int i=0; i<best_n_inlier; ++i)
    {
        A(i,0) = inlier[max_inlier_cnt_index_1][i].x;
        A(i,1) = inlier[max_inlier_cnt_index_1][i].y;
        A(i,2) = 1;
    }
    
    Eigen::Vector3f t;
    Eigen::JacobiSVD<Eigen::MatrixXf> svd(A, Eigen::ComputeFullU | Eigen::ComputeFullV);
    t = svd.matrixV().col(2);
        // std::cout << svd.matrixU() << std::endl;
        // std::cout << "***************" << std::endl;
        // std::cout << svd.matrixV() << std::endl;
        // std::cout << "***************" << std::endl;
        // std::cout << svd.singularValues() << std::endl;
        // exit(0);

    // Eigen::JacobiSVD<Eigen::MatrixXf> svd(A, Eigen::ComputeThinU | Eigen::ComputeThinV);
    // // cout << "Its singular values are:" << endl << svd.singularValues() << endl;
    // // cout << "Its left singular vectors are the columns of the thin U matrix:" << endl << svd.matrixU() << endl;
    // // cout << "Its right singular vectors are the columns of the thin V matrix:" << endl << svd.matrixV() << endl;

    t = -t/t(1,0);

    Eigen::Vector2f line;
    line << t(0), t(2); //y = ax + b

    float residual_leastsquare = 0.0;
    float line_updown = 0.0;

    for (int i=0; i<points_rho_dup.size(); ++i)
    {
        residual_leastsquare = std::abs(t(0)*ptr_points_rho_dup[i] + t(1)*ptr_points_z_dup[i] + t(2)) /
                                std::sqrt(t(0)*t(0)+t(1)*t(1));
        line_updown = t(0)*ptr_points_rho_dup[i] + t(1)*ptr_points_z_dup[i] + t(2);

        if ((residual_leastsquare < thr) || (line_updown > 0))
        {
            mask_inlier[i] = true;
        }
    }
    // output: mask_inlier
}

void MaplessDynamic::checkSegment(cv::Mat& accumulated_dRdt, StrRhoPts* str_next, cv::Mat& groundPtsIdx_next)
{
    float weight_factor = 0.95;
    int n_col = str_next->img_rho.cols;
    int n_row = str_next->img_rho.rows;
    std::vector<int> idx_row;
    std::vector<int> idx_col;
    std::vector<int> check;
    idx_row.reserve(100000);
    idx_col.reserve(100000);
    check.reserve(100000);
    float* ptr_accumulated_dRdt = accumulated_dRdt.ptr<float>(0);
    float* ptr_img_rho = str_next->img_rho.ptr<float>(0);
    uchar* ptr_groundPtsIdx_next = groundPtsIdx_next.ptr<uchar>(0);
    bool isempty = true;

    float phi = (float)360.0/(float)n_col;
    int cnt = 0;
    int row = 0;
    int col = 0;
    float R = 0.0;

    float d1 = 0.0;
    float d2 = 0.0;

    cv::Mat roi_up = cv::Mat::zeros(img_height_, img_width_, CV_8UC1);
    uchar* ptr_roi_up = roi_up.ptr<uchar>(0);

    for (int i=0; i<n_row; ++i)
    {
        int i_ncols = i * n_col;
        for (int j = 0; j < n_col; ++j)
        {
            if (*(ptr_accumulated_dRdt + i_ncols + j) != 0)
            {
                // std::cout << accumulated_dRdt.at<float>(i,j) << std::endl;
                // std::cout<<i<<" " <<j<<std::endl;
                idx_row.push_back(i);
                idx_col.push_back(j);
                check.push_back(0);
                isempty = false;
                *(ptr_roi_up + i_ncols + j) = 1;
            }
        }
    }

    if (isempty==true)
    {
        return;
    }

    while(1)
    {
        if(check[cnt]==0)
        {
            row = idx_row[cnt];
            col = idx_col[cnt];
            R = *(ptr_img_rho + row * n_col + col);
            int i = 0;
            int j = 0;
            int ii = 0;
            int jj = 0;
            for (int idx_cw = 1; idx_cw < 5; ++idx_cw)
            {
                if (idx_cw == 1)
                {
                    i = row - 1;
                    j = col;
                }
                else if (idx_cw == 2)
                {
                    i = row;
                    j = col + 1;
                }
                else if (idx_cw == 3)
                {
                    i = row + 1;
                    j = col;
                }
                else if (idx_cw == 4)
                {
                    i = row;
                    j = col - 1;
                }

                if ( (i<0) || (i> n_row-1))
                {
                    continue;
                }

                if ( (j<0) || (j> n_col-1))
                {
                    continue;
                }
                int i_ncols_j = i * n_col + j;
                if ((*(ptr_roi_up + i_ncols_j) == 0) && (*(ptr_img_rho + i_ncols_j) > 0))
                {
                    if (*(ptr_groundPtsIdx_next + i_ncols_j) == 255)
                    {
                        continue;
                    }

                    d1 = std::max(R, *(ptr_img_rho + i_ncols_j));
                    d2 = std::min(R, *(ptr_img_rho + i_ncols_j));
                    if (atan2f(d2 * sinf(phi * D2R), (d1 - d2 * cosf(phi * D2R))) > 10.0 * D2R)
                    {
                        idx_row.push_back(i);
                        idx_col.push_back(j);
                        check.push_back(0);
                        *(ptr_roi_up + i_ncols_j) = 1;
                        *(ptr_accumulated_dRdt + i_ncols_j) = weight_factor * *(ptr_accumulated_dRdt + row * n_col + col);
                    }
                }
                else if ((*(ptr_roi_up + i_ncols_j) == 0) && (*(ptr_img_rho + i_ncols_j) == 0))
                {
                    if (*(ptr_groundPtsIdx_next + i_ncols_j) == 255)
                    {
                        continue;
                    }

                    if (idx_cw == 1)
                    {
                        ii = row - 2;
                        jj = col;
                    }
                    else if (idx_cw == 2)
                    {
                        ii = row;
                        jj = col + 2;
                    }
                    else if (idx_cw == 3)
                    {
                        ii = row + 2;
                        jj = col;
                    }
                    else if (idx_cw == 4)
                    {
                        ii = row;
                        jj = col - 2;
                    }
                    if ((ii < 0) || (ii > n_row-1))
                    {
                        continue;
                    }
                    if ((jj < 0) || (jj > n_col-1))
                    {
                        continue;
                    }
                    int ii_ncols_jj = ii * n_col + jj;
                    if (*(ptr_groundPtsIdx_next + ii_ncols_jj) == 255)
                    {
                        continue;
                    }

                    if (*(ptr_roi_up + ii_ncols_jj) == 1){}
                    else
                    {
                        d1 = std::max(R, *(ptr_img_rho + ii_ncols_jj));
                        d2 = std::min(R, *(ptr_img_rho + ii_ncols_jj));

                        if (atan2f(d2 * sinf(2.0 * phi * D2R), (d1 - d2 * cosf(2.0 * phi * D2R))) > 10.0 * D2R)
                        {
                            idx_row.push_back(ii);
                            idx_col.push_back(jj);

                            check.push_back(0);
                            *(ptr_roi_up + ii_ncols_jj) = 1;
                            *(ptr_accumulated_dRdt + ii_ncols_jj) = weight_factor * (*(ptr_accumulated_dRdt + row * n_col + col));
                            if (idx_cw == 1)
                            {
                                *(ptr_accumulated_dRdt + (ii + 1) * n_col + jj) = weight_factor * (*(ptr_accumulated_dRdt + row * n_col + col));
                            }
                            else if (idx_cw == 2)
                            {
                                *(ptr_accumulated_dRdt + ii * n_col + (jj - 1)) = weight_factor * (*(ptr_accumulated_dRdt + row * n_col + col));
                            }
                            else if (idx_cw == 3)
                            {
                                *(ptr_accumulated_dRdt + (ii - 1) * n_col + jj) = weight_factor * (*(ptr_accumulated_dRdt + row * n_col + col));
                            }
                            else if (idx_cw == 4)
                            {
                                *(ptr_accumulated_dRdt + ii * n_col + (jj + 1)) = weight_factor * (*(ptr_accumulated_dRdt + row * n_col + col));
                            }
                            else{}
                        }
                    }
                }
            }

            if (*(ptr_groundPtsIdx_next + row * n_col + col) == 255)
            {
                *(ptr_roi_up + row * n_col + col) = 0;
            }
            else{}

            check[cnt] = 1;
        }

        cnt += 1;
        if (cnt == check.size())
        {
            break;
        }
            // std::cout << "checkSegment cnt: "<< cnt << std::endl;
    }

    // cv::imshow("accumulated_dRdt", accumulated_dRdt);
    // cv::waitKey(0);
    // exit(0);
}

void MaplessDynamic::updateScore(cv::Mat& accumulated_dRdt, cv::Mat& accumulated_dRdt_score)
{
    int n_row = accumulated_dRdt.rows;
    int n_col = accumulated_dRdt.cols;
    float* ptr_accumulated_dRdt = accumulated_dRdt.ptr<float>(0);
    float* ptr_accumulated_dRdt_score = accumulated_dRdt_score.ptr<float>(0);
    
    for (int i=0; i<n_row; ++i)
    {
        int i_ncols = i * n_col;
        for (int j=0; j<n_col; ++j)
        {
            if (*(ptr_accumulated_dRdt + i_ncols + j) != 0)
            {
                *(ptr_accumulated_dRdt_score + i_ncols + j) += 1;
            }
        }
    }

    for (int i=0; i<n_row; ++i)
    {
        int i_ncols = i * n_col;
        for (int j=0; j<n_col; ++j)
        {
            if (*(ptr_accumulated_dRdt_score + i_ncols + j) > 2.0)
            {
                *(ptr_accumulated_dRdt + i_ncols + j) *= 5.0;
                if (*(ptr_accumulated_dRdt + i_ncols + j) > 1e3)
                {
                    *(ptr_accumulated_dRdt + i_ncols + j) = 1e3;
                }
                else{}
            }
            else{}
        }
    }
}

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
    for (int j = 0; j < n_col; ++j)
    {
        *(ptr_dRdt_bin_inv + (n_row - 1) * n_col + j) = 255;
    }
    cv::floodFill(dRdt_bin_inv, cv::Point(0,0), cv::Scalar(0));
    cv::Mat dRdt_bin_filled = (dRdt_bin | dRdt_bin_inv);
    interpAndfill_image(accumulated_dRdt, dRdt_bin_filled);

    cv::Mat dRdt_score_bin_inv = cv::Mat::zeros(img_height_, img_width_, CV_8UC1);
    cv::bitwise_not(dRdt_score_bin, dRdt_score_bin_inv);
    uchar *ptr_dRdt_score_bin_inv = dRdt_score_bin_inv.ptr<uchar>(0);
    for (int j = 0; j < n_col; ++j)
    {
        *(ptr_dRdt_score_bin_inv + (n_row - 1) * n_col + j) = 255;
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

    std::vector<int> object_row;
    std::vector<int> object_col;
    std::vector<float> object_rho_roi;
    object_row.reserve(100000);
    object_col.reserve(100000);
    object_rho_roi.reserve(100000);

    cv::Mat zero_candidate = cv::Mat::zeros(img_height_,img_width_, CV_8UC1);
    uchar *ptr_zero_candidate = zero_candidate.ptr<uchar>(0);

    std::vector<int> filled_object_row;
    std::vector<int> filled_object_col;
    std::vector<float> filled_object_rho_roi;
    filled_object_row.reserve(100000);
    filled_object_col.reserve(100000);
    filled_object_rho_roi.reserve(100000);


    std::vector<float> max_his_filled_object_rho_roi;
    max_his_filled_object_rho_roi.reserve(100000);

    std::vector<int> rho_zero_filled_value_row;
    std::vector<int> rho_zero_filled_value_col;
    std::vector<float> rho_zero_filled_value_rho_roi;
    rho_zero_filled_value_row.reserve(100000);
    rho_zero_filled_value_col.reserve(100000);
    rho_zero_filled_value_rho_roi.reserve(100000);

    std::vector<int> disconti_row;
    std::vector<int> disconti_col;
    disconti_row.reserve(100000);
    disconti_col.reserve(100000);

    cv::Mat object_area = cv::Mat::zeros(img_height_, img_width_, CV_8UC1);
    uchar *ptr_object_area = object_area.ptr<uchar>(0);
    cv::Mat object_area_filled = cv::Mat::zeros(img_height_, img_width_, CV_8UC1);
    uchar *ptr_object_area_filled = object_area_filled.ptr<uchar>(0);
    cv::Mat filled_object_rho_mat = cv::Mat::zeros(img_height_, img_width_, CV_32FC1);
    float *ptr_filled_object_rho_mat = filled_object_rho_mat.ptr<float>(0);

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
                *(ptr_rho_zero_value + object_row[i] * n_col + object_col[i]) = 0;
                *(ptr_accumulated_dRdt + object_row[i] * n_col + object_col[i]) = 0.0;
            }
            continue;
        }
        else
        {
            float connect_zero_mean = 0.0;
            int n_connet_zero = 0;

            cv::Mat object_area_inv = cv::Mat::zeros(img_height_, img_width_, CV_8UC1);
            cv::bitwise_not(object_area, object_area_inv);
            uchar* ptr_object_area_inv = object_area_inv.ptr<uchar>(0);
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
                    *(ptr_accumulated_dRdt + object_row[i] * n_col + object_col[i]) = 0;
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
                if (filled_object_rho_roi[p]>bin_range_min && filled_object_rho_roi[p]<bin_range_max)
                {
                    max_his_filled_object_rho_roi.push_back(filled_object_rho_roi[p]);
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
                if ((rho_zero_filled_value_rho_roi[i] < range_min) || (rho_zero_filled_value_rho_roi[i] > range_max))
                {
                    if (*(ptr_object_area_filled + rho_zero_filled_value_row[i] * n_col + rho_zero_filled_value_col[i]) != 0)
                    {
                        disconti_row.push_back(rho_zero_filled_value_row[i]);
                        disconti_col.push_back(rho_zero_filled_value_col[i]);
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