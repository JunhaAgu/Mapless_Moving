#include "mapless_dynamic.h"
#include <string>
#include <fstream> 


MaplessDynamic::MaplessDynamic(ros::NodeHandle& nh, bool test_flag)
: nh_(nh), test_flag_(test_flag), is_initialized_test_(false)
 {
    // constructor
    ROS_INFO_STREAM("MaplessDynamic - constructed.");
    // IMPLEMENT YOUR CODE FROM THIS LINE.

    // do something...

    // END YOUR CODE
    pub_dynamic_pts_ = nh_.advertise<sensor_msgs::PointCloud2>("/dynamic_pts",1);
    pub_static_pts_  = nh_.advertise<sensor_msgs::PointCloud2>("/static_pts",1);

    this->getUserSettingParameters();

    // allocation for solver
    accumulated_dRdt_ = cv::Mat::zeros(img_height_, img_width_, CV_32FC1);
    accumulated_dRdt_score_ = cv::Mat::zeros(img_height_, img_width_, CV_32FC1);
    residual_ = cv::Mat::zeros(img_height_, img_width_, CV_32FC1);

    str_cur_            = new StrRhoPts();
    str_next_           = new StrRhoPts();
    str_cur_warped_     = new StrRhoPts();
    str_warpPointcloud_ = new StrRhoPts();

    str_cur_->rho.reserve(500000);
    str_cur_->phi.reserve(500000);
    str_cur_->theta.reserve(500000);
    str_cur_->img_rho = cv::Mat::zeros(img_height_, img_width_, CV_32FC1);
    str_cur_->img_index = cv::Mat::zeros(img_height_, img_width_, CV_32SC1);

    str_cur_->img_x = cv::Mat::zeros(img_height_, img_width_, CV_32FC1);
    str_cur_->img_y = cv::Mat::zeros(img_height_, img_width_, CV_32FC1);
    str_cur_->img_z = cv::Mat::zeros(img_height_, img_width_, CV_32FC1);

    str_cur_->img_restore_mask = cv::Mat::zeros(img_height_, img_width_, CV_32SC1);
    str_cur_->pts_per_pixel_n.resize(img_height_*img_width_);
    str_cur_->pts_per_pixel_index.resize(img_height_*img_width_);
    str_cur_->pts_per_pixel_rho.resize(img_height_ * img_width_);
    for (int i=0; i<img_height_*img_width_; ++i)
    {
        str_cur_->pts_per_pixel_index[i].reserve(500);
    }
    for (int i=0; i<img_height_*img_width_; ++i)
    {
        str_cur_->pts_per_pixel_rho[i].reserve(500);
    }
    str_cur_->pts_per_pixel_index_valid.resize(img_height_ * img_width_);
    for (int i=0; i<img_height_*img_width_; ++i)
    {
        str_cur_->pts_per_pixel_index_valid[i].reserve(500);
    }
    // str_cur_->valid_original_pts_idx_.reserve(500000);
    str_cur_->pts        = boost::make_shared<pcl::PointCloud<pcl::PointXYZ>>();
    str_cur_->ptsInImage = boost::make_shared<pcl::PointCloud<pcl::PointXYZ>>();
    str_cur_->img_restore_warp_mask = cv::Mat::zeros(img_height_, img_width_, CV_32SC1);

    

    str_next_->rho.reserve(500000);
    str_next_->phi.reserve(500000);
    str_next_->theta.reserve(500000);
    str_next_->img_rho   = cv::Mat::zeros(img_height_, img_width_, CV_32FC1);
    str_next_->img_index = cv::Mat::zeros(img_height_, img_width_, CV_32SC1);

    str_next_->img_x     = cv::Mat::zeros(img_height_, img_width_, CV_32FC1);
    str_next_->img_y     = cv::Mat::zeros(img_height_, img_width_, CV_32FC1);
    str_next_->img_z     = cv::Mat::zeros(img_height_, img_width_, CV_32FC1);

    str_next_->img_restore_mask = cv::Mat::zeros(img_height_, img_width_, CV_32SC1);
    str_next_->pts_per_pixel_n.resize(img_height_*img_width_);
    str_next_->pts_per_pixel_index.resize(img_height_*img_width_);
    str_next_->pts_per_pixel_rho.resize(img_height_ * img_width_);
    for (int i=0; i<img_height_*img_width_; ++i)
    {
        str_next_->pts_per_pixel_index[i].reserve(500);
    }
    for (int i=0; i<img_height_*img_width_; ++i)
    {
        str_next_->pts_per_pixel_rho[i].reserve(500);
    }
    str_next_->pts_per_pixel_index_valid.resize(img_height_ * img_width_);
    for (int i=0; i<img_height_*img_width_; ++i)
    {
        str_next_->pts_per_pixel_index_valid[i].reserve(500);
    }
    // str_next_->valid_original_pts_idx_.reserve(500000);
    str_next_->pts        = boost::make_shared<pcl::PointCloud<pcl::PointXYZ>>();
    str_next_->ptsInImage = boost::make_shared<pcl::PointCloud<pcl::PointXYZ>>();
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

    str_cur_warped_->img_restore_mask = cv::Mat::zeros(img_height_, img_width_, CV_32SC1);
    str_cur_warped_->pts_per_pixel_n.resize(img_height_*img_width_);
    str_cur_warped_->pts_per_pixel_index.resize(img_height_*img_width_);
    str_cur_warped_->pts_per_pixel_rho.resize(img_height_ * img_width_);
    for (int i=0; i<img_height_*img_width_; ++i)
    {
        str_cur_warped_->pts_per_pixel_index[i].reserve(500);
    }
    for (int i=0; i<img_height_*img_width_; ++i)
    {
        str_cur_warped_->pts_per_pixel_rho[i].reserve(500);
    }
    str_cur_warped_->pts_per_pixel_index_valid.resize(img_height_ * img_width_);
    for (int i=0; i<img_height_*img_width_; ++i)
    {
        str_cur_warped_->pts_per_pixel_index_valid[i].reserve(500);
    }
    // str_cur_warped_->valid_original_pts_idx_.reserve(500000);
    str_cur_warped_->pts        = boost::make_shared<pcl::PointCloud<pcl::PointXYZ>>();
    str_cur_warped_->ptsInImage = boost::make_shared<pcl::PointCloud<pcl::PointXYZ>>();
    str_cur_warped_->img_restore_warp_mask = cv::Mat::zeros(img_height_, img_width_, CV_32SC1);

    velo_cur_.reserve(500000);
    velo_xyz_.reserve(500000);

    str_warpPointcloud_->rho.reserve(500000);
    str_warpPointcloud_->phi.reserve(500000);
    str_warpPointcloud_->theta.reserve(500000);
    str_warpPointcloud_->img_rho = cv::Mat::zeros(img_height_, img_width_, CV_32FC1);
    str_warpPointcloud_->img_index = cv::Mat::zeros(img_height_, img_width_, CV_32SC1);

    str_warpPointcloud_->img_x = cv::Mat::zeros(img_height_, img_width_, CV_32FC1);
    str_warpPointcloud_->img_y = cv::Mat::zeros(img_height_, img_width_, CV_32FC1);
    str_warpPointcloud_->img_z = cv::Mat::zeros(img_height_, img_width_, CV_32FC1);

    str_warpPointcloud_->img_restore_mask = cv::Mat::zeros(img_height_, img_width_, CV_32SC1);
    str_warpPointcloud_->pts_per_pixel_n.resize(img_height_*img_width_);
    str_warpPointcloud_->pts_per_pixel_index.resize(img_height_*img_width_);
    str_warpPointcloud_->pts_per_pixel_rho.resize(img_height_ * img_width_);
    for (int i=0; i<img_height_*img_width_; ++i)
    {
        str_warpPointcloud_->pts_per_pixel_index[i].reserve(500);
    }
    for (int i=0; i<img_height_*img_width_; ++i)
    {
        str_warpPointcloud_->pts_per_pixel_rho[i].reserve(500);
    }
    str_warpPointcloud_->pts_per_pixel_index_valid.resize(img_height_ * img_width_);
    for (int i=0; i<img_height_*img_width_; ++i)
    {
        str_warpPointcloud_->pts_per_pixel_index_valid[i].reserve(500);
    }
    // str_warpPointcloud_->valid_original_pts_idx_.reserve(500000);
    str_warpPointcloud_->pts        = boost::make_shared<pcl::PointCloud<pcl::PointXYZ>>();
    str_warpPointcloud_->ptsInImage = boost::make_shared<pcl::PointCloud<pcl::PointXYZ>>();
    str_warpPointcloud_->img_restore_warp_mask = cv::Mat::zeros(img_height_, img_width_, CV_32SC1);

    ptr_cur_pts_warped_ = boost::make_shared<pcl::PointCloud<pcl::PointXYZ>>();
    pts_warpewd_        = boost::make_shared<pcl::PointCloud<pcl::PointXYZ>>();

    if (test_flag_){
        this->loadTestData();
    }
};


MaplessDynamic::~MaplessDynamic() {
    // destructor
    ROS_INFO_STREAM("MaplessDynamic - deleted.");
    // IMPLEMENT YOUR CODE FROM THIS LINE.

    // do something...

    // END YOUR CODE
    delete str_cur_;
    delete str_next_;
    delete str_cur_warped_;
    delete str_warpPointcloud_;
};


void MaplessDynamic::TEST(){
    static int cnt_data = 0;
    std::cout<< "Test iter: "<< cnt_data << std::endl;
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

        // 1. Calculate T01 from the SLAM (or Odometry) algorithm
        Pose T01;
        timer::tic();
        // T01 =  vo->solve();
        T01 = data_buf_[cnt_data]->T_gt_.inverse();
        double dt_slam = timer::toc(); // milliseconds
        ROS_INFO_STREAM("elapsed time for 'SLAM' :" << dt_slam << " [ms]");

        // 2. Solve the Mapless Dynamic algorithm.
        timer::tic();
        Mask mask1;
        this->solve(p0_pcl_test_, p1_pcl_test_, T01, mask1, cnt_data);
        double dt_solver = timer::toc(); // milliseconds
        ROS_INFO_STREAM("elapsed time for 'solver' :" << dt_solver << " [ms]");

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
        start_num = 2350 + 00;
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

        data_buf_[i]->pcl_.reset(new pcl::PointCloud<pcl::PointXYZI>);
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
                data_buf_[ii]->T_gt_ = all_T_gt_[i].inverse() * all_T_gt_[i];
            else
                data_buf_[ii]->T_gt_ = all_T_gt_[i-1].inverse() * all_T_gt_[i];
            // std::cout << data_buf_[ii]->T_gt_ << std::endl;
        }
        else if (test_data_type_ == "CARLA")
        {
            if (ii == 0){}
            else
                data_buf_[ii-1]->T_gt_ = all_T_gt_[i-1].inverse() * all_T_gt_[i-1];
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

    // if (cnt_data == 2)
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

    // pointcloud input, p1
    // p0 is already processed in initial step
    genRangeImages(p1, str_next_);

    // Warp pcl represented in current frame to next frame
    T_next2cur_ = T01;
    
    // Segment ground
    segmentGround(str_next_);

    //// Occlusion accumulation ////
    // Compute the occlusion dRdt
    
    dR_warpPointcloud(p0, T_next2cur_, cnt_data);
    // str_next_->state();

    // warp the occlusion accumulation map
    warpPointcloud(str_cur_, T_next2cur_, accumulated_dRdt_, cnt_data);
    warpPointcloud(str_cur_, T_next2cur_, accumulated_dRdt_score_, cnt_data);
    
    //     if (cnt_data == 2)
    // {
    //     cv::imshow("before d", accumulated_dRdt_);
    //     countZerofloat(accumulated_dRdt_);
    //     cv::imshow("before  k", accumulated_dRdt_score_);
    //     countZerofloat(accumulated_dRdt_score_);
    // }

    // filter out outliers
    filterOutAccumdR(str_next_, str_cur_warped_, accumulated_dRdt_, accumulated_dRdt_score_, residual_);
    // if (cnt_data == 2)
    // {
    //     cv::imshow("d", accumulated_dRdt_);
    //     countZerofloat(accumulated_dRdt_);
    //     cv::imshow("k", accumulated_dRdt_score_);
    //     countZerofloat(accumulated_dRdt_score_);
    //     cv::waitKey(0);
    //     exit(0);
    // }

    // timer::tic();
    // Extract object candidate via connected components in 2-D binary image
    extractObjectCandidate(accumulated_dRdt_, str_next_, object_threshold_);
    // double dt_extractObjectCandidate = timer::toc(); // milliseconds
    // ROS_INFO_STREAM("elapsed time for 'dt_extractObjectCandidate' :" << dt_extractObjectCandidate << " [ms]");

    //// update object_mask
    //object_mask = accumulated_dRdt>0;

    // Fast Segment
    checkSegment(accumulated_dRdt_, str_next_, groundPtsIdx_next_);

    //// update object_mask
    //object_mask = accumulated_dRdt>0;

    updateScore(accumulated_dRdt_, accumulated_dRdt_score_);

    plugImageZeroHoles(accumulated_dRdt_, accumulated_dRdt_score_, str_next_, groundPtsIdx_next_, object_threshold_);

    float* ptr_accumulated_dRdt_ = accumulated_dRdt_.ptr<float>(0);
    float* ptr_accumulated_dRdt_score_ = accumulated_dRdt_score_.ptr<float>(0);
    float *ptr_next_img_rho = str_next_->img_rho.ptr<float>(0);

    for (int i = 0; i < img_height_; ++i)
    {
        for (int j = 0; j < img_width_; ++j)
        {
            if (*(ptr_accumulated_dRdt_ + i * img_width_ + j)!=0 && *(ptr_next_img_rho + i * img_width_ + j)!=0)
            {
            }
            else
            {
                *(ptr_accumulated_dRdt_ + i * img_width_ + j) = 0;
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
    //     for (int j = 0; j < img_width_; ++j)
    //     {
    //         if (*(ptr_accumulated_dRdt_ + i * img_width_ + j)!=0)
    //         {
    //             pcl_dynamic.push_back(
    //                 pcl::PointXYZ(*(ptr_next_img_x + i * img_width_ + j), *(ptr_next_img_y + i * img_width_ + j), *(ptr_next_img_z + i * img_width_ + j)));
    //         }
    //         else
    //         {
    //             pcl_static.push_back(
    //                 pcl::PointXYZ(*(ptr_next_img_x + i * img_width_ + j), *(ptr_next_img_y + i * img_width_ + j), *(ptr_next_img_z + i * img_width_ + j)));
    //         }
    //     }
    // }

    for (int i = 0; i < img_height_; ++i)
    {
        for (int j = 0; j < img_width_; ++j)
        {
            if (*(ptr_accumulated_dRdt_ + i * img_width_ + j)!=0)
            {
                for (int k = 0; k < str_next_->pts_per_pixel_index_valid[i * img_width_ + j].size(); ++k)
                {
                    pcl_dynamic.push_back(
                    pcl::PointXYZ(p1[str_next_->pts_per_pixel_index_valid[i * img_width_ + j][k]]));
                }
            }
            else
            {
                for (int k = 0; k < str_next_->pts_per_pixel_index_valid[i * img_width_ + j].size(); ++k)
                {
                    pcl_static.push_back(
                    pcl::PointXYZ(p1[str_next_->pts_per_pixel_index_valid[i * img_width_ + j][k]]));
                }
            }
        }
    }

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
    if (cnt_data == 2)
    {exit(0);}

    //// update for next iteration
    for (int i = 0; i < img_height_; ++i)
    {
        for (int j = 0; j < img_width_; ++j)
        {
            if (*(ptr_accumulated_dRdt_ + i * img_width_ + j)==0)
            {
                *(ptr_accumulated_dRdt_score_ + i * img_width_ + j) = 0;
            }
        }
    }

    copyStruct(str_next_, str_cur_, p1, p0, cnt_data);


};

void MaplessDynamic::copyStruct(StrRhoPts* str_next, StrRhoPts* str_cur, pcl::PointCloud<pcl::PointXYZ>& p1 ,pcl::PointCloud<pcl::PointXYZ>& p0, int cnt_data)
{
    // memcpy(str_cur, str_next, sizeof(struct StrRhoPts));

    // CHK
    {       
        str_cur->pts = str_next->pts;
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

        str_cur->pts_per_pixel_n.resize(0);
        std::copy(str_next->pts_per_pixel_n.begin(),         str_next->pts_per_pixel_n.end(),         str_cur->pts_per_pixel_n.begin());
        // for (int i=0; i<img_height_*img_width_; ++i)
        // {
        //     str_cur->pts_per_pixel_index[i].resize(0);
        //     std::copy(str_next->pts_per_pixel_index[i].begin(),     str_next->pts_per_pixel_index[i].end(),     str_cur->pts_per_pixel_index[i].begin());
        // }
        str_cur->pts_per_pixel_index.resize(0);
        std::copy(str_next->pts_per_pixel_index.begin(),     str_next->pts_per_pixel_index.end(),     str_cur->pts_per_pixel_index.begin());
        // for (int i=0; i<img_height_*img_width_; ++i)
        // {
        //     str_cur->pts_per_pixel_rho[i].resize(0);
        //     std::copy(str_next->pts_per_pixel_rho[i].begin(),     str_next->pts_per_pixel_rho[i].end(),     str_cur->pts_per_pixel_rho[i].begin());
        // }
        str_cur->pts_per_pixel_rho.resize(0);
        std::copy(str_next->pts_per_pixel_rho.begin(),       str_next->pts_per_pixel_rho.end(),       str_cur->pts_per_pixel_rho.begin());
        str_cur->pts_per_pixel_index_valid.resize(0);
        std::copy(str_next->pts_per_pixel_index_valid.begin(), str_next->pts_per_pixel_index_valid.end(), str_cur->pts_per_pixel_index_valid.begin());

        str_next->img_restore_mask.copyTo(str_cur->img_restore_mask);
        str_next->img_restore_warp_mask.copyTo(str_cur->img_restore_warp_mask);
    }

    str_next->reset();
    {
        str_next_->img_rho               = cv::Mat::zeros(img_height_, img_width_, CV_32FC1);
        str_next_->img_index             = cv::Mat::zeros(img_height_, img_width_, CV_32SC1);
 
        str_next_->img_x                 = cv::Mat::zeros(img_height_, img_width_, CV_32FC1);
        str_next_->img_y                 = cv::Mat::zeros(img_height_, img_width_, CV_32FC1);
        str_next_->img_z                 = cv::Mat::zeros(img_height_, img_width_, CV_32FC1);
        str_next_->img_restore_warp_mask = cv::Mat::zeros(img_height_, img_width_, CV_32SC1);
    }

    // memcpy(&p0_pcl_test_, &p1, sizeof(pcl::PointCloud<pcl::PointXYZ>));
    p0_pcl_test_ = p1;

    str_cur_warped_->reset();
    {
        str_cur_warped_->img_rho               = cv::Mat::zeros(img_height_, img_width_, CV_32FC1);
        str_cur_warped_->img_index             = cv::Mat::zeros(img_height_, img_width_, CV_32SC1);
                   
        str_cur_warped_->img_x                 = cv::Mat::zeros(img_height_, img_width_, CV_32FC1);
        str_cur_warped_->img_y                 = cv::Mat::zeros(img_height_, img_width_, CV_32FC1);
        str_cur_warped_->img_z                 = cv::Mat::zeros(img_height_, img_width_, CV_32FC1);
        str_cur_warped_->img_restore_warp_mask = cv::Mat::zeros(img_height_, img_width_, CV_32SC1);
    }

    str_warpPointcloud_->reset();
    {
        str_warpPointcloud_->img_rho               = cv::Mat::zeros(img_height_, img_width_, CV_32FC1);
        str_warpPointcloud_->img_index             = cv::Mat::zeros(img_height_, img_width_, CV_32SC1);
                   
        str_warpPointcloud_->img_x                 = cv::Mat::zeros(img_height_, img_width_, CV_32FC1);
        str_warpPointcloud_->img_y                 = cv::Mat::zeros(img_height_, img_width_, CV_32FC1);
        str_warpPointcloud_->img_z                 = cv::Mat::zeros(img_height_, img_width_, CV_32FC1);
        str_warpPointcloud_->img_restore_warp_mask = cv::Mat::zeros(img_height_, img_width_, CV_32SC1);
    }
  
    groundPtsIdx_next_ = cv::Mat::zeros(img_height_, img_width_, CV_8UC1);

    velo_cur_.resize(0);
    ptr_cur_pts_warped_->resize(0);
    
    residual_ = cv::Mat::zeros(img_height_, img_width_, CV_32FC1);

    velo_xyz_.resize(0);
    pts_warpewd_->resize(0);
}

void MaplessDynamic::getUserSettingParameters(){
    // IMPLEMENT YOUR CODE FROM THIS LINE.

    // do something...

    // END YOUR CODE  

    img_height_ = 64/1;
    img_width_ = 4500/5+1;
    object_threshold_ = 30;

    alpha_ = 0.3;
    beta_ = 0.1;

    //initialize
    score_cnt_ = 0;

    //for test
    test_data_type_ = "KITTI";      
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

    std::cout<< "bin_num: "<<file_num <<" "<<"num_pts: "<<data_buf_[valid_cnt]->pcl_->size() <<std::endl;
    // std::cout<< "bin_num: "<<file_num <<" "<<"final pts: "<<data_buf_[file_num]->pcl_->at((data_buf_[file_num]->pcl_)->size()-2) <<std::endl;
    std::cout<< "valid_num: "<<valid_cnt <<" "<<"num_pts: "<<data_buf_[valid_cnt]->pcl_msg_->width <<std::endl;
    input.close();
    valid_cnt+=1;
}

void MaplessDynamic::genRangeImages(pcl::PointCloud<pcl::PointXYZ>& pcl_in1, StrRhoPts* str_in1)
{
    int h_factor = 5;
    int v_factor = 1;
    float azimuth_res = 0.08*h_factor;

    // generate range images
    float az_step = 1/azimuth_res; // 0.08 degrees step.
    int n_radial = 360*az_step+1;
    int n_ring = 64/v_factor;
    int n_pts = pcl_in1.size();

    calcuateRho(pcl_in1, str_in1);
    makeRangeImageAndPtsPerPixel(str_in1, n_pts, n_ring, n_radial, az_step);
    interpRangeImage(str_in1, n_ring, n_radial);
    interpPts(pcl_in1, str_in1, n_ring, n_radial);

    // int cnt1 = 0;
    // int cnt2 = 0;
    // int cnt3 = 0;
    // int cnt4 = 0;
    // float* ptr_img_rho = str_in1->img_rho.ptr<float>(0);
    // float *ptr_x = str_in1->img_x.ptr<float>(0);
    // float* ptr_y = str_in1->img_y.ptr<float>(0);
    // float* ptr_z = str_in1->img_z.ptr<float>(0);

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
    float twopi = 2*M_PI;
    float offset_theta = M_PI;

    int n_pts = pcl_in.size();
    float invrhocos = 0.0;
    float cospsi = 0.0;
    float sinpsi = 0.0;

    for (int i = 0; i < n_pts; ++i)
    {
        str_in->rho.push_back(NORM(pcl_in.points[i].x, pcl_in.points[i].y, pcl_in.points[i].z));
        // std::cout << NORM(pcl_in.points[i].x, pcl_in.points[i].y, pcl_in.points[i].z) <<std::endl;
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
            if (kk == (n_ring-1))
            {
                i_row = n_ring - 1;
            }
        }
        i_col = roundf(str_in->theta[i]*az_step*R2D);

        if ( (i_row > n_ring-1) || (i_row < 0) )
        {
            continue;
        }

        if (*(ptr_img_rho + i_row * n_col + i_col) == 0) //(str_in->img_rho.at<float>(i_row,i_col) == 0)
        {   
            *(ptr_img_rho + i_row * n_col + i_col) = str_in->rho[i];
            *(ptr_img_index + i_row * n_col + i_col) = i;
        }
        else if (*(ptr_img_rho + i_row * n_col + i_col) > str_in->rho[i])
        {
            *(ptr_img_rho + i_row * n_col + i_col) = str_in->rho[i];
            *(ptr_img_index + i_row * n_col + i_col) = i;
        }

        str_in->pts_per_pixel_n[(i_row)*n_col + i_col] += 1;

        str_in->pts_per_pixel_index[(i_row)*n_col + i_col].push_back(i);
        str_in->pts_per_pixel_rho[(i_row)*n_col + i_col].push_back(str_in->rho[i]);
        // str_in->pts_per_pixel_index[n_ring*(i_col)+i_row][str_in->pts_per_pixel_n[n_ring*(i_col)+i_row-1] -1] = i;
        // str_in->pts_per_pixel_rho[n_ring*(i_col)+i_row][str_in->pts_per_pixel_n[n_ring*(i_col)+i_row-1] -1] = str_in->rho[i];
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
    float* ptr_img_rho = str_in->img_rho.ptr<float>(0);
    int* ptr_img_index = str_in->img_index.ptr<int>(0);
    float* ptr_img_rho_new = img_rho_new.ptr<float>(0);
    int* ptr_img_restore_mask = str_in->img_restore_mask.ptr<int>(0);
    int n_col = str_in->img_rho.cols;
    int n_row = str_in->img_rho.rows;

    for (int i = 27; i < 32; i++)
    {
        for (int j = 0 + 2; j < (n_radial - 2); ++j)
        {
            if (*(ptr_img_rho+ i*n_col + j) == 0)
            {
                if ((*(ptr_img_rho + (i - 1) * n_col + j) != 0) && (*(ptr_img_rho + (i + 1) * n_col + j) != 0))
                {
                    if (fabsf32(*(ptr_img_rho + (i - 1) * n_col + j) - *(ptr_img_rho + (i + 1) * n_col + j)) < 0.1)
                    {
                        *(ptr_img_restore_mask + i * n_col + j) = 1;
                        *(ptr_img_rho_new + i * n_col + j) = (*(ptr_img_rho + (i - 1) * n_col + j) + *(ptr_img_rho + (i + 1) * n_col + j)) * 0.5;
                    }
                    else
                    {
                        *(ptr_img_restore_mask + i * n_col + j) = 10;
                        *(ptr_img_rho_new + i * n_col + j) = std::max(*(ptr_img_rho + (i - 1) * n_col + j), *(ptr_img_rho + (i + 1) * n_col + j));
                    }
                }

                else if ((*(ptr_img_rho + (i - 1) * n_col + j) != 0) && (*(ptr_img_rho + (i + 2) * n_col + j) != 0))
                {
                    if (fabsf32(*(ptr_img_rho + (i - 1) * n_col + j) - *(ptr_img_rho + (i + 2) * n_col + j)) < 0.1)
                    {
                        *(ptr_img_restore_mask + i * n_col + j) = 2;
                        *(ptr_img_restore_mask + (i + 1) * n_col + j) = 3;
                        *(ptr_img_rho_new + i * n_col + j) = *(ptr_img_rho + (i - 1) * n_col + j)*(0.6666667) + *(ptr_img_rho + (i + 2) * n_col + j)*(0.3333333);
                        *(ptr_img_rho_new + (i+1) * n_col + j) = *(ptr_img_rho + (i - 1) * n_col + j)*(0.3333333) + *(ptr_img_rho + (i + 2) * n_col + j)*(0.6666667);
                    }
                    else
                    {
                        *(ptr_img_restore_mask + i * n_col + j) = 20;
                        *(ptr_img_restore_mask + (i + 1) * n_col + j) = 30;
                        *(ptr_img_rho_new + i * n_col + j) = std::max(*(ptr_img_rho + (i - 1) * n_col + j), *(ptr_img_rho + (i + 2) * n_col + j));
                        *(ptr_img_rho_new + (i+1) * n_col + j) = std::max(*(ptr_img_rho + (i - 1) * n_col + j), *(ptr_img_rho + (i + 2) * n_col + j));
                    }
                }
            } // end if

            if ((*(ptr_img_rho + i * n_col + (j - 1)) != 0) && (*(ptr_img_rho + i * n_col + (j + 1)) != 0))
            {
                if (fabsf32(*(ptr_img_rho + i * n_col + (j - 1)) - *(ptr_img_rho + i * n_col + (j + 1))) < 0.05)
                {
                    *(ptr_img_restore_mask + i * n_col + j) = 4;
                    *(ptr_img_rho_new + i * n_col + j) = (*(ptr_img_rho + i * n_col + (j - 1)) + *(ptr_img_rho + i * n_col + (j + 1))) * 0.5;
                }
            }

            else if ((*(ptr_img_rho + i * n_col + (j - 1)) != 0) && (*(ptr_img_rho + i * n_col + (j + 2)) != 0))
            {
                if (fabsf32(*(ptr_img_rho + i * n_col + (j - 1)) - *(ptr_img_rho + i * n_col + (j + 2))) < 0.05)
                {
                    *(ptr_img_restore_mask + i * n_col + j) = 5;
                    *(ptr_img_restore_mask + i * n_col + (j + 1)) = 6;
                    *(ptr_img_rho_new + i * n_col + j) = *(ptr_img_rho + i * n_col + (j - 1)) * (0.6666667) + *(ptr_img_rho + i * n_col + (j + 2)) * (0.3333333);
                    *(ptr_img_rho_new + i * n_col + (j + 1)) = *(ptr_img_rho + i * n_col + (j - 1)) * (0.3333333) + *(ptr_img_rho + i * n_col + (j + 2)) * (0.6666667);
                }
            }

        } // end col

    } // end row

    img_rho_new.copyTo(str_in->img_rho);
}

void MaplessDynamic::interpPts(pcl::PointCloud<pcl::PointXYZ>& pcl_in, StrRhoPts* str_in, int n_ring, int n_radial)
{
    
    float* ptr_img_rho = str_in->img_rho.ptr<float>(0);
    int* ptr_img_index = str_in->img_index.ptr<int>(0);
    float* ptr_img_x = str_in->img_x.ptr<float>(0);
    float* ptr_img_y = str_in->img_y.ptr<float>(0);
    float* ptr_img_z = str_in->img_z.ptr<float>(0);
    int* ptr_img_restore_mask = str_in->img_restore_mask.ptr<int>(0);

    int n_col = str_in->img_rho.cols;
    int n_row = str_in->img_rho.rows;

    for (int i = 0; i < n_ring; ++i)
    {
        for (int j = 0; j < n_radial; ++j)
        {
            if (str_in->pts_per_pixel_rho[i * n_col + j].size() > 0)
            {
                
                for (int k = 0; k < (str_in->pts_per_pixel_rho[i * n_col + j].size()); ++k)
                {
                    
                    if (std::abs((str_in->pts_per_pixel_rho[i * n_col + j][k] - *(ptr_img_rho + i * n_col + j))) < 0.5)
                    {
                        str_in->pts_per_pixel_index_valid[i*n_col+j].push_back(str_in->pts_per_pixel_index[i * n_col + j][k]);
                    }
                }
            } // end if
            if (*(ptr_img_index+i*n_col+j)!=0 )
            {
                *(ptr_img_x + i * n_col + j) = pcl_in[*(ptr_img_index + i * n_col + j)].x;
                *(ptr_img_y + i * n_col + j) = pcl_in[*(ptr_img_index + i * n_col + j)].y;
                *(ptr_img_z + i * n_col + j) = pcl_in[*(ptr_img_index + i * n_col + j)].z;
            }

            if (*(ptr_img_restore_mask + i * n_col + j) == 1)
            {
                *(ptr_img_x + i * n_col + j) = 0.5 * (pcl_in[*(ptr_img_index + (i - 1) * n_col + j)].x + pcl_in[*(ptr_img_index + (i + 1) * n_col + j)].x);
                *(ptr_img_y + i * n_col + j) = 0.5 * (pcl_in[*(ptr_img_index + (i - 1) * n_col + j)].y + pcl_in[*(ptr_img_index + (i + 1) * n_col + j)].y);
                *(ptr_img_z + i * n_col + j) = 0.5 * (pcl_in[*(ptr_img_index + (i - 1) * n_col + j)].z + pcl_in[*(ptr_img_index + (i + 1) * n_col + j)].z);
            }
            else if (*(ptr_img_restore_mask + i * n_col + j) == 10)
            {
                if ((*(ptr_img_rho + (i - 1) * n_col + j) > *(ptr_img_rho + (i + 1) * n_col + j)))
                {
                    *(ptr_img_x + i * n_col + j) = pcl_in[*(ptr_img_index + (i-1) * n_col + j)].x;
                    *(ptr_img_y + i * n_col + j) = pcl_in[*(ptr_img_index + (i-1) * n_col + j)].y;
                    *(ptr_img_z + i * n_col + j) = pcl_in[*(ptr_img_index + (i-1) * n_col + j)].z;
                }
                else
                {
                    *(ptr_img_x + i * n_col + j) = pcl_in[*(ptr_img_index + (i+1) * n_col + j)].x;
                    *(ptr_img_y + i * n_col + j) = pcl_in[*(ptr_img_index + (i+1) * n_col + j)].y;
                    *(ptr_img_z + i * n_col + j) = pcl_in[*(ptr_img_index + (i+1) * n_col + j)].z;
                }
            }
            else if (*(ptr_img_restore_mask + i * n_col + j) == 2)
            {
                *(ptr_img_x + i * n_col + j) = (0.6666667) * pcl_in[*(ptr_img_index + (i - 1) * n_col + j)].x + (0.3333333) * pcl_in[*(ptr_img_index + (i + 2) * n_col + j)].x;
                *(ptr_img_y + i * n_col + j) = (0.6666667) * pcl_in[*(ptr_img_index + (i - 1) * n_col + j)].y + (0.3333333) * pcl_in[*(ptr_img_index + (i + 2) * n_col + j)].y;
                *(ptr_img_z + i * n_col + j) = (0.6666667) * pcl_in[*(ptr_img_index + (i - 1) * n_col + j)].z + (0.3333333) * pcl_in[*(ptr_img_index + (i + 2) * n_col + j)].z;
            }
            else if (*(ptr_img_restore_mask + i * n_col + j) == 20)
            {
                if ((*(ptr_img_rho + (i - 1) * n_col + j) > *(ptr_img_rho + (i + 2) * n_col + j)))
                {
                    *(ptr_img_x + i * n_col + j) = pcl_in[*(ptr_img_index + (i-1) * n_col + j)].x;
                    *(ptr_img_y + i * n_col + j) = pcl_in[*(ptr_img_index + (i-1) * n_col + j)].y;
                    *(ptr_img_z + i * n_col + j) = pcl_in[*(ptr_img_index + (i-1) * n_col + j)].z;
                }
                else
                {
                    *(ptr_img_x + i * n_col + j) = pcl_in[*(ptr_img_index + (i+2) * n_col + j)].x;
                    *(ptr_img_y + i * n_col + j) = pcl_in[*(ptr_img_index + (i+2) * n_col + j)].y;
                    *(ptr_img_z + i * n_col + j) = pcl_in[*(ptr_img_index + (i+2) * n_col + j)].z;
                }
            }
            else if (*(ptr_img_restore_mask + i * n_col + j) == 3)
            {
                *(ptr_img_x + i * n_col + j) = (0.3333333) * pcl_in[*(ptr_img_index + (i - 2) * n_col + j)].x + (0.6666667) * pcl_in[*(ptr_img_index + (i + 1) * n_col + j)].x;
                *(ptr_img_y + i * n_col + j) = (0.3333333) * pcl_in[*(ptr_img_index + (i - 2) * n_col + j)].y + (0.6666667) * pcl_in[*(ptr_img_index + (i + 1) * n_col + j)].y;
                *(ptr_img_z + i * n_col + j) = (0.3333333) * pcl_in[*(ptr_img_index + (i - 2) * n_col + j)].z + (0.6666667) * pcl_in[*(ptr_img_index + (i + 1) * n_col + j)].z;
            }
            else if (*(ptr_img_restore_mask + i * n_col + j) == 30)
            {
                if ((*(ptr_img_rho + (i - 2) * n_col + j) > *(ptr_img_rho + (i + 1) * n_col + j)))
                {
                    *(ptr_img_x + i * n_col + j) = pcl_in[*(ptr_img_index + (i-2) * n_col + j)].x;
                    *(ptr_img_y + i * n_col + j) = pcl_in[*(ptr_img_index + (i-2) * n_col + j)].y;
                    *(ptr_img_z + i * n_col + j) = pcl_in[*(ptr_img_index + (i-2) * n_col + j)].z;
                }
                else
                {
                    *(ptr_img_x + i * n_col + j) = pcl_in[*(ptr_img_index + (i+1) * n_col + j)].x;
                    *(ptr_img_y + i * n_col + j) = pcl_in[*(ptr_img_index + (i+1) * n_col + j)].y;
                    *(ptr_img_z + i * n_col + j) = pcl_in[*(ptr_img_index + (i+1) * n_col + j)].z;
                }
            }
            else if (*(ptr_img_restore_mask + i * n_col + j) == 4)
            {
                *(ptr_img_x + i * n_col + j) = 0.5 * (pcl_in[*(ptr_img_index + i * n_col + (j-1))].x + pcl_in[*(ptr_img_index + i * n_col + (j+1))].x);
                *(ptr_img_y + i * n_col + j) = 0.5 * (pcl_in[*(ptr_img_index + i * n_col + (j-1))].y + pcl_in[*(ptr_img_index + i * n_col + (j+1))].y);
                *(ptr_img_z + i * n_col + j) = 0.5 * (pcl_in[*(ptr_img_index + i * n_col + (j-1))].z + pcl_in[*(ptr_img_index + i * n_col + (j+1))].z);
            }
            else if (*(ptr_img_restore_mask + i * n_col + j) == 5)
            {
                *(ptr_img_x + i * n_col + j) = (0.6666667) * pcl_in[*(ptr_img_index + i * n_col + (j-1))].x + (0.3333333)*pcl_in[*(ptr_img_index + i * n_col + (j+2))].x;
                *(ptr_img_y + i * n_col + j) = (0.6666667) * pcl_in[*(ptr_img_index + i * n_col + (j-1))].y + (0.3333333)*pcl_in[*(ptr_img_index + i * n_col + (j+2))].y;
                *(ptr_img_z + i * n_col + j) = (0.6666667) * pcl_in[*(ptr_img_index + i * n_col + (j-1))].z + (0.3333333)*pcl_in[*(ptr_img_index + i * n_col + (j+2))].z;
            }
            else if (*(ptr_img_restore_mask + i * n_col + j) == 6)
            {
                *(ptr_img_x + i * n_col + j) = (0.3333333) * pcl_in[*(ptr_img_index + i * n_col + (j-2))].x + (0.6666667)*pcl_in[*(ptr_img_index + i * n_col + (j+1))].x;
                *(ptr_img_y + i * n_col + j) = (0.3333333) * pcl_in[*(ptr_img_index + i * n_col + (j-2))].y + (0.6666667)*pcl_in[*(ptr_img_index + i * n_col + (j+1))].y;
                *(ptr_img_z + i * n_col + j) = (0.3333333) * pcl_in[*(ptr_img_index + i * n_col + (j-2))].z + (0.6666667)*pcl_in[*(ptr_img_index + i * n_col + (j+1))].z;
            }

        }     // end for j
    }         // end for i
}

void MaplessDynamic::dR_warpPointcloud(pcl::PointCloud<pcl::PointXYZ>& p0, Pose& T01, int cnt_data)
{
    int n_row = str_next_->img_rho.rows;
    int n_col = str_next_->img_rho.cols;
    int n_ring = n_row;
    int n_radial = n_col;
    float* ptr_img_x = str_cur_->img_x.ptr<float>(0);
    float* ptr_img_y = str_cur_->img_y.ptr<float>(0);
    float* ptr_img_z = str_cur_->img_z.ptr<float>(0);
    // int cnt = 0;

    // representative pts
    for (int i = 0; i < n_row; ++i)
    {
        for (int j = 0; j < n_col; ++j)
        {

            if ((*(ptr_img_x + i * n_col + j) > 10.0) || (*(ptr_img_x + i * n_col + j) < -10.0) || (*(ptr_img_y + i * n_col + j) > 10.0) || (*(ptr_img_y + i * n_col + j) < -10.0))
            {
                velo_cur_.push_back(pcl::PointXYZ(*(ptr_img_x + i * n_col + j), *(ptr_img_y + i * n_col + j), *(ptr_img_z + i * n_col + j)));
            }
        }
    }
    

    // Far pts are warped by the original pts
    for (int i=0; i<p0.size(); ++i)
    {
        if( (p0[i].x < 10) && (p0[i].x > -10.0) && (p0[i].y < 10.0) && (p0[i].y > -10.0) )
        {
            velo_cur_.push_back(p0[i]);
        }
    }

    // compensate zero in current rho image for warping
    // timer::tic();
    compensateCurRhoZeroWarp(str_cur_, n_ring, n_radial, v_angle_, velo_cur_);
    // double dt_slam = timer::toc(); // milliseconds
    // ROS_INFO_STREAM("elapsed time for 'compensateCurRhoZeroWarp' :" << dt_slam << " [ms]");
    // exit(0);
    pcl::transformPointCloud(velo_cur_, *ptr_cur_pts_warped_, T01);
    
    // current warped image
    genRangeImages(*(ptr_cur_pts_warped_), str_cur_warped_);

    // fill range image using interpolation
    interpRangeImageMin(str_cur_warped_, n_ring, n_radial);

    // fill pts corresponding to filled range image (no affect the original pts)
    interpPtsWarp(str_cur_warped_, n_ring, n_radial);

    // calculate occlusions
    cv::subtract(str_cur_warped_->img_rho, str_next_->img_rho, residual_); 
    // residual_ = str_cur_warped_->img_rho - str_next_->img_rho;    
}

void MaplessDynamic::compensateCurRhoZeroWarp(StrRhoPts* str_cur_, int n_ring, int n_radial, std::vector<float>& v_angle_, pcl::PointCloud<pcl::PointXYZ>& velo_cur_)
{
    int n_row = n_ring;
    int n_col = n_radial;
    float left_dir_rho = 0;
    float right_dir_rho = 0;
    float up_dir_rho = 0;
    float down_dir_rho = 0;
    float* ptr_img_rho = str_cur_->img_rho.ptr<float>(0);

    int cnt_left = 1;
    int cnt_right = 1;
    int cnt_up = 1;
    int cnt_down = 1;

    float min = 0.0 ;
    int min_index = 0;

    std::vector<float> four_dir;
    four_dir.resize(4);

    float new_phi = 0.0;
    float new_theta = 0.0;

    for (int i=0+1; i<n_ring-1 ; ++i)
    {
        for (int j=0+1; j<n_radial-1 ; ++j)
        {
            left_dir_rho = 0.0;
            right_dir_rho = 0.0;
            up_dir_rho = 0.0;
            down_dir_rho = 0.0;

            if (*(ptr_img_rho + i * n_col + j) == 0)
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
                        left_dir_rho = *(ptr_img_rho + i * n_col + (j - cnt_left));
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
                        right_dir_rho = *(ptr_img_rho + i * n_col + (j + cnt_right));
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
                        up_dir_rho = *(ptr_img_rho + (i - cnt_up) * n_col + j);
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
                        down_dir_rho = *(ptr_img_rho + (i + cnt_down) * n_col + j);
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
                        new_phi = v_angle_[i] * D2R;
                        new_theta = 0.4 * j *D2R;
                        for (int m = 0; m < 5; ++m)
                        {
                            for (int p = 0; p < 5; ++p)
                            {
                                velo_cur_.push_back(pcl::PointXYZ(-min * cosf(new_phi + (float)(m - 2) * D2R) * cosf(new_theta + (float)(p - 2) * 0.08 * D2R),
                                                                  -min * cosf(new_phi + (float)(m - 2) * D2R) * sinf(new_theta + ((float)p - 2) * 0.08 * D2R),
                                                                   min * sinf(new_phi + (float)(m - 2) * D2R)));
                            }
                        }
                    } // end if
            } // end if
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
        for (int j = 0 + 2; j < (n_radial - 2); ++j)
        {
            if (*(ptr_img_rho + i * n_col + j) == 0)
            {
                if (*(ptr_img_rho + (i - 1) * n_col + j) != 0 && *(ptr_img_rho + (i + 1) * n_col + j) != 0)
                {
                    if (fabsf32(*(ptr_img_rho + (i - 1) * n_col + j) - *(ptr_img_rho + (i + 1) * n_col + j)) < 0.1)
                    {
                        *(ptr_img_restore_warp_mask + i * n_col + j) = 1;
                        *(ptr_img_rho_new + i * n_col + j) = (*(ptr_img_rho + (i - 1) * n_col + j) + *(ptr_img_rho + (i + 1) * n_col + j)) * 0.5;
                    }
                    else
                    {
                        *(ptr_img_restore_warp_mask + i * n_col + j) = 10;
                        *(ptr_img_rho_new + i * n_col + j) = std::min(*(ptr_img_rho + (i - 1) * n_col + j), *(ptr_img_rho + (i + 1) * n_col + j));
                    }
                }
                else if (*(ptr_img_rho + (i - 1) * n_col + j) != 0 && *(ptr_img_rho + (i + 2) * n_col + j) != 0)
                {
                    if (fabsf32(*(ptr_img_rho + (i - 1) * n_col + j) - *(ptr_img_rho + (i + 2) * n_col + j)) < 0.1)
                    {
                        *(ptr_img_restore_warp_mask + i * n_col + j) = 2;
                        *(ptr_img_restore_warp_mask + (i + 1) * n_col + j) = 3;
                        *(ptr_img_rho_new + i * n_col + j) = *(ptr_img_rho + (i - 1) * n_col + j) * (0.6666667) + *(ptr_img_rho + (i + 2) * n_col + j) * (0.3333333);
                        *(ptr_img_rho_new + (i + 1) * n_col + j) = *(ptr_img_rho + (i - 1) * n_col + j) * (0.3333333) + *(ptr_img_rho + (i + 2) * n_col + j) * (0.6666667);
                    }
                    else
                    {
                        *(ptr_img_restore_warp_mask + i * n_col + j) = 20;
                        *(ptr_img_restore_warp_mask + (i + 1) * n_col + j) = 30;
                        *(ptr_img_rho_new + i * n_col + j) = std::min(*(ptr_img_rho + (i - 1) * n_col + j), *(ptr_img_rho + (i + 2) * n_col + j));
                        *(ptr_img_rho_new + (i + 1) * n_col + j) = std::min(*(ptr_img_rho + (i - 1) * n_col + j), *(ptr_img_rho + (i + 2) * n_col + j));
                    }
                }

                if (*(ptr_img_rho + i * n_col + (j - 1)) != 0 && *(ptr_img_rho + i * n_col + (j + 1)) != 0)
                {
                    if (fabsf32(*(ptr_img_rho + i * n_col + (j - 1)) - *(ptr_img_rho + i * n_col + (j + 1))) < 0.05)
                    {
                        *(ptr_img_restore_warp_mask + i * n_col + j) = 4;
                        *(ptr_img_rho_new + i * n_col + j) = (*(ptr_img_rho + i * n_col + (j - 1)) + *(ptr_img_rho + i * n_col + (j + 1))) * 0.5;
                    }
                }
                else if (*(ptr_img_rho + i * n_col + (j - 1)) != 0 && *(ptr_img_rho + i * n_col + (j + 2)) != 0)
                {
                    if (fabsf32(*(ptr_img_rho + i * n_col + (j - 1)) - *(ptr_img_rho + i * n_col + (j + 2))) < 0.05)
                    {
                        *(ptr_img_restore_warp_mask + i * n_col + j) = 5;
                        *(ptr_img_restore_warp_mask + i * n_col + (j + 1)) = 6;
                        *(ptr_img_rho_new + i * n_col + j) = *(ptr_img_rho + i * n_col + (j - 1)) * (0.6666667) + *(ptr_img_rho + i * n_col + (j + 2)) * (0.3333333);
                        *(ptr_img_rho_new + i * n_col + (j + 1)) = *(ptr_img_rho + i * n_col + (j - 1)) * (0.3333333) + *(ptr_img_rho + i * n_col + (j + 2)) * (0.6666667);
                    }
                }
            } // end if

        } // end col

    } // end row

    img_rho_new.copyTo(str_in->img_rho);
}

void MaplessDynamic::interpPtsWarp(StrRhoPts* str_in, int n_ring, int n_radial)
{
    int n_col = n_radial;
    int n_row = n_ring;
    int* ptr_img_index = str_in->img_index.ptr<int>(0);
    float* ptr_img_rho = str_in->img_rho.ptr<float>(0);
    float* ptr_img_x = str_in->img_x.ptr<float>(0);
    float* ptr_img_y = str_in->img_y.ptr<float>(0);
    float* ptr_img_z = str_in->img_z.ptr<float>(0);
    int* ptr_img_restore_warp_mask = str_in->img_restore_warp_mask.ptr<int>(0);
    

    for (int i=0+2; i<n_row-2; ++i)
    {
        for (int j=0+2; j<n_col-2; ++j)
        {
            if (*(ptr_img_restore_warp_mask + i * n_col + j) == 1)
            {
                *(ptr_img_x + i * n_col + j) = 0.5 * (*(ptr_img_x + (i - 1) * n_col + j) + *(ptr_img_x + (i + 1) * n_col + j));
                *(ptr_img_y + i * n_col + j) = 0.5 * (*(ptr_img_y + (i - 1) * n_col + j) + *(ptr_img_y + (i + 1) * n_col + j));
                *(ptr_img_z + i * n_col + j) = 0.5 * (*(ptr_img_z + (i - 1) * n_col + j) + *(ptr_img_z + (i + 1) * n_col + j));
            }
            else if (*(ptr_img_restore_warp_mask + i * n_col + j) == 10)
            {
                if ((*(ptr_img_rho + (i - 1) * n_col + j) < *(ptr_img_rho + (i + 1) * n_col + j)))
                {
                    *(ptr_img_x + i * n_col + j) = *(ptr_img_x + (i-1) * n_col + j);
                    *(ptr_img_y + i * n_col + j) = *(ptr_img_y + (i-1) * n_col + j);
                    *(ptr_img_z + i * n_col + j) = *(ptr_img_z + (i-1) * n_col + j);
                }
                else
                {
                    *(ptr_img_x + i * n_col + j) = *(ptr_img_x + (i+1) * n_col + j);
                    *(ptr_img_y + i * n_col + j) = *(ptr_img_y + (i+1) * n_col + j);
                    *(ptr_img_z + i * n_col + j) = *(ptr_img_z + (i+1) * n_col + j);
                }
            }
            else if (*(ptr_img_restore_warp_mask + i * n_col + j) == 2)
            {
                *(ptr_img_x + i * n_col + j) = (0.6666667) * (*(ptr_img_x + (i - 1) * n_col + j)) + (0.3333333) * (*(ptr_img_x + (i + 2) * n_col + j));
                *(ptr_img_y + i * n_col + j) = (0.6666667) * (*(ptr_img_y + (i - 1) * n_col + j)) + (0.3333333) * (*(ptr_img_y + (i + 2) * n_col + j));
                *(ptr_img_z + i * n_col + j) = (0.6666667) * (*(ptr_img_z + (i - 1) * n_col + j)) + (0.3333333) * (*(ptr_img_z + (i + 2) * n_col + j));
            }
            else if (*(ptr_img_restore_warp_mask + i * n_col + j) == 20)
            {
                if ((*(ptr_img_rho + (i - 1) * n_col + j) < *(ptr_img_rho + (i + 2) * n_col + j)))
                {
                    *(ptr_img_x + i * n_col + j) = *(ptr_img_x + (i-1) * n_col + j);
                    *(ptr_img_y + i * n_col + j) = *(ptr_img_y + (i-1) * n_col + j);
                    *(ptr_img_z + i * n_col + j) = *(ptr_img_z + (i-1) * n_col + j);
                }
                else
                {
                    *(ptr_img_x + i * n_col + j) = *(ptr_img_x + (i+2) * n_col + j);
                    *(ptr_img_y + i * n_col + j) = *(ptr_img_y + (i+2) * n_col + j);
                    *(ptr_img_z + i * n_col + j) = *(ptr_img_z + (i+2) * n_col + j);
                }
            }
            else if (*(ptr_img_restore_warp_mask + i * n_col + j) == 3)
            {
                *(ptr_img_x + i * n_col + j) = (0.3333333) * (*(ptr_img_x + (i - 2) * n_col + j)) + (0.6666667) * (*(ptr_img_x + (i + 1) * n_col + j));
                *(ptr_img_y + i * n_col + j) = (0.3333333) * (*(ptr_img_y + (i - 2) * n_col + j)) + (0.6666667) * (*(ptr_img_y + (i + 1) * n_col + j));
                *(ptr_img_z + i * n_col + j) = (0.3333333) * (*(ptr_img_z + (i - 2) * n_col + j)) + (0.6666667) * (*(ptr_img_z + (i + 1) * n_col + j));
            }
            else if (*(ptr_img_restore_warp_mask + i * n_col + j) == 30)
            {
                if ((*(ptr_img_rho + (i - 2) * n_col + j) < *(ptr_img_rho + (i + 1) * n_col + j)))
                {
                    *(ptr_img_x + i * n_col + j) = *(ptr_img_x + (i-2) * n_col + j);
                    *(ptr_img_y + i * n_col + j) = *(ptr_img_y + (i-2) * n_col + j);
                    *(ptr_img_z + i * n_col + j) = *(ptr_img_z + (i-2) * n_col + j);
                }
                else
                {
                    *(ptr_img_x + i * n_col + j) = *(ptr_img_x + (i+1) * n_col + j);
                    *(ptr_img_y + i * n_col + j) = *(ptr_img_y + (i+1) * n_col + j);
                    *(ptr_img_z + i * n_col + j) = *(ptr_img_z + (i+1) * n_col + j);
                }
            }
            else if (*(ptr_img_restore_warp_mask + i * n_col + j) == 4)
            {
                *(ptr_img_x + i * n_col + j) = 0.5 * (*(ptr_img_x + i * n_col + (j - 1)) + (*(ptr_img_x + i * n_col + (j + 1))));
                *(ptr_img_y + i * n_col + j) = 0.5 * (*(ptr_img_y + i * n_col + (j - 1)) + (*(ptr_img_y + i * n_col + (j + 1))));
                *(ptr_img_z + i * n_col + j) = 0.5 * (*(ptr_img_z + i * n_col + (j - 1)) + (*(ptr_img_z + i * n_col + (j + 1))));
            }
            else if (*(ptr_img_restore_warp_mask + i * n_col + j) == 5)
            {
                *(ptr_img_x + i * n_col + j) = (0.6666667) * (*(ptr_img_x + i * n_col + (j - 1))) + (0.3333333) * (*(ptr_img_x + i * n_col + (j + 2)));
                *(ptr_img_y + i * n_col + j) = (0.6666667) * (*(ptr_img_y + i * n_col + (j - 1))) + (0.3333333) * (*(ptr_img_y + i * n_col + (j + 2)));
                *(ptr_img_z + i * n_col + j) = (0.6666667) * (*(ptr_img_z + i * n_col + (j - 1))) + (0.3333333) * (*(ptr_img_z + i * n_col + (j + 2)));
            }
            else if (*(ptr_img_restore_warp_mask + i * n_col + j) == 6)
            {
                *(ptr_img_x + i * n_col + j) = (0.3333333) * (*(ptr_img_x + i * n_col + (j - 2))) + (0.6666667) * (*(ptr_img_x + i * n_col + (j + 1)));
                *(ptr_img_y + i * n_col + j) = (0.3333333) * (*(ptr_img_y + i * n_col + (j - 2))) + (0.6666667) * (*(ptr_img_y + i * n_col + (j + 1)));
                *(ptr_img_z + i * n_col + j) = (0.3333333) * (*(ptr_img_z + i * n_col + (j - 2))) + (0.6666667) * (*(ptr_img_z + i * n_col + (j + 1)));
            }
        } // end for j
    } // end for i
}

void MaplessDynamic::warpPointcloud(StrRhoPts *str_cur_, const Pose &T01, /*output*/ cv::Mat &mat_in, int cnt_data)
{
    int n_row = str_next_->img_rho.rows;
    int n_col = str_next_->img_rho.cols;
    float *ptr_mat_in = mat_in.ptr<float>(0);
    float *ptr_img_x = str_cur_->img_x.ptr<float>(0);
    float *ptr_img_y = str_cur_->img_y.ptr<float>(0);
    float *ptr_img_z = str_cur_->img_z.ptr<float>(0);
    std::vector<float> I_vec1;
    I_vec1.reserve(n_row*n_col);
    bool isempty_flag = 0;
    float *ptr_img_rho = str_cur_->img_rho.ptr<float>(0);
    cv::Mat mat_out = cv::Mat::zeros(img_height_, img_width_, CV_32FC1);
    int *ptr_warp_img_index = str_warpPointcloud_->img_index.ptr<int>(0);
    float *ptr_mat_out = mat_out.ptr<float>(0);

    // if (cnt_data==2)
    // {
    //     std::cout<<velo_xyz_.size()<<std::endl;
    //     exit(0);
    // }

    for (int i = 0; i<n_row; ++i)
    {
        for (int j=0; j<n_col; ++j)
        {
            if (*(ptr_mat_in + i * n_col + j) != 0 && *(ptr_img_rho + i * n_col + j) != 0)
            {
                velo_xyz_.push_back(pcl::PointXYZ(*(ptr_img_x + i * n_col + j), *(ptr_img_y + i * n_col + j), *(ptr_img_z + i * n_col + j)));

                I_vec1.push_back(*(ptr_mat_in + i * n_col + j));
                isempty_flag = 1;
            }
        } //end for i
    } // end for j

    if (isempty_flag ==0)
    {
        return;
    }

    pcl::transformPointCloud(velo_xyz_, *pts_warpewd_, T01);

    genRangeImages(*pts_warpewd_, str_warpPointcloud_);

    for (int i = 0; i<n_row; ++i)
    {
        for (int j=0; j<n_col; ++j)
        {
            if (*(ptr_warp_img_index + i * n_col + j) != 0)
            {
                *(ptr_mat_out + i * n_col + j) = I_vec1[*(ptr_warp_img_index + i * n_col + j)];
            } // end if
        } // end for j
    } //end for i

    // ptr_mat_out = ptr_mat_in;
    mat_out.copyTo(mat_in);
}

void MaplessDynamic::filterOutAccumdR(StrRhoPts* str_next_, StrRhoPts* str_cur_warped_,cv::Mat& accumulated_dRdt_,cv::Mat& accumulated_dRdt_score_,cv::Mat& residual_)
{
    int n_row = str_next_->img_rho.rows;
    int n_col = str_next_->img_rho.cols;
    float coef_accum_w[2] = {0.5, 0.9};
    float* ptr_accumulated_dRdt_ = accumulated_dRdt_.ptr<float>(0);
    float* ptr_next_img_rho = str_next_->img_rho.ptr<float>(0);
    float* ptr_cur_warped_img_rho = str_cur_warped_->img_rho.ptr<float>(0);
    float* ptr_residual_ = residual_.ptr<float>(0);

    // Accumulate the occlusion
    for (int i = 0; i<n_row; ++i)
    {
        for (int j=0; j<n_col; ++j)
        {
            if (*(ptr_next_img_rho+i*n_col+j)<10)
            {
                *(ptr_accumulated_dRdt_ + i * n_col + j) = coef_accum_w[0] * (*(ptr_accumulated_dRdt_ + i * n_col + j)) + *(ptr_residual_ + i * n_col + j);
            }
            else // >10
            {
                *(ptr_accumulated_dRdt_ + i * n_col + j) = coef_accum_w[1] * (*(ptr_accumulated_dRdt_ + i * n_col + j)) + *(ptr_residual_ + i * n_col + j);
            }            
        }
    }

    // Extract candidate for objects
    for (int i = 0; i<n_row; ++i)
    {
        for (int j = 0; j < n_col; ++j)
        {
            if ( (*(ptr_next_img_rho + i * n_col + j) > 40) 
            || (*(ptr_accumulated_dRdt_ + i * n_col + j) < alpha_ * (*(ptr_next_img_rho + i * n_col + j)))
            ||  (*(ptr_next_img_rho + i * n_col + j) == 0) 
            || (*(ptr_cur_warped_img_rho + i * n_col + j) == 0)
            || (*(ptr_residual_ + i * n_col + j) < (- beta_ * (*(ptr_next_img_rho + i * n_col + j)))) )
            {
                *(ptr_accumulated_dRdt_ + i * n_col + j) = 0;
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

    cv::Mat object_label = cv::Mat::zeros(img_height_, img_width_, CV_32SC1);

    for (int i=0; i<n_row; ++i)
    {
        for (int j=0; j<n_col; ++j)
        {
            if (*(ptr_accumulated_dRdt + i * n_col + j) > 0)
            {
                *(ptr_object_mask+i*n_col+j) = 255;
                cnt += 1;
            }
        }
    }
    int n_label = cv::connectedComponents(object_mask, object_label, 8);
    int* ptr_object_label = object_label.ptr<int>(0);
    // cv::imshow("accumulated_dRdt", object_label);
    // cv::waitKey(0);
    // std::cout << cnt << std::endl;
    // exit(0);
    if (n_label == 0)
    {
        return;
    }

    std::vector<int> object_row;
    std::vector<int> object_col;
    std::vector<float> object_rho_roi;
    object_row.reserve(10000);
    object_col.reserve(10000);
    object_rho_roi.reserve(10000);
    cv::Mat object_rho_mat = cv::Mat::zeros(img_height_,img_width_, CV_32FC1);
    float *ptr_object_rho_mat = object_rho_mat.ptr<float>(0);
    std::vector<float> max_his_object_rho_roi;
    max_his_object_rho_roi.reserve(10000);

    std::vector<int> disconti_row;
    std::vector<int> disconti_col;
    disconti_row.reserve(10000);
    disconti_col.reserve(10000);

    std::vector<int> diff_object_area_bw_disconti_row;
    std::vector<int> diff_object_area_bw_disconti_col;
    diff_object_area_bw_disconti_row.reserve(10000);
    diff_object_area_bw_disconti_col.reserve(10000);

    std::vector<float> diff_z;
    diff_z.reserve(10000);

    cv::MatND histogram;
    for (int object_idx = 0; object_idx < n_label; ++object_idx)
    {
        if (object_idx==0) //background
        {
            continue;
        }

        object_row.clear();
        object_col.clear();
        object_rho_roi.clear();
        max_his_object_rho_roi.clear();
        disconti_row.clear();
        disconti_col.clear();
        diff_object_area_bw_disconti_row.clear();
        diff_object_area_bw_disconti_col.clear();
        diff_z.clear();
        for (int i=0; i<n_row; ++i)
        {
            for (int j=0; j<n_col; ++j)
            {
                if(*(ptr_object_label+i*n_col+j)==object_idx)
                {
                    object_row.push_back(i);
                    object_col.push_back(j);
                    object_rho_roi.push_back(*(ptr_accumulated_dRdt + i * n_col + j));
                    *(ptr_object_rho_mat + i * n_col + j) = *(ptr_accumulated_dRdt + i * n_col + j);
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
            float his_range_max = *max_element(object_rho_roi.begin(), object_rho_roi.end());
            float his_range_min = *min_element(object_rho_roi.begin(), object_rho_roi.end());
            float his_range[] = {his_range_min, his_range_max};
            const int* channel_numbers = {0};
            const float* his_ranges= his_range;
            int number_bins = 50;
            std::cout<<object_rho_roi.size()<<std::endl;
            std::cout<<his_range[0]<<std::endl;
            std::cout<<his_range[1]<<std::endl;
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

            for (int p = 0; p<object_rho_roi.size(); ++p)
            {
                if (object_rho_roi[p]>bin_range_min && object_rho_roi[p]<bin_range_max)
                {
                    max_his_object_rho_roi.push_back(object_rho_roi[p]);
                }
            }
            float max_his_average = 0.0;
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
                diff_z.push_back(*(ptr_next_img_z+diff_object_area_bw_disconti_row[i]*n_col+diff_object_area_bw_disconti_col[i]));
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
                std_diff_z += powf((diff_z[i]-mean_diff_z),2.0);
            }
            std_diff_z = (1.0/((float)diff_z.size()-1.0)*std_diff_z);
            if (std_diff_z < 0.07*0.07)
            {
                for (int i = 0; i < diff_object_area_bw_disconti_row.size(); ++i)
                {
                    *(ptr_accumulated_dRdt + diff_object_area_bw_disconti_row[i] * n_col + diff_object_area_bw_disconti_col[i]) = 0;
                }
            }

            // std::cout<<"   ====================    " <<std::endl;
            // std::cout<<histogram<<std::endl;
            // std::cout<<max_idx<<std::endl;
            // std::cout<<bin_range_min<<" " << bin_range_max << std::endl;
            // std::cout<<max_his_average<< std::endl;
            // std::cout<<(float)max_his_object_rho_roi.size() << std::endl;


        }
    } // end for object_idx
    // cv::imshow("accumulated_dRdt", accumulated_dRdt);
    // cv::waitKey(0);
    // exit(0);

}

void MaplessDynamic::segmentGround(StrRhoPts* str_in)
{
    int n_row = str_in->img_rho.rows;
    int n_col = str_in->img_rho.cols;
    float* ptr_img_z = str_in->img_z.ptr<float>(0);
    uchar* ptr_groundPtsIdx_next = groundPtsIdx_next_.ptr<uchar>(0);

    for (int i=0; i<n_row; ++i)
    {
        for (int j=0; j<n_col; ++j)
        {
            if (*(ptr_img_z + i * n_col + j) < -1.6)
            {
                *(ptr_groundPtsIdx_next + i * n_col + j) = 255;
            }
        }
    }
}

void MaplessDynamic::checkSegment(cv::Mat& accumulated_dRdt, StrRhoPts* str_next, cv::Mat& groundPtsIdx_next)
{
    float weight_factor = 0.95;
    int n_col = str_next->img_rho.cols;
    int n_row = str_next->img_rho.rows;
    std::vector<int> idx_row;
    std::vector<int> idx_col;
    std::vector<int> check;
    idx_row.reserve(10000);
    idx_col.reserve(10000);
    check.reserve(10000);
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
        for (int j=0; j<n_col; ++j)
        {
            if (*(ptr_accumulated_dRdt + i * n_col + j) != 0)
            {
                // std::cout << accumulated_dRdt.at<float>(i,j) << std::endl;
                // std::cout<<i<<" " <<j<<std::endl;
                idx_row.push_back(i);
                idx_col.push_back(j);
                check.push_back(0);
                isempty = false;
                *(ptr_roi_up+i*n_col+j) = 1;
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

                if ((*(ptr_roi_up + i * n_col + j) == 0) && (*(ptr_img_rho + i * n_col + j) > 0))
                {
                    if (*(ptr_groundPtsIdx_next + i * n_col + j) == 255)
                    {
                        continue;
                    }

                    d1 = std::max(R, *(ptr_img_rho + i * n_col + j));
                    d2 = std::min(R, *(ptr_img_rho + i * n_col + j));
                    if (atan2f(d2 * sinf(phi * D2R), (d1 - d2 * cosf(phi * D2R))) > 10.0 * D2R)
                    {
                        idx_row.push_back(i);
                        idx_col.push_back(j);
                        check.push_back(0);
                        *(ptr_roi_up + i * n_col + j) = 1;
                        *(ptr_accumulated_dRdt + i * n_col + j) = weight_factor * *(ptr_accumulated_dRdt + row * n_col + col);
                    }
                }
                else if ((*(ptr_roi_up + i * n_col + j) == 0) && (*(ptr_img_rho + i * n_col + j) == 0))
                {
                    if (*(ptr_groundPtsIdx_next + i * n_col + j) == 255)
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

                    if (*(ptr_groundPtsIdx_next + ii * n_col + jj) == 255)
                    {
                        continue;
                    }

                    if (*(ptr_roi_up + ii * n_col + jj) == 1){}
                    else
                    {
                        d1 = std::max(R, *(ptr_img_rho + ii * n_col + jj));
                        d2 = std::min(R, *(ptr_img_rho + ii * n_col + jj));

                        if (atan2f(d2 * sinf(2.0 * phi * D2R), (d1 - d2 * cosf(2.0 * phi * D2R))) > 10.0 * D2R)
                        {
                            idx_row.push_back(ii);
                            idx_col.push_back(jj);

                            check.push_back(0);
                            *(ptr_roi_up + ii * n_col + jj) = 1;
                            *(ptr_accumulated_dRdt + ii * n_col + jj) = weight_factor * (*(ptr_accumulated_dRdt + row * n_col + col));
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
                        }
                    }
                }
            }

            if (*(ptr_groundPtsIdx_next + row * n_col + col) == 255)
            {
                *(ptr_roi_up + row * n_col + col) = 0;
            }

            check[cnt] = 1;
        }

        cnt += 1;
        if (cnt == check.size())
        {
            break;
        }
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
    
    for (int i=1; i<n_row; ++i)
    {
        for (int j=1; j<n_col; ++j)
        {
            if (*(ptr_accumulated_dRdt+i*n_col+j)!=0)
            {
                *(ptr_accumulated_dRdt_score+i*n_col+j) += 1;
            }
        }
    }

    for (int i=1; i<n_row; ++i)
    {
        for (int j=1; j<n_col; ++j)
        {
            if (*(ptr_accumulated_dRdt_score+i*n_col+j)>2.0)
            {
                *(ptr_accumulated_dRdt+i*n_col+j) *= 5.0;
                if (*(ptr_accumulated_dRdt+i*n_col+j) > 1e3)
                {
                    *(ptr_accumulated_dRdt+i*n_col+j) = 1e3;
                }
            }
        }
    }
}

void MaplessDynamic::plugImageZeroHoles(cv::Mat& accumulated_dRdt, cv::Mat& accumulated_dRdt_score, StrRhoPts* str_next, cv::Mat& groundPtsIdx_next, int object_threshold)
{
    int n_row = accumulated_dRdt.rows;
    int n_col = accumulated_dRdt.cols;
    float* ptr_accumulated_dRdt = accumulated_dRdt.ptr<float>(0);
    float* ptr_accumulated_dRdt_score = accumulated_dRdt_score.ptr<float>(0);
    cv::Mat dRdt_bin = cv::Mat::zeros(img_height_,img_width_, CV_8UC1);
    cv::Mat dRdt_score_bin = cv::Mat::zeros(img_height_,img_width_, CV_8UC1);
    uchar* ptr_dRdt_bin = dRdt_bin.ptr<uchar>(0);
    uchar* ptr_dRdt_score_bin = dRdt_score_bin.ptr<uchar>(0);

    float* ptr_img_rho = str_next->img_rho.ptr<float>(0);

    for (int i=0; i<n_row; ++i)
    {
        for (int j=0; j<n_col; ++j)
        {
            if (*(ptr_accumulated_dRdt+i*n_col+j)!=0)
            {
                *(ptr_dRdt_bin+i*n_col+j) = 255;
            }
        }
    }

    for (int i=0; i<n_row; ++i)
    {
        for (int j=0; j<n_col; ++j)
        {
            if (*(ptr_accumulated_dRdt_score+i*n_col+j)!=0)
            {
                *(ptr_dRdt_score_bin+i*n_col+j) = 255;
            }
        }
    }    
    // imfill 
    //invert dRdt_bin
    cv::Mat dRdt_bin_inv = cv::Mat::zeros(img_height_, img_width_, CV_8UC1);
    cv::bitwise_not(dRdt_bin, dRdt_bin_inv);
    cv::floodFill(dRdt_bin_inv, cv::Point(0,0), cv::Scalar(0));
    cv::Mat dRdt_bin_filled = (dRdt_bin | dRdt_bin_inv);
    interpAndfill_image(accumulated_dRdt, dRdt_bin_filled);

    cv::Mat dRdt_score_bin_inv = cv::Mat::zeros(img_height_, img_width_, CV_8UC1);
    cv::bitwise_not(dRdt_score_bin, dRdt_score_bin_inv);
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
        for (int j=0; j<n_col; ++j)
        {
            if (*(ptr_rho_zero_value + i * n_col + j) == 0)
            {
                *(ptr_rho_zero_value + i * n_col + j) = 255;
            }
        }
    }

    cv::Mat input_img_mask = cv::Mat::zeros(img_height_, img_width_, CV_8UC1);
    uchar* ptr_input_img_mask = input_img_mask.ptr<uchar>(0);

    for (int i=1; i<n_row; ++i)
    {
        for (int j=0; j<n_col; ++j)
        {
            if (*(ptr_input_img_mask + i * n_col + j) == 0)
            {
                *(ptr_input_img_mask + i * n_col + j) = 255;
            }
        }
    }

    cv::Mat input_img_tmp = accumulated_dRdt.clone();

    cv::Mat connet_input = (rho_zero_value | input_img_mask);
    cv::Mat object_label = cv::Mat::zeros(img_height_, img_width_, CV_32SC1);

    int n_label = cv::connectedComponents(connet_input, object_label, 8);
    int* ptr_object_label = object_label.ptr<int>(0);

    std::vector<int> object_row;
    std::vector<int> object_col;
    std::vector<float> object_rho_roi;
    object_row.reserve(10000);
    object_col.reserve(10000);
    object_rho_roi.reserve(10000);
   
    cv::Mat object_area = cv::Mat::zeros(img_height_,img_width_, CV_8UC1);
    uchar *ptr_object_area = object_area.ptr<uchar>(0);

    cv::Mat object_area_filled = cv::Mat::zeros(img_height_,img_width_, CV_8UC1);
    uchar *ptr_object_area_filled = object_area_filled.ptr<uchar>(0);

    cv::Mat zero_candidate = cv::Mat::zeros(img_height_,img_width_, CV_8UC1);
    uchar *ptr_zero_candidate = zero_candidate.ptr<uchar>(0);

    std::vector<int> filled_object_row;
    std::vector<int> filled_object_col;
    std::vector<float> filled_object_rho_roi;
    filled_object_row.reserve(10000);
    filled_object_col.reserve(10000);
    filled_object_rho_roi.reserve(10000);
    cv::Mat filled_object_rho_mat = cv::Mat::zeros(img_height_, img_width_, CV_32FC1);
    float *ptr_filled_object_rho_mat = filled_object_rho_mat.ptr<float>(0);

    std::vector<float> max_his_filled_object_rho_roi;
    max_his_filled_object_rho_roi.reserve(10000);

    std::vector<int> rho_zero_filled_value_row;
    std::vector<int> rho_zero_filled_value_col;
    std::vector<float> rho_zero_filled_value_rho_roi;
    rho_zero_filled_value_row.reserve(10000);
    rho_zero_filled_value_col.reserve(10000);
    rho_zero_filled_value_rho_roi.reserve(10000);

    std::vector<int> disconti_row;
    std::vector<int> disconti_col;
    disconti_row.reserve(10000);
    disconti_col.reserve(10000);

    cv::MatND histogram;
    for (int object_idx = 0; object_idx < n_label; ++object_idx)
    {
        if (object_idx==0) //background
        {
            continue;
        }

        object_row.clear();
        object_col.clear();
        object_rho_roi.clear();
        filled_object_row.clear();
        filled_object_col.clear();
        filled_object_rho_roi.clear();
        max_his_filled_object_rho_roi.clear();
        rho_zero_filled_value_row.clear();
        rho_zero_filled_value_col.clear();
        rho_zero_filled_value_rho_roi.clear();
        disconti_row.clear();
        disconti_col.clear();

        for (int i=0; i<n_row; ++i)
        {
            for (int j=0; j<n_col; ++j)
            {
                if(*(ptr_object_label+i*n_col+j)==object_idx)
                {
                    object_row.push_back(i);
                    object_col.push_back(j);
                    object_rho_roi.push_back(*(ptr_accumulated_dRdt + i * n_col + j));
                    *(ptr_object_area + i * n_col + j) = 255;
                }
            }
        }

        if (object_row.size() < object_threshold)
        {
            for (int i = 0; i < object_row.size(); ++i)
            {
                *(ptr_rho_zero_value + object_row[i] * n_col + object_col[i]) = 0.0;
                *(ptr_accumulated_dRdt + object_row[i] * n_col + object_col[i]) = 0.0;
                continue;
            }
        }
        else
        {
            float connect_zero_mean = 0.0;
            int n_connet_zero = 0;

            cv::Mat object_area_inv = cv::Mat::zeros(img_height_, img_width_, CV_8UC1);
            cv::bitwise_not(object_area, object_area_inv);
            cv::floodFill(object_area_inv, cv::Point(0, 0), cv::Scalar(0));
            cv::Mat object_area_filled = (object_area | object_area_inv);

            for (int i=0; i<n_row; ++i)
            {
                for (int j=0; j<n_col; ++j)
                {
                    if(*(ptr_object_area_filled+i*n_col+j)!=0 && *(ptr_input_img_mask+i*n_col+j)!=0)
                    {
                        connect_zero_mean += *(ptr_accumulated_dRdt+i*n_col + j);
                        n_connet_zero += 1;
                    }
                }
            }
            if (n_connet_zero > 0)
            {
                connect_zero_mean /= n_connet_zero;

                for (int i = 0; i < n_row; ++i)
                {
                    for (int j = 0; j < n_col; ++j)
                    {
                        if(*(ptr_object_area_filled+i*n_col+j)!=0 && *(ptr_input_img_mask+i*n_col+j)==0)
                        {
                            *(ptr_img_rho + i * n_col + j) = connect_zero_mean;
                        }
                    }
                }
            }
            else
            {
                continue;
            }

            for (int i = 0; i < n_row; ++i)
            {
                for (int j = 0; j < n_col; ++j)
                {
                    if (*(ptr_object_area_filled + i * n_col + j) != 0 && *(ptr_input_img_mask + i * n_col + j) == 0 && *(ptr_img_rho + i * n_col + j) != 0)
                    {
                        filled_object_row.push_back(i);
                        filled_object_col.push_back(j);
                        filled_object_rho_roi.push_back(*(ptr_img_rho + i * n_col + j));
                        *(ptr_filled_object_rho_mat + i * n_col + j) = 255;
                    }
                }
            }

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
                for (int j = 0; j < n_col; ++j)
                {
                    if (*(ptr_object_area_filled + i * n_col + j) != 0 && *(ptr_img_rho + i * n_col + j) != 0)
                    {
                        rho_zero_filled_value_row.push_back(i);
                        rho_zero_filled_value_col.push_back(j);
                        rho_zero_filled_value_rho_roi.push_back(*(ptr_img_rho + i * n_col + j));
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
    row.reserve(10000);
    col.reserve(10000);

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
        for (int j=0; j<n_col; ++j)
        {
            if ((*(ptr_input_img + i * n_col + j) == 0) && (*(ptr_filled_bin + i * n_col + j) > 0))
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

    for (int i_v=0; i_v<row.size(); ++i_v)
    {
        int i = row[i_v];

        for (int j_v=0; j_v<col.size(); ++j_v)
        {
            int j = col[j_v];

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
    }
    input_img_cp.copyTo(input_img);
}

void MaplessDynamic::countZerofloat(cv::Mat& input_mat)
{
    int n_row = input_mat.rows;
    int n_col = input_mat.cols;
    float* ptr_input_mat = input_mat.ptr<float>(0);
    int cnt = 0;
    for (int i=0; i<n_row; ++i)
    {
        for (int j=0; j<n_col; ++j)
        {
            if(*(ptr_input_mat+i*n_col+j)!=0)
            {
                cnt+=1;
            }
        }
    }
    std::cout<<"# of non zero: "<<cnt <<std::endl;
}

void MaplessDynamic::countZeroint(cv::Mat& input_mat)
{
    int n_row = input_mat.rows;
    int n_col = input_mat.cols;
    int* ptr_input_mat = input_mat.ptr<int>(0);
    int cnt = 0;
    for (int i=0; i<n_row; ++i)
    {
        for (int j=0; j<n_col; ++j)
        {
            if(*(ptr_input_mat+i*n_col+j)!=0)
            {
                cnt+=1;
            }
        }
    }
    std::cout<<"# of non zero: "<<cnt <<std::endl;
}

void MaplessDynamic::countZerouchar(cv::Mat& input_mat)
{
    int n_row = input_mat.rows;
    int n_col = input_mat.cols;
    uchar* ptr_input_mat = input_mat.ptr<uchar>(0);
    int cnt = 0;
    for (int i=0; i<n_row; ++i)
    {
        for (int j=0; j<n_col; ++j)
        {
            if(*(ptr_input_mat+i*n_col+j)!=0)
            {
                cnt+=1;
            }
        }
    }
    std::cout<<"# of non zero: "<<cnt <<std::endl;
}