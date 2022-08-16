#include "mapless_dynamic.h"
#include <string>
#include <fstream>

#include <user_param.h>

MaplessDynamic::MaplessDynamic(ros::NodeHandle& nh, bool rosbag_play, std::string& data_number)
: nh_(nh), rosbag_play_(rosbag_play), data_number_(data_number), is_initialized_test_(false)
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

    p0_pcl_test_ = boost::make_shared<pcl::PointCloud<pcl::PointXYZ>>();
    p1_pcl_test_ = boost::make_shared<pcl::PointCloud<pcl::PointXYZ>>();
    

    // Construct Class
    CloudFrame_cur_             = std::make_unique<CloudFrame>(UserParam_);
    CloudFrame_next_            = std::make_unique<CloudFrame>(UserParam_);
    CloudFrame_cur_warped_      = std::make_unique<CloudFrame>(UserParam_);
    CloudFrame_warpPointcloud_  = std::make_unique<CloudFrame>(UserParam_);
    SegmentGround_              = std::make_unique<SegmentGround>(UserParam_);
    dRCalc_                     = std::make_unique<dRCalc>(UserParam_);
    PclWarp_                    = std::make_unique<PclWarp>(UserParam_);
    ObjectExt_                  = std::make_unique<ObjectExt>(UserParam_);
    ImageFill_                  = std::make_unique<ImageFill>(UserParam_);

    // allocation for solver
    img_height_ = UserParam_->image_param_.height_;
    img_width_  = UserParam_->image_param_.width_;
    accumulated_dRdt_       = cv::Mat::zeros(img_height_, img_width_, CV_32FC1);
    accumulated_dRdt_score_ = cv::Mat::zeros(img_height_, img_width_, CV_32FC1);
    dRdt_                   = cv::Mat::zeros(img_height_, img_width_, CV_32FC1);

    this->loadTestData();
    // END YOUR CODE
};


MaplessDynamic::~MaplessDynamic() {
    // destructor
    ROS_INFO_STREAM("MaplessDynamic - deleted.");
    // IMPLEMENT YOUR CODE FROM THIS LINE.

    // END YOUR CODE
};

void MaplessDynamic::RosbagData(){
    static int cnt_data = 0;
    std::cout<< "Iter: "<< cnt_data << std::endl;

    // p0 = get!
    // p1 = get!
    // T01 = get!

    // this->algorithm(p0,p1,T01);
    
    // END YOUR CODE

    if( is_initialized_test_ ){ // If initialized, 
        // 0. Get the current LiDAR data
        p1_msg_test_ = *(data_buf_[cnt_data]->pcl_msg_);
        pcl::fromROSMsg(p1_msg_test_, *p1_pcl_test_);

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
        pcl::fromROSMsg(p0_msg_test_, *p0_pcl_test_);
        
        CloudFrame_cur_ ->genRangeImages(p0_pcl_test_, true);

        mask0_test_.resize(p0_msg_test_.width, true);
    }
    cnt_data += 1;
    // std::cout << cnt_data << std::endl;
}

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
        pcl::fromROSMsg(p1_msg_test_, *p1_pcl_test_);

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
        pcl::fromROSMsg(p0_msg_test_, *p0_pcl_test_);
        
        CloudFrame_cur_ ->genRangeImages(p0_pcl_test_, true);

        mask0_test_.resize(p0_msg_test_.width, true);
    }
    cnt_data += 1;
    // std::cout << cnt_data << std::endl;
};





void MaplessDynamic::loadTestData(){
    std::string data_num = data_number_; //"07";
    std::cout <<"data_number: " << data_num <<std::endl;
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
        start_num = 150 + 00; //94;
        final_num = 250 + 1; // 1 ~ 101+2
    }
    else if (data_num == "02"){
        start_num = 860 + 00;
        final_num = 950 + 1; // 1 ~ 91+2
    }
    else if (data_num == "05"){
        start_num = 2350 + 00; //144+41;
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
    pcl::PointCloud<pcl::PointXYZ>::Ptr p0, pcl::PointCloud<pcl::PointXYZ>::Ptr p1, const Pose& T01, 
    /* outputs */ 
    Mask& mask1, int cnt_data)
{
    // IMPLEMENT YOUR ALGORITHM FROM THIS LINE.

    // do something...

    // END YOUR ALGORITHM

    // if (cnt_data == 1)
    // {
    //     std::cout <<"p0 size: " << p0->size()<<std::endl;
    //     sensor_msgs::PointCloud2 converted_msg_d;
    //     pcl::toROSMsg(*p0, converted_msg_d);
    //     converted_msg_d.header.frame_id = "map";
    //     pub_dynamic_pts_.publish(converted_msg_d);

    //     std::cout <<"p1 size: " << p1->size()<<std::endl;
    //     sensor_msgs::PointCloud2 converted_msg_s;
    //     pcl::toROSMsg(*p1, converted_msg_s);
    //     converted_msg_s.header.frame_id = "map";
    //     pub_static_pts_.publish(converted_msg_s);
    //     exit(0);
    // }
    // ROS_INFO_STREAM("# of p0' :" << p0->size() << " " << "# of p1' :" << p1->size() << " ");

    // timer::tic();
    // double dt_slam = timer::toc(); // milliseconds
    // ROS_INFO_STREAM("elapsed time for 'compensateCurRhoZeroWarp' :" << dt_slam << " [ms]");

    // pointcloud input, p1
    // p0 is already processed in initial step
    timer::tic();
    CloudFrame_next_->genRangeImages(p1, true);
    double dt_toc1 = timer::toc(); // milliseconds
    ROS_INFO_STREAM("elapsed time for 'genRangeImages' :" << dt_toc1 << " [ms]");

    // Warp pcl represented in current frame to next frame
    T_next2cur_ = T01;
    
    // Segment ground
    timer::tic();
    SegmentGround_->fastsegmentGround(CloudFrame_next_);
    double dt_toc2 = timer::toc(); // milliseconds
    ROS_INFO_STREAM("elapsed time for 'segmentSGround' :" << dt_toc2 << " [ms]");
    //// Occlusion accumulation ////
    // Compute the occlusion dRdt

        // cv::imshow("SegmentGround_", SegmentGround_->groundPtsIdx_next_);

    
    timer::tic();
    dRCalc_->dR_warpPointcloud(CloudFrame_next_, CloudFrame_cur_, CloudFrame_cur_warped_, p0, T_next2cur_, cnt_data, dRdt_);
    double dt_toc3 = timer::toc(); // milliseconds
    ROS_INFO_STREAM("elapsed time for 'dR_warpPointcloud' :" << dt_toc3 << " [ms]");
    // str_next_->state();
    //     cv::imshow("residual_", residual_);
    // cv::waitKey(0);
    // exit(0);

    timer::tic();
    // warp the occlusion accumulation map
    PclWarp_->warpPointcloud(CloudFrame_cur_, CloudFrame_warpPointcloud_, T_next2cur_, accumulated_dRdt_, cnt_data);
    PclWarp_->initializeStructAndPcl(CloudFrame_warpPointcloud_);
    PclWarp_->warpPointcloud(CloudFrame_cur_, CloudFrame_warpPointcloud_, T_next2cur_, accumulated_dRdt_score_, cnt_data);
    double dt_toc4 = timer::toc(); // milliseconds
    ROS_INFO_STREAM("elapsed time for 'warpPointcloud' :" << dt_toc4 << " [ms]");

    // cv::imshow("after warpPointcloud", accumulated_dRdt_);
    
    //     if (cnt_data == 2)
    // {
    //     cv::imshow("before d", accumulated_dRdt_);
    //     countZerofloat(accumulated_dRdt_);
    //     cv::imshow("before  k", accumulated_dRdt_score_);
    //     countZerofloat(accumulated_dRdt_score_);
    // }
    timer::tic();
    // filter out outliers
    ObjectExt_->filterOutAccumdR(CloudFrame_next_, CloudFrame_cur_warped_, accumulated_dRdt_, accumulated_dRdt_score_, dRdt_);
    double dt_toc5 = timer::toc(); // milliseconds
    ROS_INFO_STREAM("elapsed time for 'filterOutAccumdR' :" << dt_toc5 << " [ms]");

    // cv::imshow("after filterOutAccumdR", accumulated_dRdt_);

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
    ObjectExt_->extractObjectCandidate(accumulated_dRdt_, CloudFrame_next_);
    double dt_toc6 = timer::toc(); // milliseconds
    ROS_INFO_STREAM("elapsed time for 'extractObjectCandidate' :" <<  dt_toc6 << " [ms]");

    // cv::imshow("after extractObjectCandidate", accumulated_dRdt_);
    //// update object_mask
    //object_mask = accumulated_dRdt>0;
    timer::tic();
    // Fast Segment
    ObjectExt_->checkSegment(accumulated_dRdt_, CloudFrame_next_, SegmentGround_->groundPtsIdx_next_);
    double dt_toc7 = timer::toc(); // milliseconds
    ROS_INFO_STREAM("elapsed time for 'checkSegment' :" <<  dt_toc7 << " [ms]");
    // cv::imshow("after checkSegment", accumulated_dRdt_);
    //// update object_mask
    //object_mask = accumulated_dRdt>0;
    timer::tic();
    ObjectExt_->updateAccum(accumulated_dRdt_, accumulated_dRdt_score_);
    double dt_toc8 = timer::toc(); // milliseconds
    ROS_INFO_STREAM("elapsed time for 'updateAccum' :" <<  dt_toc8 << " [ms]");

    // cv::imshow("after updateAccum", accumulated_dRdt_);

    timer::tic();
    ImageFill_->plugImageZeroHoles(accumulated_dRdt_, accumulated_dRdt_score_, CloudFrame_next_);
    double dt_toc9 = timer::toc(); // milliseconds
    ROS_INFO_STREAM("elapsed time for 'plugImageZeroHoles' :" <<  dt_toc9 << " [ms]");

    // cv::imshow("after plugImageZeroHoles", accumulated_dRdt_);
    
    timer::tic();
    ObjectExt_->updateAccumdRdt(CloudFrame_next_, accumulated_dRdt_, accumulated_dRdt_score_, dRdt_, SegmentGround_->groundPtsIdx_next_);
    double dt_toc10 = timer::toc(); // milliseconds
    ROS_INFO_STREAM("elapsed time for 'updateAccumdRdt' :" <<  dt_toc10 << " [ms]");

    float* ptr_accumulated_dRdt = accumulated_dRdt_.ptr<float>(0);
    float* ptr_accumulated_dRdt_score = accumulated_dRdt_score_.ptr<float>(0);

    // cv::imshow("accumulated_dRdt", accumulated_dRdt_);
    // cv::waitKey(0);
    // exit(0);
    timer::tic;
    pcl::PointCloud<pcl::PointXYZ> pcl_dynamic;
    pcl::PointCloud<pcl::PointXYZ> pcl_static;

    // float* ptr_next_img_x = CloudFrame_next_->str_rhopts_->img_x.ptr<float>(0);
    // float* ptr_next_img_y = CloudFrame_next_->str_rhopts_->img_y.ptr<float>(0);
    // float* ptr_next_img_z = CloudFrame_next_->str_rhopts_->img_z.ptr<float>(0);

    // for (int i = 0; i < img_height_; ++i)
    // {
    //     int i_ncols = i * img_width_;
    //     for (int j = 0; j < img_width_; ++j)
    //     {
    //         if (*(ptr_accumulated_dRdt + i_ncols + j)!=0)
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
            if (*(ptr_accumulated_dRdt + i_ncols + j)!=0) // dynamic
            {
                if (CloudFrame_next_->str_rhopts_->pts_per_pixel_index_valid[i_ncols + j].size() != 0)
                {
                    for (int k = 0; k < CloudFrame_next_->str_rhopts_->pts_per_pixel_index_valid[i_ncols + j].size(); ++k)
                    {
                        pcl_dynamic.push_back(
                            pcl::PointXYZ((*p1)[CloudFrame_next_->str_rhopts_->pts_per_pixel_index_valid[i_ncols + j][k]]));
                    }
                }
            }
            else // static
            {
                if (CloudFrame_next_->str_rhopts_->pts_per_pixel_index_valid[i_ncols + j].size() != 0)
                {
                    for (int k = 0; k < CloudFrame_next_->str_rhopts_->pts_per_pixel_index_valid[i_ncols + j].size(); ++k)
                    {
                        pcl_static.push_back(
                            pcl::PointXYZ((*p1)[CloudFrame_next_->str_rhopts_->pts_per_pixel_index_valid[i_ncols + j][k]]));
                    }
                }
            }
        }
    }
    double dt_toc11 = timer::toc(); // milliseconds
    ROS_INFO_STREAM("elapsed time for 'segmentWholePts' :" <<  dt_toc11 << " [ms]");

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
    double dt_toc12 = timer::toc(); // milliseconds
    ROS_INFO_STREAM("elapsed time for 'publish' :" <<  dt_toc12 << " [ms]");

    timer::tic();
    //// update for next iteration
    for (int i = 0; i < img_height_; ++i)
    {
        int i_ncols = i * img_width_;
        for (int j = 0; j < img_width_; ++j)
        {
            if (*(ptr_accumulated_dRdt + i_ncols + j)==0)
            {
                *(ptr_accumulated_dRdt_score + i_ncols + j) = 0;
            }
        }
    }

    copyStructAndinitialize(p1, p0, cnt_data);
    double dt_toc13 = timer::toc(); // milliseconds
    ROS_INFO_STREAM("elapsed time for 'copyRemove' :" <<  dt_toc13 << " [ms]");
    
    double dt_toc_total = dt_toc1 + dt_toc2 + dt_toc3 + dt_toc4 + dt_toc5 + dt_toc6 + dt_toc7 + dt_toc8 + dt_toc9 + dt_toc10 + dt_toc11 + dt_toc12 + dt_toc13;
    ROS_INFO_STREAM("elapsed time for 'total' :" <<  dt_toc_total << " [ms]");
    ROS_INFO_STREAM("=======================================================");

    // cv::imshow("final", accumulated_dRdt_);
    // cv::waitKey(0);
};

void MaplessDynamic::copyStructAndinitialize(pcl::PointCloud<pcl::PointXYZ>::Ptr p1 ,pcl::PointCloud<pcl::PointXYZ>::Ptr p0, int cnt_data)
{
    // copy Cur to Next
    {   CloudFrame_cur_->str_rhopts_->pts        = CloudFrame_next_->str_rhopts_->pts;

        CloudFrame_cur_->str_rhopts_->rho.resize(0);
        CloudFrame_cur_->str_rhopts_->phi.resize(0);
        CloudFrame_cur_->str_rhopts_->theta.resize(0);
        std::copy(CloudFrame_next_->str_rhopts_->rho.begin(),CloudFrame_next_->str_rhopts_->rho.end(), CloudFrame_cur_->str_rhopts_->rho.begin());
        std::copy(CloudFrame_next_->str_rhopts_->phi.begin(),CloudFrame_next_->str_rhopts_->phi.end(), CloudFrame_cur_->str_rhopts_->phi.begin());
        std::copy(CloudFrame_next_->str_rhopts_->theta.begin(),CloudFrame_next_->str_rhopts_->theta.end(), CloudFrame_cur_->str_rhopts_->theta.begin());

        CloudFrame_next_->str_rhopts_->img_rho.copyTo(CloudFrame_cur_->str_rhopts_->img_rho);
        CloudFrame_next_->str_rhopts_->img_index.copyTo(CloudFrame_cur_->str_rhopts_->img_index);
        CloudFrame_next_->str_rhopts_->img_x.copyTo(CloudFrame_cur_->str_rhopts_->img_x);
        CloudFrame_next_->str_rhopts_->img_y.copyTo(CloudFrame_cur_->str_rhopts_->img_y);
        CloudFrame_next_->str_rhopts_->img_z.copyTo(CloudFrame_cur_->str_rhopts_->img_z);

        // CloudFrame_cur_->str_rhopts_->pts_per_pixel_n.resize(0);
        // std::copy(CloudFrame_next_->str_rhopts_->pts_per_pixel_n.begin(), CloudFrame_next_->str_rhopts_->pts_per_pixel_n.end(), CloudFrame_cur_->str_rhopts_->pts_per_pixel_n.begin());

        for (int i=0; i<img_height_*img_width_; ++i)
        {
            if (CloudFrame_cur_->str_rhopts_->pts_per_pixel_index[i].size() != 0)
            {
                CloudFrame_cur_->str_rhopts_->pts_per_pixel_index[i].resize(0);
                std::copy(CloudFrame_next_->str_rhopts_->pts_per_pixel_index[i].begin(), CloudFrame_next_->str_rhopts_->pts_per_pixel_index[i].end(), CloudFrame_cur_->str_rhopts_->pts_per_pixel_index[i].begin());
            }

            if (CloudFrame_cur_->str_rhopts_->pts_per_pixel_rho[i].size() != 0)
            {
                CloudFrame_cur_->str_rhopts_->pts_per_pixel_rho[i].resize(0);
                std::copy(CloudFrame_next_->str_rhopts_->pts_per_pixel_rho[i].begin(), CloudFrame_next_->str_rhopts_->pts_per_pixel_rho[i].end(), CloudFrame_cur_->str_rhopts_->pts_per_pixel_rho[i].begin());
            }

            if (CloudFrame_cur_->str_rhopts_->pts_per_pixel_index_valid[i].size() != 0)
            {
                CloudFrame_cur_->str_rhopts_->pts_per_pixel_index_valid[i].resize(0);
                std::copy(CloudFrame_next_->str_rhopts_->pts_per_pixel_index_valid[i].begin(), CloudFrame_next_->str_rhopts_->pts_per_pixel_index_valid[i].end(), CloudFrame_cur_->str_rhopts_->pts_per_pixel_index_valid[i].begin());
            }
        }

        CloudFrame_next_->str_rhopts_->img_restore_mask.copyTo(CloudFrame_cur_->str_rhopts_->img_restore_mask);
        CloudFrame_next_->str_rhopts_->img_restore_warp_mask.copyTo(CloudFrame_cur_->str_rhopts_->img_restore_warp_mask);
    }

    // only in test
    if (rosbag_play_ == false)
    {
        p0_pcl_test_->resize(0);
        pcl::copyPointCloud(*p1, *p0_pcl_test_);
        p1->resize(0);
    }

    // Next -> reset();
    CloudFrame_next_->reset();

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



    // if (cnt_data == 2)
    // {
    //     std::cout << p0_pcl_test_.size() << std::endl;
    //     sensor_msgs::PointCloud2 converted_msg_s;
    //     pcl::toROSMsg(p0_pcl_test_, converted_msg_s);
    //     converted_msg_s.header.frame_id = "map";
    //     pub_static_pts_.publish(converted_msg_s);
        
    // }

    // str_cur_warped_ -> reset();
    CloudFrame_cur_warped_->reset();

    // str_warpPointcloud_ -> reset();
    CloudFrame_warpPointcloud_->reset();
    
    // ground index -> reset();
    SegmentGround_->reset();
    
    dRCalc_->velo_cur_->resize(0);
    dRCalc_->cur_pts_warped_->resize(0);
    
    dRdt_ = cv::Mat::zeros(img_height_, img_width_, CV_32FC1);

    PclWarp_->reset();

    // std::cout << cur_pts_warped_->width << std::endl;
    // std::cout << pts_warpewd_->width << std::endl;
    // exit(0);
}

void MaplessDynamic::getUserSettingParameters(){
    // IMPLEMENT YOUR CODE FROM THIS LINE.

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