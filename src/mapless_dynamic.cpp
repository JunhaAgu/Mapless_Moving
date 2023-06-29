#include "mapless_dynamic.h"

#include <user_param.h>

#include <fstream>
#include <string>

MaplessDynamic::MaplessDynamic(ros::NodeHandle& nh, bool rosbag_play,
                               std::string& dataset_kitti_dir,
                               std::string& dataset_carla_dir,
                               std::string& dataset_name,
                               std::string& data_number)
    : nh_(nh),
      rosbag_play_(rosbag_play),
      dataset_kitti_dir_(dataset_kitti_dir),
      dataset_carla_dir_(dataset_carla_dir),
      dataset_name_(dataset_name),
      data_number_(data_number),
      is_initialized_test_(false) {
    // constructor
    ROS_INFO_STREAM("MaplessDynamic - constructed.");

    pub_dynamic_pts_ =
        nh_.advertise<sensor_msgs::PointCloud2>("/dynamic_pts", 1);
    pub_static_pts_ = nh_.advertise<sensor_msgs::PointCloud2>(
        "/static_pts", 1);  // /static_pts
    // pub_static_pts_  = nh_.advertise<PointCloudwithTime>("/static_pts",1); //
    // ct_icp

    // Class UserParam
    std::unique_ptr<UserParam> UserParam_;
    UserParam_ = std::make_unique<UserParam>();
    UserParam_->getUserSettingParameters(dataset_name_);

    // this->getUserSettingParameters();

    p0_pcl_test_ = boost::make_shared<PointCloudwithTime>();
    p1_pcl_test_ = boost::make_shared<PointCloudwithTime>();

    // Construct Class
    CloudFrame_cur_ = std::make_unique<CloudFrame>(UserParam_);
    CloudFrame_next_ = std::make_unique<CloudFrame>(UserParam_);
    CloudFrame_cur_warped_ = std::make_unique<CloudFrame>(UserParam_);
    CloudFrame_warpPointcloud_ = std::make_unique<CloudFrame>(UserParam_);
    SegmentGround_ = std::make_unique<SegmentGround>(UserParam_);
    dRCalc_ = std::make_unique<dRCalc>(UserParam_);
    PclWarp_ = std::make_unique<PclWarp>(UserParam_);
    ObjectExt_ = std::make_unique<ObjectExt>(UserParam_);
    ImageFill_ = std::make_unique<ImageFill>(UserParam_);

    // allocation for solver
    img_height_ = UserParam_->image_param_.height_;
    img_width_ = UserParam_->image_param_.width_;
    accumulated_dRdt_ = cv::Mat::zeros(img_height_, img_width_, CV_32FC1);
    accumulated_dRdt_score_ = cv::Mat::zeros(img_height_, img_width_, CV_32FC1);
    dRdt_ = cv::Mat::zeros(img_height_, img_width_, CV_32FC1);

    this->loadTestData();
    // END YOUR CODE
};

MaplessDynamic::~MaplessDynamic() {
    // destructor
    ROS_INFO_STREAM("MaplessDynamic - deleted.");
    // IMPLEMENT YOUR CODE FROM THIS LINE.

    // END YOUR CODE
};

bool closeEnough(const float& a, const float& b,
                 const float& epsilon = std::numeric_limits<float>::epsilon()) {
    return (epsilon > std::abs(a - b));
}

void eulerAngles(Rot& R, Euler& e) {
    // check for gimbal lock
    if (closeEnough(R(0, 2), -1.0f)) {
        float x = 0;  // gimbal lock, value of x doesn't matter
        float y = M_PI / 2;
        float z = x + atan2(R(1, 0), R(2, 0));
        e << x, y, z;
    } else if (closeEnough(R(0, 2), 1.0f)) {
        float x = 0;
        float y = -M_PI / 2;
        float z = -x + atan2(-R(1, 0), -R(2, 0));
        e << x, y, z;
    } else {  // two solutions exist
        float x1 = -asin(R(0, 2));
        float x2 = M_PI - x1;

        float y1 = atan2(R(1, 2) / cos(x1), R(2, 2) / cos(x1));
        float y2 = atan2(R(1, 2) / cos(x2), R(2, 2) / cos(x2));

        float z1 = atan2(R(0, 1) / cos(x1), R(0, 0) / cos(x1));
        float z2 = atan2(R(0, 1) / cos(x2), R(0, 0) / cos(x2));

        // choose one solution to return
        // for example the "shortest" rotation
        if ((std::abs(x1) + std::abs(y1) + std::abs(z1)) <=
            (std::abs(x2) + std::abs(y2) + std::abs(z2))) {
            e << x1, y1, z1;
        } else {
            e << x1, y1, z1;
        }
    }
}

// with pcd datasets
void MaplessDynamic::TEST() {
    static int cnt_data = 0;
    static double total_time_gen = 0.0;

    ROS_INFO_STREAM("================= Test iter: " << cnt_data
                                                    << " =================");

    if (is_initialized_test_) {  // If initialized,
        cloudHeader_test_ = data_buf_[cnt_data]->pcl_msg_->header;

        // 0. Get the next LiDAR data
        p1_msg_test_ = *(data_buf_[cnt_data]->pcl_msg_);
        pcl::fromROSMsg(p1_msg_test_, *p1_pcl_test_);

        // 1. Calculate T01 from the SLAM (or Odometry) algorithm
        Pose T10;
        T10 =
            data_buf_[cnt_data]
                ->T_gt_.inverse();  // KITTI: cnt_data / CARLA: cnt_data-1 ,,,,
        std::cout << data_buf_[cnt_data]->T_gt_ << std::endl;
        ROS_INFO_STREAM("size of p0_pcl_test_: " << p0_pcl_test_->size());
        ROS_INFO_STREAM("size of p1_pcl_test_: " << p1_pcl_test_->size());

        // 2. Solve the Mapless Dynamic algorithm.
        timer::tic();
        Mask mask1;

        CloudFrame_next_->genRangeImages(p1_pcl_test_, true);

        double dt_gen = timer::toc();  // milliseconds
        total_time_gen += dt_gen;
        ROS_INFO_STREAM("Average time for 'genRangeImages' :"
                        << total_time_gen / cnt_data << " [ms]"
                        << " "
                        << "window :" << cnt_data);

        this->solve(p0_pcl_test_, p1_pcl_test_, T10, mask1, cnt_data,
                    cloudHeader_test_);

        // 3. Update the previous variables
        if (rosbag_play_ == false) {
            p0_pcl_test_->resize(0);
            pcl::copyPointCloud(*p1_pcl_test_, *p0_pcl_test_);
        }

        // End condition
        if (cnt_data == n_valid_data_ - 1)  // -1 -1
        {
            ROS_INFO_STREAM("All data is processed");
            exit(0);
        }
    } else {  // If not initialized,
        is_initialized_test_ = true;

        // Initialize the first data.
        p0_msg_test_ = *(data_buf_[0]->pcl_msg_);
        pcl::fromROSMsg(p0_msg_test_, *p0_pcl_test_);

        CloudFrame_cur_->genRangeImages(p0_pcl_test_, true);
        ROS_INFO_STREAM("size of p0_pcl_test_: " << p0_pcl_test_->size());
    }
    cnt_data += 1;
    // std::cout << cnt_data << std::endl;
};

void MaplessDynamic::loadTestData() {
    std::string dataset_name = dataset_name_;  // KITTI or CARLA
    std::string data_num = data_number_;       //"07";
    std::cout << "dataset_name: " << dataset_name << ", "
              << "data_number: " << data_num << std::endl;
    std::string dataset_dir;
    if (dataset_name == "KITTI") {
        dataset_dir = dataset_kitti_dir_;
        //"/mnt/g/reinstall_ubuntu/KITTI_odometry/";
    } else if (dataset_name == "CARLA") {
        dataset_dir = dataset_carla_dir_;
        //"/mnt/g/mapless_dataset/CARLA/";
    }

    float pose_arr[12];
    Pose T_tmp;
    Pose T_warp;
    T_warp << 0, -1, 0, 0, 0, 0, -1, 0, 1, 0, 0, 0, 0, 0, 0, 1;

    int start_num;
    int final_num;
    if (dataset_name == "KITTI") {
        if (data_num == "00") {
            start_num = 4390 + 00;
            final_num = 4530 + 1;  // 1 ~ 141+2 75
        } else if (data_num == "01") {
            start_num = 150 + 00;  // 94;
            final_num = 250 + 1;   // 1 ~ 101+2
        } else if (data_num == "02") {
            start_num = 860 + 00;
            final_num = 950 + 1;  // 1 ~ 91+2
        } else if (data_num == "05") {
            start_num = 2350 + 00;  // 144+41;
            final_num = 2670 + 1;   // 1 ~ 321+2
        } else if (data_num == "07") {
            start_num = 630 + 00;
            final_num = 820 + 1;  // 1 ~ 190+2
        }
        // start_num = start_num - 1;
        // final_num = final_num - 1;
    } else if (dataset_name == "CARLA") {
        if (data_num == "01") {
            start_num = 10 + 00;
            final_num = 370 + 1;  // 1 ~ 141+2 75
        } else if (data_num == "03") {
            start_num = 10 + 00;  // 94;
            final_num = 400 + 1;  // 1 ~ 101+2
        }
    }

    // read Association --> n_data_
    std::string pose_dir;
    if (dataset_name == "KITTI") {
        pose_dir = dataset_dir + "data_odometry_poses/dataset/poses/" +
                   data_num + ".txt";
    } else if (dataset_name == "CARLA") {
        pose_dir = dataset_dir + "sequences/" + data_num + "/" + "poses.txt";
    }
    std::ifstream posefile;
    posefile.open(pose_dir.c_str());
    std::string s;
    int cnt_line = 0;

    std::cout << "Test data directory: " << pose_dir << std::endl;
    while (getline(posefile, s, '\n')) {
        ++cnt_line;
    }
    posefile.close();
    n_data_ = cnt_line;
    // std::cout << cnt_line << std::endl;

    // allocation
    data_buf_.reserve(n_data_);  // resize error...
    all_pose_.reserve(n_data_);  // resize error...
    all_T_gt_.reserve(n_data_);
    for (int i = 0; i < n_data_; ++i) {
        all_pose_[i].reserve(12);
        data_buf_.push_back(new TestData);

        data_buf_[i]->pcl_ = boost::make_shared<PointCloudwithTime>();
        data_buf_[i]->pcl_msg_ = (new sensor_msgs::PointCloud2);
    }

    // read pose file
    posefile.open(pose_dir.c_str());
    cnt_line = 0;
    while (!posefile.eof()) {
        getline(posefile, s);
        if (!s.empty()) {
            std::stringstream ss;
            ss << s;
            for (int i = 0; i < 12; ++i) {
                ss >> pose_arr[i];
                all_pose_[cnt_line].push_back(pose_arr[i]);
                // std::cout << pose_arr[i] << std::endl;
            }
            ++cnt_line;
        }
    }

    n_valid_data_ = final_num - start_num + 1;

    for (int i = 0; i < n_valid_data_; ++i) {
        valid_data_.push_back(start_num + i);
    }

    for (int ii = 0; ii < n_valid_data_; ++ii) {
        int i = valid_data_[ii];
        T_tmp = Pose::Zero(4, 4);
        T_tmp(3, 3) = 1;
        T_tmp.block<1, 4>(0, 0) << all_pose_[i][0], all_pose_[i][1],
            all_pose_[i][2], all_pose_[i][3];
        T_tmp.block<1, 4>(1, 0) << all_pose_[i][4], all_pose_[i][5],
            all_pose_[i][6], all_pose_[i][7];
        T_tmp.block<1, 4>(2, 0) << all_pose_[i][8], all_pose_[i][9],
            all_pose_[i][10], all_pose_[i][11];

        if (dataset_name_ == "KITTI") {
            all_T_gt_[i] = T_tmp * T_warp;
        } else if (dataset_name_ == "CARLA") {
            all_T_gt_[i] = T_tmp;
        }
    }

    for (int ii = 0; ii < n_valid_data_; ++ii) {
        int i = valid_data_[ii];
        // if (dataset_name_ == "KITTI")
        // {
        if (ii == 0) {
            data_buf_[ii]->T_gt_ = all_T_gt_[i].inverse() * all_T_gt_[i];
        } else {
            data_buf_[ii]->T_gt_ = all_T_gt_[i - 1].inverse() * all_T_gt_[i];
        }

        // std::cout << data_buf_[ii]->T_gt_ << std::endl;
    }

    // LiDAR data (bin) read
    std::string bin_path;
    if (dataset_name == "KITTI") {
        bin_path = dataset_dir + "data_odometry_velodyne/dataset/sequences/" +
                   data_num + "/velodyne/";
    } else if (dataset_name == "CARLA") {
        bin_path = dataset_dir + "sequences/" + data_num + "/velodyne/";
    }
    read_filelists(bin_path, file_lists_, "bin");
    sort_filelists(file_lists_, "bin");

#pragma omp parallel num_threads(8)
#pragma omp parallel for
    for (int i = valid_data_[0]; i < valid_data_[valid_data_.size() - 1] + 1;
         ++i) {
        std::string bin_file = bin_path + file_lists_[i];
        // std::cout << i << std::endl;
        readKittiPclBinData(bin_file, i);
    }

    int v_factor = 1;
    // vertical angle resolusion for KITTI or CARLA
    if (dataset_name_ == "KITTI") {
        float inter_top = (2.5 - (-8.0)) / (32 / v_factor - 1);
        for (int i = 0; i < 32 / v_factor; ++i) {
            if (i == 31) {
                v_angle_.push_back(-8.0);
            } else {
                v_angle_.push_back(2.5 - inter_top * i);
            }
        }
        float inter_bottom = (-8.50 - (-23.8)) / (32 / v_factor - 1);
        for (int i = 0; i < 32 / v_factor; ++i) {
            if (i == 31) {
                v_angle_.push_back(-23.8);
            } else {
                v_angle_.push_back(-8.50 - inter_bottom * i);
            }
        }
    } else if (dataset_name_ == "CARLA") {
        float inter = (2.0 - (-24.8)) / (64);
        for (int i = 0; i < 64 / v_factor; ++i) {
            if (i == 63) {
                v_angle_.push_back(-24.8);
            } else {
                v_angle_.push_back(2.0 - inter * i);
            }
        }
    }
    // for (int i=0; i<v_angle_.size(); ++i)
    // std::cout << v_angle_[i] <<std::endl;
    // exit(0);
}

void MaplessDynamic::solve(PointCloudwithTime::Ptr p0,
                           PointCloudwithTime::Ptr p1, const Pose& T10,
                           Mask& mask1, int cnt_data,
                           std_msgs::Header& cloudHeader) {
    ROS_INFO_STREAM("================= Start of the solver =================");

    static int pub_num = 0;

    static double avg_time_C = 0.0;
    static double sum_time_C = 0.0;
    static int iter_time_C = 1;

    static double avg_time_D = 0.0;
    static double sum_time_D = 0.0;
    static int iter_time_D = 1;

    static double avg_time_E = 0.0;
    static double sum_time_E = 0.0;
    static int iter_time_E = 1;

    static double avg_time_F = 0.0;
    static double sum_time_F = 0.0;
    static int iter_time_F = 1;

    static double avg_time_G = 0.0;
    static double sum_time_G = 0.0;
    static int iter_time_G = 1;

    // Warp pcl represented in current frame to next frame
    float object_factor = 1.0;
    T_next2cur_ = T10;
    Euler euler;
    Rot rot = T10.block(0, 0, 3, 3);
    eulerAngles(rot, euler);  // pitch roll yaw
    euler = euler * R2D;
    ROS_INFO_STREAM(
        "Norm of euler angle: " << NORM(euler(0), euler(1), euler(2)));

    if (NORM(euler(0), euler(1), euler(2)) > 3.0) {
        object_factor = 1.5;
    }

    // Segment ground
    timer::tic();
    SegmentGround_->fastsegmentGround(CloudFrame_next_);
    double dt_toc2 = timer::toc();  // milliseconds
    // ROS_INFO_STREAM("elapsed time for 'segmentSGround' :" << dt_toc2 << "
    // [ms]");
    sum_time_C += dt_toc2;
    avg_time_C = sum_time_C / iter_time_C;
    ROS_INFO_STREAM("Average time for 'C' :" << avg_time_C << " [ms]");
    iter_time_C++;

    //// Occlusion accumulation ////
    // Compute the occlusion dRdt

    // cv::imshow("SegmentGround_", SegmentGround_->groundPtsIdx_next_);

    timer::tic();
    dRCalc_->dR_warpPointcloud(CloudFrame_next_, CloudFrame_cur_,
                               CloudFrame_cur_warped_, p0, T_next2cur_,
                               cnt_data, dRdt_);
    // double dt_toc3 = timer::toc(); // milliseconds
    // ROS_INFO_STREAM("elapsed time for 'dR_warpPointcloud' :" << dt_toc3 << "
    // [ms]"); str_next_->state();
    // cv::imshow("dRdt_", dRdt_);
    // cv::waitKey(0);
    // exit(0);

    // timer::tic();
    // warp the occlusion accumulation map
    PclWarp_->warpPointcloud(CloudFrame_cur_, CloudFrame_warpPointcloud_,
                             T_next2cur_, accumulated_dRdt_, cnt_data);
    PclWarp_->initializeStructAndPcl(CloudFrame_warpPointcloud_);
    PclWarp_->warpPointcloud(CloudFrame_cur_, CloudFrame_warpPointcloud_,
                             T_next2cur_, accumulated_dRdt_score_, cnt_data);
    // double dt_toc4 = timer::toc(); // milliseconds
    // ROS_INFO_STREAM("elapsed time for 'warpPointcloud' :" << dt_toc4 << "
    // [ms]");

    // cv::imshow("after warpPointcloud", accumulated_dRdt_);

    //     if (cnt_data == 2)
    // {
    //     cv::imshow("before d", accumulated_dRdt_);
    //     countZerofloat(accumulated_dRdt_);
    //     cv::imshow("before  k", accumulated_dRdt_score_);
    //     countZerofloat(accumulated_dRdt_score_);
    // }
    // timer::tic();
    // filter out outliers
    ObjectExt_->filterOutAccumdR(CloudFrame_next_, CloudFrame_cur_warped_,
                                 accumulated_dRdt_, accumulated_dRdt_score_,
                                 dRdt_);
    // double dt_toc5 = timer::toc(); // milliseconds
    // ROS_INFO_STREAM("elapsed time for 'filterOutAccumdR' :" << dt_toc5 << "
    // [ms]");

    double dt_toc3 = timer::toc();  // milliseconds
    sum_time_D += dt_toc3;
    avg_time_D = sum_time_D / iter_time_D;
    ROS_INFO_STREAM("Average time for 'D' :" << avg_time_D << " [ms]");
    iter_time_D++;

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
    ObjectExt_->extractObjectCandidate(accumulated_dRdt_, CloudFrame_next_,
                                       object_factor);
    // double dt_toc6 = timer::toc(); // milliseconds
    // ROS_INFO_STREAM("elapsed time for 'extractObjectCandidate' :" <<  dt_toc6
    // << " [ms]");

    // cv::imshow("after extractObjectCandidate", accumulated_dRdt_);
    //// update object_mask
    // object_mask = accumulated_dRdt>0;
    //  timer::tic();
    //  Fast Segment
    ObjectExt_->checkSegment(accumulated_dRdt_, CloudFrame_next_,
                             SegmentGround_->groundPtsIdx_next_);
    // double dt_toc7 = timer::toc(); // milliseconds
    // ROS_INFO_STREAM("elapsed time for 'checkSegment' :" <<  dt_toc7 << "
    // [ms]"); cv::imshow("after checkSegment", accumulated_dRdt_);
    //// update object_mask
    // object_mask = accumulated_dRdt>0;
    //  timer::tic();
    ObjectExt_->updateAccum(accumulated_dRdt_, accumulated_dRdt_score_);
    // double dt_toc8 = timer::toc(); // milliseconds
    // ROS_INFO_STREAM("elapsed time for 'updateAccum' :" <<  dt_toc8 << "
    // [ms]");

    double dt_toc4 = timer::toc();  // milliseconds
    sum_time_E += dt_toc4;
    avg_time_E = sum_time_E / iter_time_E;
    ROS_INFO_STREAM("Average time for 'E' :" << avg_time_E << " [ms]");
    iter_time_E++;

    // cv::imshow("after updateAccum", accumulated_dRdt_);

    timer::tic();
    ImageFill_->plugImageZeroHoles(accumulated_dRdt_, accumulated_dRdt_score_,
                                   CloudFrame_next_, object_factor);
    // double dt_toc9 = timer::toc(); // milliseconds
    // ROS_INFO_STREAM("elapsed time for 'plugImageZeroHoles' :" <<  dt_toc9 <<
    // " [ms]");

    // cv::imshow("after plugImageZeroHoles", accumulated_dRdt_);

    // timer::tic();
    ObjectExt_->updateAccumdRdt(CloudFrame_next_, accumulated_dRdt_,
                                accumulated_dRdt_score_, dRdt_,
                                SegmentGround_->groundPtsIdx_next_);
    // double dt_toc10 = timer::toc(); // milliseconds
    // ROS_INFO_STREAM("elapsed time for 'updateAccumdRdt' :" <<  dt_toc10 << "
    // [ms]");
    float* ptr_accumulated_dRdt = accumulated_dRdt_.ptr<float>(0);
    float* ptr_accumulated_dRdt_score = accumulated_dRdt_score_.ptr<float>(0);

    double dt_toc5 = timer::toc();  // milliseconds
    sum_time_F += dt_toc5;
    avg_time_F = sum_time_F / iter_time_F;
    ROS_INFO_STREAM("Average time for 'F' :" << avg_time_F << " [ms]");
    iter_time_F++;

    // cv::imshow("accumulated_dRdt", accumulated_dRdt_);
    // cv::waitKey(0);
    // exit(0);

    // PointCloudwithTime pcl_dynamic;
    // PointCloudwithTime pcl_static;
    // PointCloudwithTime pcl_static_wtime;

    // float* ptr_next_img_x =
    // CloudFrame_next_->str_rhopts_->img_x.ptr<float>(0); float* ptr_next_img_y
    // = CloudFrame_next_->str_rhopts_->img_y.ptr<float>(0); float*
    // ptr_next_img_z = CloudFrame_next_->str_rhopts_->img_z.ptr<float>(0);

    // for (int i = 0; i < img_height_; ++i)
    // {
    //     int i_ncols = i * img_width_;
    //     for (int j = 0; j < img_width_; ++j)
    //     {
    //         if (*(ptr_accumulated_dRdt + i_ncols + j)!=0)
    //         {
    //             pcl_dynamic.push_back(
    //                 pcl::PointXYZ(*(ptr_next_img_x + i_ncols + j),
    //                 *(ptr_next_img_y + i_ncols + j), *(ptr_next_img_z +
    //                 i_ncols + j)));
    //         }
    //         else
    //         {
    //             pcl_static.push_back(
    //                 pcl::PointXYZ(*(ptr_next_img_x + i_ncols + j),
    //                 *(ptr_next_img_y + i_ncols + j), *(ptr_next_img_z +
    //                 i_ncols + j)));
    //         }
    //     }
    // }

    // uchar* ptr_groundPtsIdx_next =
    // SegmentGround_->groundPtsIdx_next_.ptr<uchar>(0);

    // for (int i = 0; i < img_height_; ++i)
    // {
    //     int i_ncols = i * img_width_;
    //     for (int j = 0; j < img_width_; ++j)
    //     {
    //         if (*(ptr_groundPtsIdx_next + i_ncols + j)!=0) // dynamic
    //         {
    //             if
    //             (CloudFrame_next_->str_rhopts_->pts_per_pixel_index_valid[i_ncols
    //             + j].size() != 0)
    //             {
    //                 for (int k = 0; k <
    //                 CloudFrame_next_->str_rhopts_->pts_per_pixel_index_valid[i_ncols
    //                 + j].size(); ++k)
    //                 {
    //                     pcl_dynamic.push_back(
    //                         slam::PointXYZT((*p1)[CloudFrame_next_->str_rhopts_->pts_per_pixel_index_valid[i_ncols
    //                         + j][k]]));
    //                 }
    //             }
    //         }
    //         else // static
    //         {
    //             if
    //             (CloudFrame_next_->str_rhopts_->pts_per_pixel_index_valid[i_ncols
    //             + j].size() != 0)
    //             {
    //                 for (int k = 0; k <
    //                 CloudFrame_next_->str_rhopts_->pts_per_pixel_index_valid[i_ncols
    //                 + j].size(); ++k)
    //                 {
    //                     pcl_static.push_back(
    //                         slam::PointXYZT((*p1)[CloudFrame_next_->str_rhopts_->pts_per_pixel_index_valid[i_ncols
    //                         + j][k]]));
    //                 }
    //             }
    //         }
    //     }
    // }

    timer::tic();
    // pcl_static_wtime.reserve(p1->size());
    slam::PointXYZT new_pt;
    new_pt.x = 0.0;
    new_pt.y = 0.0;
    new_pt.z = 0.0;

    std::vector<int>* ptr_vec_tmp =
        CloudFrame_next_->str_rhopts_->pts_per_pixel_index_valid.data();

    ///////////////////////// for dynamic_pts /////////////////////////
    // /*
    for (int i = 0; i < img_height_; ++i) {
        int i_ncols = i * img_width_;
        for (int j = 0; j < img_width_; ++j) {
            const int i_ncols_j = i_ncols + j;
            const std::vector<int>& vec_tmp = ptr_vec_tmp[i_ncols_j];

            // static
            if (vec_tmp.size() != 0) {
                for (int k = 0; k < vec_tmp.size(); ++k) {
                    // pcl_static_.push_back((*p1)[vec_tmp[k]]);

                    const int& vec_value_tmp = vec_tmp[k];
                    if (*(ptr_accumulated_dRdt + i_ncols_j) != 0) {
                        new_pt.timestamp = (*p1)[vec_value_tmp].timestamp;
                        pcl_static_wtime_.push_back(new_pt);

                        // dynamic
                        for (int k = 0; k < vec_tmp.size(); ++k) {
                            pcl_dynamic_.push_back((*p1)[vec_value_tmp]);
                        }
                    } else {
                        pcl_static_wtime_.push_back((*p1)[vec_value_tmp]);
                        *(ptr_accumulated_dRdt_score + i_ncols_j) =
                            0;  //// update for next iteration
                    }
                }
            }
            // dynamic
            // if (*(ptr_accumulated_dRdt + i_ncols_j) != 0)
            // {
            //     if (vec_tmp.size() != 0)
            //     {
            //         for (int k = 0; k < vec_tmp.size(); ++k)
            //         {
            //             pcl_dynamic_.push_back((*p1)[vec_tmp[k]]);
            //         }
            //     }
            // }
        }
    } /*
       */
    ///////////////////////// for dynamic_pts /////////////////////////

    // for (int i = 0; i < img_height_; ++i)
    // {
    //     int i_ncols = i * img_width_;
    //     for (int j = 0; j < img_width_; ++j)
    //     {
    //         const int i_ncols_j = i_ncols + j;
    //         const std::vector<int>& vec_tmp = ptr_vec_tmp[i_ncols_j];
    //         if (*(ptr_accumulated_dRdt + i_ncols_j) != 0) // dynamic
    //         {
    //             if (vec_tmp.size() != 0)
    //             {
    //                 for (int k = 0; k < vec_tmp.size(); ++k)
    //                 {
    //                     pcl_dynamic_.push_back((*p1)[vec_tmp[k]]);
    //                 }
    //             }
    //         }
    //         else // static
    //         {
    //             if (vec_tmp.size() != 0)
    //             {
    //                 for (int k = 0; k < vec_tmp.size(); ++k)
    //                 {
    //                     pcl_static_.push_back(slam::PointXYZT((*p1)[vec_tmp[k]]));

    //                     const int& vec_value_tmp = vec_tmp[k];
    //                     slam::PointXYZT p1_point_tmp = (*p1)[vec_value_tmp];
    //                     new_pt.x = p1_point_tmp.x;
    //                     new_pt.y = p1_point_tmp.y;
    //                     new_pt.z = p1_point_tmp.z;
    //                     new_pt.timestamp =
    //                     (*p1_w_time)[vec_value_tmp].timestamp;
    //                         // std::cout <<vec_tmp[k] << "      "
    //                         <<new_pt.timestamp <<std::endl;
    //                     pcl_static_wtime_.push_back(new_pt);
    //                 }
    //             }
    //             *(ptr_accumulated_dRdt_score + i_ncols + j) = 0; //// update
    //             for next iteration
    //         }
    //     }
    // }

    ///////////////////////// for all_pts /////////////////////////
    /*
    for (int i = 0; i < img_height_; ++i)
    {
        int i_ncols = i * img_width_;
        for (int j = 0; j < img_width_; ++j)
        {
            const int i_ncols_j = i_ncols + j;
            const std::vector<int> &vec_tmp = ptr_vec_tmp[i_ncols_j];
            if (vec_tmp.size() != 0)
            {
                for (int k = 0; k < vec_tmp.size(); ++k)
                {
                    pcl_static_.push_back(slam::PointXYZT((*p1)[vec_tmp[k]]));

                    const int &vec_value_tmp = vec_tmp[k];
                    slam::PointXYZT p1_point_tmp = (*p1)[vec_value_tmp];
                    new_pt.x = p1_point_tmp.x;
                    new_pt.y = p1_point_tmp.y;
                    new_pt.z = p1_point_tmp.z;
                    new_pt.timestamp = (*p1)[vec_value_tmp].timestamp;
                    // std::cout <<vec_tmp[k] << "      " <<new_pt.timestamp
    <<std::endl; pcl_static_wtime_.push_back(new_pt);
                }
            }
            *(ptr_accumulated_dRdt_score + i_ncols + j) = 0; //// update for
    next iteration
        }
    }/*
    */
    ///////////////////////// for all_pts /////////////////////////

    // double dt_toc11 = timer::toc(); // milliseconds
    // ROS_INFO_STREAM("elapsed time for 'segmentWholePts' :" <<  dt_toc11 << "
    // [ms]");

    // timer::tic();
    //// visualization ////
    // dynamic //

    pcl::toROSMsg(pcl_dynamic_, converted_msg_d_);

    // save pcl_dynamic points
    /*
    std::string folder_name = "/home/junhakim/label_results/" + dataset_name_ +
    "/" + data_number_; std::string cloud_name = folder_name + "/dynamic_" +
    WithLeadingZerosStr(cnt_data) + ".pcd"; if(pcl_dynamic_.size() > 0)
    {
        pcl::io::savePCDFileBinary(cloud_name, pcl_dynamic_);
    }/*
    */

    converted_msg_d_.header.frame_id = "map";
    converted_msg_d_.header.stamp = cloudHeader.stamp;
    pub_dynamic_pts_.publish(converted_msg_d_);

    // sensor_msgs::PointCloud2 converted_msg_s;
    // pcl::toROSMsg(pcl_static, converted_msg_s);
    // converted_msg_s.header.frame_id = "/map";
    // converted_msg_s.header.stamp = cloudHeader.stamp;
    // pub_static_pts_.publish(converted_msg_s);

    // sensor_msgs::PointCloud2 converted_msg_s;
    // pcl::toROSMsg(*p1_w_time, converted_msg_s);
    // converted_msg_s.header.frame_id = "/map";
    // converted_msg_s.header.stamp = cloudHeader.stamp;
    // pub_static_pts_.publish(converted_msg_s);

    pcl::toROSMsg(pcl_static_wtime_, converted_msg_s_);
    converted_msg_s_.header.frame_id = "map";
    converted_msg_s_.header.stamp = cloudHeader.stamp;
    pub_static_pts_.publish(converted_msg_s_);

    pub_num += 1;
    ROS_INFO_STREAM("pub_num = " << pub_num << ", "
                                 << "size of pcl_static_wtime: "
                                 << pcl_static_wtime_.size());
    ROS_INFO_STREAM("pub_num = " << pub_num << ", "
                                 << "size of pcl_dynamic_: "
                                 << pcl_dynamic_.size());

    // if (cnt_data == 3)
    // {exit(0);}
    // double dt_toc12 = timer::toc(); // milliseconds
    // ROS_INFO_STREAM("elapsed time for 'publish' :" <<  dt_toc12 << " [ms]");

    // timer::tic();
    copyStructAndinitialize(p1, p0, cnt_data);
    // double dt_toc13 = timer::toc(); // milliseconds
    // ROS_INFO_STREAM("elapsed time for 'copyRemove' :" <<  dt_toc13 << "
    // [ms]");

    double dt_toc6 = timer::toc();  // milliseconds
    sum_time_G += dt_toc6;
    avg_time_G = sum_time_G / iter_time_G;
    ROS_INFO_STREAM("Average time for 'G' :" << avg_time_G << " [ms]");
    iter_time_G++;

    // double dt_toc_total = dt_toc2 + dt_toc3 + dt_toc4 + dt_toc5 + dt_toc6 +
    // dt_toc7 + dt_toc8 + dt_toc9 + dt_toc10 + dt_toc11 + dt_toc12 + dt_toc13;
    // ROS_INFO_STREAM("elapsed time for 'total' :" <<  dt_toc_total << "
    // [ms]");

    ROS_INFO_STREAM("================== End of the solver ==================");
    // cv::imshow("final", accumulated_dRdt_);
    // cv::waitKey(0);
};

std::string MaplessDynamic::WithLeadingZerosStr(int num) {
    size_t leading_zeros_num = 6;
    auto counter_str = std::to_string(num);
    return std::string(leading_zeros_num - counter_str.size(), '0')
        .append(counter_str);
}

void MaplessDynamic::copyStructAndinitialize(PointCloudwithTime::Ptr p1,
                                             PointCloudwithTime::Ptr p0,
                                             int cnt_data) {
    pcl_dynamic_.resize(0);
    // pcl_static_.resize(0);
    pcl_static_wtime_.resize(0);
    // copy Cur to Next
    std::shared_ptr<StrRhoPts> cur_tmp = CloudFrame_cur_->str_rhopts_;
    std::shared_ptr<StrRhoPts> next_tmp = CloudFrame_next_->str_rhopts_;

    {
        // CloudFrame_cur_->str_rhopts_->pts        =
        // CloudFrame_next_->str_rhopts_->pts;

        cur_tmp->rho.resize(0);
        cur_tmp->phi.resize(0);
        cur_tmp->theta.resize(0);
        std::copy(next_tmp->rho.begin(), next_tmp->rho.end(),
                  cur_tmp->rho.begin());
        std::copy(next_tmp->phi.begin(), next_tmp->phi.end(),
                  cur_tmp->phi.begin());
        std::copy(next_tmp->theta.begin(), next_tmp->theta.end(),
                  cur_tmp->theta.begin());

        next_tmp->img_rho.copyTo(cur_tmp->img_rho);
        next_tmp->img_index.copyTo(cur_tmp->img_index);
        next_tmp->img_x.copyTo(cur_tmp->img_x);
        next_tmp->img_y.copyTo(cur_tmp->img_y);
        next_tmp->img_z.copyTo(cur_tmp->img_z);

        // cur_tmp->pts_per_pixel_n.resize(0);
        // std::copy(next_tmp->pts_per_pixel_n.begin(),
        // next_tmp->pts_per_pixel_n.end(), cur_tmp->pts_per_pixel_n.begin());

        for (int i = 0; i < img_height_ * img_width_; ++i) {
            if (cur_tmp->pts_per_pixel_index[i].size() != 0) {
                cur_tmp->pts_per_pixel_index[i].resize(0);
                std::copy(next_tmp->pts_per_pixel_index[i].begin(),
                          next_tmp->pts_per_pixel_index[i].end(),
                          cur_tmp->pts_per_pixel_index[i].begin());
            }

            if (cur_tmp->pts_per_pixel_rho[i].size() != 0) {
                cur_tmp->pts_per_pixel_rho[i].resize(0);
                std::copy(next_tmp->pts_per_pixel_rho[i].begin(),
                          next_tmp->pts_per_pixel_rho[i].end(),
                          cur_tmp->pts_per_pixel_rho[i].begin());
            }

            if (cur_tmp->pts_per_pixel_index_valid[i].size() != 0) {
                cur_tmp->pts_per_pixel_index_valid[i].resize(0);
                std::copy(next_tmp->pts_per_pixel_index_valid[i].begin(),
                          next_tmp->pts_per_pixel_index_valid[i].end(),
                          cur_tmp->pts_per_pixel_index_valid[i].begin());
            }
        }

        next_tmp->img_restore_mask.copyTo(cur_tmp->img_restore_mask);
        next_tmp->img_restore_warp_mask.copyTo(cur_tmp->img_restore_warp_mask);
    }

    // // only in test
    // if (rosbag_play_ == false) {
    //     p0_pcl_test_->resize(0);
    //     pcl::copyPointCloud(*p1, *p0_pcl_test_);
    //     p1->resize(0);
    // }

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
    // test_data_type_ = "KITTI";

    // END YOUR CODE
};

void MaplessDynamic::calculateGTpose(int cnt_line) {
    Pose T_tmp = Pose::Zero(4, 4);
    std::vector<float> pose_tmp = all_pose_[cnt_line];
    // T_tmp.block<1,4>(0,0) << pose_tmp[0], pose_tmp[1], pose_tmp[2],
    // pose_tmp[3]; T_tmp.block<1,4>(1,0) << pose_tmp[4], pose_tmp[5],
    // pose_tmp[6], pose_tmp[7]; T_tmp.block<1,4>(2,0) << pose_tmp[8],
    // pose_tmp[9], pose_tmp[10], pose_tmp[11];
}

//////////// TO READ KITTI BIN FILES ////////////

void MaplessDynamic::read_filelists(const std::string& dir_path,
                                    std::vector<std::string>& out_filelsits,
                                    std::string type) {
    struct dirent* ptr;
    DIR* dir;
    dir = opendir(dir_path.c_str());
    out_filelsits.clear();
    while ((ptr = readdir(dir)) != NULL) {
        std::string tmp_file = ptr->d_name;
        if (tmp_file[0] == '.') continue;
        if (type.size() <= 0) {
            out_filelsits.push_back(ptr->d_name);
        } else {
            if (tmp_file.size() < type.size()) continue;
            std::string tmp_cut_type =
                tmp_file.substr(tmp_file.size() - type.size(), type.size());
            if (tmp_cut_type == type) {
                out_filelsits.push_back(ptr->d_name);
            }
        }
    }
}

bool computePairNum(std::string pair1, std::string pair2) {
    return pair1 < pair2;
}

void MaplessDynamic::sort_filelists(std::vector<std::string>& filists,
                                    std::string type) {
    if (filists.empty()) return;

    std::sort(filists.begin(), filists.end(), computePairNum);
}

void MaplessDynamic::readKittiPclBinData(std::string& in_file, int file_num) {
    static int valid_cnt = 0;
    // load point cloud
    std::fstream input(in_file.c_str(), std::ios::in | std::ios::binary);
    if (!input.good()) {
        std::cerr << "Could not read file: " << in_file << std::endl;
        exit(EXIT_FAILURE);
    }
    input.seekg(0, std::ios::beg);

    // PointCloudwithTime::Ptr points (new PointCloudwithTime);

    int i;
    for (i = 0; input.good() && !input.eof(); ++i) {
        slam::PointXYZT point;
        input.read((char*)&point.x, 3 * sizeof(float));
        input.read((char*)&point.timestamp, sizeof(float));
        data_buf_[valid_cnt]->pcl_->push_back(point);
    }

    data_buf_[valid_cnt]->pcl_->resize(i - 1);
    pcl::toROSMsg(*data_buf_[valid_cnt]->pcl_,
                  *(data_buf_[valid_cnt]->pcl_msg_));
    // std::cout<< "bin_num: "<<file_num <<" "<<"num_pts:
    // "<<data_buf_[valid_cnt]->pcl_->size() <<std::endl; std::cout<< "bin_num:
    // "<<file_num <<" "<<"final pts:
    // "<<data_buf_[file_num]->pcl_->at((data_buf_[file_num]->pcl_)->size()-2)
    // <<std::endl; std::cout<< "valid_num: "<<valid_cnt <<" "<<"num_pts:
    // "<<data_buf_[valid_cnt]->pcl_msg_->width <<std::endl;
    input.close();
    valid_cnt += 1;
}

void MaplessDynamic::countZerofloat(cv::Mat& input_mat) {
    int n_row = input_mat.rows;
    int n_col = input_mat.cols;
    float* ptr_input_mat = input_mat.ptr<float>(0);
    int cnt = 0;
    for (int i = 0; i < n_row; ++i) {
        int i_ncols = i * n_col;
        for (int j = 0; j < n_col; ++j) {
            if (*(ptr_input_mat + i_ncols + j) != 0) {
                cnt += 1;
            }
        }
    }
    std::cout << "# of non zero: " << cnt << std::endl;
}

void MaplessDynamic::countZeroint(cv::Mat& input_mat) {
    int n_row = input_mat.rows;
    int n_col = input_mat.cols;
    int* ptr_input_mat = input_mat.ptr<int>(0);
    int cnt = 0;
    for (int i = 0; i < n_row; ++i) {
        int i_ncols = i * n_col;
        for (int j = 0; j < n_col; ++j) {
            if (*(ptr_input_mat + i_ncols + j) != 0) {
                cnt += 1;
            }
        }
    }
    std::cout << "# of non zero: " << cnt << std::endl;
}

void MaplessDynamic::countZerouchar(cv::Mat& input_mat) {
    int n_row = input_mat.rows;
    int n_col = input_mat.cols;
    uchar* ptr_input_mat = input_mat.ptr<uchar>(0);
    int cnt = 0;
    for (int i = 0; i < n_row; ++i) {
        int i_ncols = i * n_col;
        for (int j = 0; j < n_col; ++j) {
            if (*(ptr_input_mat + i_ncols + j) != 0) {
                cnt += 1;
            }
        }
    }
    std::cout << "# of non zero: " << cnt << std::endl;
}