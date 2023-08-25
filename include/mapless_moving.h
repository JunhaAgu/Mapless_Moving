#ifndef _MAPLESS_MOVING_H_
#define _MAPLESS_MOVING_H_

#include <pcl/common/transforms.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>

#include <Eigen/Dense>
#include <iostream>
#include <vector>

#include "defines.h"
#include "timer.h"
// #include <pcl/registration/icp.h>

#include <algorithm>
#include <opencv2/core.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/opencv.hpp>

#include "cloud_frame.h"
#include "dR_calc.h"
#include "image_fill.h"
#include "object_ext.h"
#include "pcl_warp.h"
#include "segment_ground.h"
#include "user_param.h"

struct TestData {
    Pose T_gt_;
    PointCloudwithTime::Ptr pcl_;
    sensor_msgs::PointCloud2* pcl_msg_;
};

class MaplessDynamic {
   public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW;

   public:
    // DECLARE PUBLIC VARIABLES
    // * NOT RECOMMAND TO MAKE PUBLIC VARIABLES

   public:
    std::unique_ptr<CloudFrame> CloudFrame_cur_;
    std::unique_ptr<CloudFrame> CloudFrame_next_;
    std::unique_ptr<CloudFrame> CloudFrame_cur_warped_;
    std::unique_ptr<CloudFrame> CloudFrame_warpPointcloud_;
    std::unique_ptr<SegmentGround> SegmentGround_;
    std::unique_ptr<dRCalc> dRCalc_;
    std::unique_ptr<PclWarp> PclWarp_;
    std::unique_ptr<ObjectExt> ObjectExt_;
    std::unique_ptr<ImageFill> ImageFill_;

    ros::Publisher pub_static_pts_;

   private:
    // DECLARE PRIVATE VARIABLES
    ros::NodeHandle nh_;
    ros::Publisher pub_dynamic_pts_;
    // ros::Publisher pub_static_pts_;

    bool rosbag_play_;
    std::string dataset_kitti_dir_;
    std::string dataset_carla_dir_;
    std::string dataset_name_;
    std::string data_number_;

    int n_valid_data_;

    int img_height_;
    int img_width_;

    float alpha_;
    float beta_;

    cv::Mat accumulated_dRdt_;
    cv::Mat accumulated_dRdt_score_;

    Pose T_next2cur_;

    cv::Mat dRdt_;

   private:
    // genRangeImages
    std::vector<float> v_angle_;

   private:
    // test
    std::vector<std::vector<float>> all_pose_;
    std::vector<Pose> all_T_gt_;
    std::vector<int> valid_data_;
    int n_data_;
    bool is_initialized_test_;
    sensor_msgs::PointCloud2 p0_msg_test_;
    PointCloudwithTime::Ptr p0_pcl_test_;
    sensor_msgs::PointCloud2 p1_msg_test_;
    PointCloudwithTime::Ptr p1_pcl_test_;

    std::vector<std::string> file_lists_;

   public:
    std::vector<TestData*> data_buf_;

    std_msgs::Header cloudHeader_test_;

    // pcl::IterativeClosestPoint<pcl::PointXYZI, pcl::PointXYZI> icp_;

   public:
    PointCloudwithTime pcl_dynamic_;
    PointCloudwithTime pcl_static_;
    PointCloudwithTime pcl_static_wtime_;

    sensor_msgs::PointCloud2 converted_msg_d_;
    sensor_msgs::PointCloud2 converted_msg_s_;

   public:
    MaplessDynamic(ros::NodeHandle& nh, bool rosbag_play,
                   std::string& dataset_kitti_dir,
                   std::string& dataset_carla_dir, std::string& dataset_name,
                   std::string& data_number);  // constructor
    ~MaplessDynamic();                         // destructor

    void TEST();  // algorithm test function with a loaded data

    void solve(
        /* inputs */
        PointCloudwithTime::Ptr p0, PointCloudwithTime::Ptr p1, const Pose& T10,
        /* outputs */
        Mask& mask1, int cnt, std_msgs::Header& cloudHeader);

    // From this, declare your own sub-functions.
   private:
    void getUserSettingParameters();

    void copyStructAndinitialize(PointCloudwithTime::Ptr p1,
                                 PointCloudwithTime::Ptr p0, int cnt_data);

    void countZerofloat(cv::Mat& input_mat);
    void countZeroint(cv::Mat& input_mat);
    void countZerouchar(cv::Mat& input_mat);

    std::string WithLeadingZerosStr(int num);

   private:
    // test: load data
    void loadTestData();
    void calculateGTpose(int cnt_line);
    void read_filelists(const std::string& dir_path,
                        std::vector<std::string>& out_filelsits,
                        std::string type);
    void sort_filelists(std::vector<std::string>& filists, std::string type);
    void readKittiPclBinData(std::string& in_file, int file_num);
};

#endif