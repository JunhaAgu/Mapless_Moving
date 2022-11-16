#ifndef _MAPLESS_DYNAMIC_H_
#define _MAPLESS_DYNAMIC_H_

#include <iostream>

#include <Eigen/Dense>

#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>

#include "defines.h"
#include <vector>
#include "timer.h"

#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/common/transforms.h>
// #include <pcl/registration/icp.h>

#include <opencv2/core.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/opencv.hpp>

#include <algorithm>

#include "user_param.h"

#include "cloud_frame.h"
#include "segment_ground.h"
#include "dR_calc.h"
#include "pcl_warp.h"
#include "object_ext.h"
#include "image_fill.h"

struct TestData 
{
    Pose T_gt_;
    CloudMessageT::Ptr pcl_;
    sensor_msgs::PointCloud2* pcl_msg_;
};

class MaplessDynamic{

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
    //genRangeImages
    std::vector<float> v_angle_;

private: 
    //test
    std::vector<std::vector<float>> all_pose_;
    std::vector<Pose> all_T_gt_;
    std::vector<int> valid_data_;
    int n_data_;
    bool is_initialized_test_;
    Mask                     mask0_test_;
    sensor_msgs::PointCloud2 p0_msg_test_;
    pcl::PointCloud<slam::PointXYZT>::Ptr p0_pcl_test_;
    sensor_msgs::PointCloud2 p1_msg_test_;
    pcl::PointCloud<slam::PointXYZT>::Ptr p1_pcl_test_;

    std::vector<std::string> file_lists_;

public:
    std::vector<TestData *> data_buf_;

    std_msgs::Header cloudHeader_test_;

    // pcl::IterativeClosestPoint<pcl::PointXYZI, pcl::PointXYZI> icp_;

public:
    CloudMessageT pcl_dynamic_;
    CloudMessageT pcl_static_;
    CloudMessageT pcl_static_wtime_;

    sensor_msgs::PointCloud2 converted_msg_d_;
    sensor_msgs::PointCloud2 converted_msg_s_;

public:
    MaplessDynamic(ros::NodeHandle& nh, bool rosbag_play, std::string& dataset_name, std::string& data_number); // constructor
    ~MaplessDynamic(); // destructor

    void TEST(); // algorithm test function with a loaded data

    void solve(        
        /* inputs */ 
        CloudMessageT::Ptr p0, CloudMessageT::Ptr p1, const Pose& T10, 
        /* outputs */
        Mask& mask1, int cnt, std_msgs::Header& cloudHeader);

// From this, declare your own sub-functions. 
private:
    void getUserSettingParameters();

    void copyStructAndinitialize(CloudMessageT::Ptr p1, CloudMessageT::Ptr p0, int cnt_data);

    void countZerofloat(cv::Mat& input_mat);
    void countZeroint(cv::Mat& input_mat);
    void countZerouchar(cv::Mat& input_mat);

    std::string WithLeadingZerosStr(int num);

private:
    //test: load data
    void loadTestData();
    void calculateGTpose(int cnt_line);
    void read_filelists(const std::string& dir_path,std::vector<std::string>& out_filelsits,std::string type);
    void sort_filelists(std::vector<std::string>& filists, std::string type);
    void readKittiPclBinData(std::string &in_file, int file_num);

};

#endif