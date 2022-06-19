#ifndef _MAPLESS_DYNAMIC_H_
#define _MAPLESS_DYNAMIC_H_

#include <iostream>

#include <Eigen/Dense>

#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>

#include "defines.h"
#include <vector>

#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

struct S_TEST_DATA
{
    Pose T_gt;
    pcl::PointCloud<pcl::PointXYZI>::Ptr pcl_;
};

class MaplessDynamic{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW;

public:
    // DECLARE PUBLIC VARIABLES 
    // * NOT RECOMMAND TO MAKE PUBLIC VARIABLES
    
private:
    // DECLARE PRIVATE VARIABLES
    bool test_flag_;
    int img_height_;
    int img_width_;
    int object_threshold_;
    int score_cnt_;

    float alpha_;
    float beta_;

    cv::Mat* accumulated_dRdt_;
    cv::Mat* accumulated_dRdt_score_;
    cv::Mat* background_mask_;

private: 
    //test
    std::string test_data_type_;
    std::vector<std::vector<float>> all_pose;
    std::vector<S_TEST_DATA *> test_data_buf_;
    int test_n_data_;
    std::vector<Pose *> T_gt_;
    
    std::vector<std::string> file_lists_;

public:
    MaplessDynamic(bool test_flag); // constructor
    ~MaplessDynamic(); // destructor

    void TEST(); // algorithm test function with a loaded data

    void solve(        
        /* inputs */ 
        const sensor_msgs::PointCloud2& p0, const sensor_msgs::PointCloud2& p1, const Pose& T01, 
        /* outputs */
        Mask& mask1);

// From this, declare your own sub-functions. 
private:
    void getUserSettingParameters();
    // void func1();
    // ...

private:
    //test
    void loadTestData();
    void calculateGTpose(int cnt_line);
    void read_filelists(const std::string& dir_path,std::vector<std::string>& out_filelsits,std::string type);
    void sort_filelists(std::vector<std::string>& filists, std::string type);
    void readKittiPclBinData(std::string &in_file, int file_num);

};

#endif