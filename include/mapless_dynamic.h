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

#include <opencv2/core.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/highgui.hpp>

struct TestData 
{
    Pose T_gt_;
    pcl::PointCloud<pcl::PointXYZI>::Ptr pcl_;
    sensor_msgs::PointCloud2* pcl_msg_;
};

struct StrRhoPts
{
    pcl::PointCloud<pcl::PointXYZ>::Ptr pts;
    pcl::PointCloud<pcl::PointXYZ>::Ptr ptsInImage;

    std::vector<float> rho;
    std::vector<float> phi;
    std::vector<float> theta;

    cv::Mat img_rho;
    cv::Mat img_index;

    cv::Mat img_x;
    cv::Mat img_y;
    cv::Mat img_z;

    std::vector<int> pts_per_pixel_n;
    std::vector<std::vector<int>> pts_per_pixel_index;
    std::vector<std::vector<float>> pts_per_pixel_rho;
    std::vector<int> valid_original_pts_idx_;

    //interpRangeImage
    cv::Mat img_restore_mask;
    cv::Mat img_restore_warp_mask;
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

    cv::Mat accumulated_dRdt_;
    cv::Mat accumulated_dRdt_score_;
    cv::Mat background_mask_;

    StrRhoPts* str_cur_;
    StrRhoPts* str_next_;

    StrRhoPts* str_cur_warped_;

    StrRhoPts* str_warpPointcloud_;

    Pose T_next2cur_;
    
    //dR_warpPointcloud
    pcl::PointCloud<pcl::PointXYZ> velo_cur_;
    pcl::PointCloud<pcl::PointXYZ>::Ptr ptr_cur_pts_warped_;
    cv::Mat residual_;
    
    pcl::PointCloud<pcl::PointXYZ> velo_xyz_;
    pcl::PointCloud<pcl::PointXYZ>::Ptr pts_warpewd_;

private:
    //genRangeImages
    std::vector<float> v_angle_;

private: 
    //test
    std::string test_data_type_;
    std::vector<TestData *> data_buf_;
    std::vector<std::vector<float>> all_pose_;
    std::vector<Pose> all_T_gt_;
    std::vector<int> valid_data_;
    int n_data_;
    bool is_initialized_test_;
    Mask                     mask0_test_;
    sensor_msgs::PointCloud2 p0_msg_test_;
    pcl::PointCloud<pcl::PointXYZ> p0_pcl_test_;
    sensor_msgs::PointCloud2 p1_msg_test_;
    pcl::PointCloud<pcl::PointXYZ> p1_pcl_test_;

    std::vector<std::string> file_lists_;

public:
    MaplessDynamic(bool test_flag); // constructor
    ~MaplessDynamic(); // destructor

    void TEST(); // algorithm test function with a loaded data

    void solve(        
        /* inputs */ 
        pcl::PointCloud<pcl::PointXYZ>& p0, pcl::PointCloud<pcl::PointXYZ>& p1, const Pose& T01, 
        /* outputs */
        Mask& mask1);

// From this, declare your own sub-functions. 
private:
    void getUserSettingParameters();
    // void func1();
    // ...
    void genRangeImages(pcl::PointCloud<pcl::PointXYZ>& pcl_in1, StrRhoPts* str_in1);
    void calcuateRho(pcl::PointCloud<pcl::PointXYZ>& pcl_in, StrRhoPts* str_in);
    void makeRangeImageAndPtsPerPixel(StrRhoPts* str_in, int n_pts, int n_ring,int n_radial,float az_step);
    void interpRangeImage(StrRhoPts* str_in, int n_ring, int n_radial);
    void interpPts(pcl::PointCloud<pcl::PointXYZ>& pcl_in, StrRhoPts* str_in1, int n_ring, int n_radial);
    
    void dR_warpPointcloud(pcl::PointCloud<pcl::PointXYZ>& p0, Pose& T01);
    void compensateCurRhoZeroWarp(StrRhoPts* str_cur_, int n_ring, int n_radial, std::vector<float>& v_angle_, pcl::PointCloud<pcl::PointXYZ>& velo_cur_);
    void interpRangeImageMin(StrRhoPts* str_in, int n_ring, int n_radial);
    void interpPtsWarp(StrRhoPts* str_cur_warped_, int n_ring, int n_radial);
    
    void warpPointcloud(StrRhoPts* str_cur_, const Pose& T01, cv::Mat& mat_in);
    void filterOutAccumdR(StrRhoPts* str_next_,StrRhoPts* str_cur_warped_,cv::Mat& accumulated_dRdt_,cv::Mat& accumulated_dRdt_score_,cv::Mat& residual_);
    void extractObjectCandidate(cv::Mat& accumulated_dRdt, StrRhoPts* str_next, int object_threshold);


private:
    //test: load data
    void loadTestData();
    void calculateGTpose(int cnt_line);
    void read_filelists(const std::string& dir_path,std::vector<std::string>& out_filelsits,std::string type);
    void sort_filelists(std::vector<std::string>& filists, std::string type);
    void readKittiPclBinData(std::string &in_file, int file_num);

};

#endif