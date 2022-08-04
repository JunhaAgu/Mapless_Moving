#ifndef _PCL_WARP_H_
#define _PCL_WARP_H_

#include "defines.h"
#include "user_param.h"
#include "cloud_frame.h"

class MaplessDynamic;
class UserParam;

class PclWarp
{
private:
    int img_height_;
    int img_width_;

public:
    StrRhoPts *str_warpPointcloud_;
    pcl::PointCloud<pcl::PointXYZ> velo_xyz_;
    pcl::PointCloud<pcl::PointXYZ>::Ptr pts_warpewd_;

public:
    PclWarp(const std::unique_ptr<UserParam> &user_param);
    ~PclWarp();

    void warpPointcloud(StrRhoPts *str_cur, const Pose &T01, cv::Mat &mat_in, int cnt_data, std::unique_ptr<CloudFrame> &CloudFrame);

    void initializeStructAndPcl();
};

#endif