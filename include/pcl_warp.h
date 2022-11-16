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
    PointCloudwithTime::Ptr velo_xyz_;
    PointCloudwithTime::Ptr pts_warpewd_;

public:
    PclWarp(const std::unique_ptr<UserParam> &user_param);
    ~PclWarp();

    void warpPointcloud(std::unique_ptr<CloudFrame>& CloudFrame_in, std::unique_ptr<CloudFrame>& CloudFrame_warpPointcloud, const Pose &T10, cv::Mat &mat_in, int cnt_data);

    void initializeStructAndPcl(std::unique_ptr<CloudFrame>& CloudFrame_warpPointcloud);

    void reset();
};

#endif