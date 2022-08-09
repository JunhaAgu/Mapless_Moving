#ifndef _DR_CALC_H_
#define _DR_CALC_H_

#include "defines.h"
#include "user_param.h"
#include <vector>
#include "cloud_frame.h"
#include "timer.h"

class MaplessDynamic;
class UserParam;
class CloudFrame;

class dRCalc
{
    private:
        std::vector<float> v_angle_;
        int n_ring_;
        int n_radial_;

    public:
        pcl::PointCloud<pcl::PointXYZ>::Ptr velo_cur_;
        pcl::PointCloud<pcl::PointXYZ>::Ptr cur_pts_warped_;

    public:
        dRCalc(const std::unique_ptr<UserParam> &user_param);
        ~dRCalc();

        void dR_warpPointcloud(std::unique_ptr<CloudFrame>& CloudFrame_next, std::unique_ptr<CloudFrame>& CloudFrame_cur, std::unique_ptr<CloudFrame>& CloudFrame_cur_warped,
                                pcl::PointCloud<pcl::PointXYZ>::Ptr p0, Pose &T01, int cnt_data, cv::Mat &dRdt);
        
        void compensateCurRhoZeroWarp(std::unique_ptr<CloudFrame>& CloudFrame_cur);
        
        void interpRangeImageMin(std::unique_ptr<CloudFrame>& CloudFrame_in);
        
        void interpPtsWarp(std::unique_ptr<CloudFrame>& CloudFrame_in, int n_ring, int n_radial);
};


#endif