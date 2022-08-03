#ifndef _DR_CALC_H_
#define _DR_CALC_H_

#include "defines.h"
#include "user_param.h"
#include <vector>
#include "cloud_frame.h"

class MaplessDynamic;
class UserParam;
class CloudFrame;

class dRCalc
{
    private:
        std::vector<float> v_angle_;

    public:
        dRCalc(const std::unique_ptr<UserParam> &user_param);
        ~dRCalc();

        void dR_warpPointcloud(StrRhoPts *str_next, StrRhoPts *str_cur, pcl::PointCloud<pcl::PointXYZ> &p0, Pose &T01, int cnt_data, StrRhoPts *str_cur_warped, cv::Mat &dRdt, 
                                pcl::PointCloud<pcl::PointXYZ>& velo_cur, pcl::PointCloud<pcl::PointXYZ>::Ptr ptr_cur_pts_warped, std::unique_ptr<CloudFrame>& CloudFrame);
        
        void compensateCurRhoZeroWarp(StrRhoPts *str_cur, int n_ring, int n_radial, std::vector<float> &v_angle, pcl::PointCloud<pcl::PointXYZ> &velo_cur);
        
        void interpRangeImageMin(StrRhoPts *str_in, int n_ring, int n_radial);
        
        void interpPtsWarp(StrRhoPts *str_in, int n_ring, int n_radial);
};


#endif