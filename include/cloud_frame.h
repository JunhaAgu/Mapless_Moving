#ifndef _CLOUD_FRAME_H_
#define _CLOUD_FRAME_H_

#include "defines.h"
#include "user_param.h"

class MaplessDynamic;
class UserParam;

class CloudFrame
{
    friend class dRCalc;
    
    private:
        float azimuth_res_;
        float az_step_;
        int n_radial_;
        int n_ring_;
        int n_pts_;
        std::vector<float> v_angle_;
    
    public:
        CloudFrame(const std::unique_ptr<UserParam>& user_param);
        ~CloudFrame();

        void genRangeImages(pcl::PointCloud<pcl::PointXYZ> &pcl_in, StrRhoPts *str_in, bool cur_next);

        void calcuateRho(pcl::PointCloud<pcl::PointXYZ> &pcl_in, StrRhoPts *str_in);

        void makeRangeImageAndPtsPerPixel(StrRhoPts *str_in, int n_pts, int n_ring, int n_radial, float az_step);
        
        void interpRangeImage(StrRhoPts *str_in, int n_ring, int n_radial, bool cur_next);

        void interpPts(pcl::PointCloud<pcl::PointXYZ> &pcl_in, StrRhoPts *str_in1, int n_ring, int n_radial, bool cur_next);
};


#endif