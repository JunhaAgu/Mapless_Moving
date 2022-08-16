#ifndef _CLOUD_FRAME_H_
#define _CLOUD_FRAME_H_

#include "defines.h"
#include "user_param.h"
#include "timer.h"

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

        int img_height_;
        int img_width_;

    public:
        std::shared_ptr<StrRhoPts> str_rhopts_;

    public:
        CloudFrame(const std::unique_ptr<UserParam>& user_param);
        ~CloudFrame();

        void genRangeImages(pcl::PointCloud<pcl::PointXYZ>::Ptr pcl_in, bool cur_next);

        void genRangeImages_dR(pcl::PointCloud<pcl::PointXYZ>::Ptr pcl_in, bool cur_next);

        void genRangeImages_noComp(pcl::PointCloud<pcl::PointXYZ>::Ptr pcl_in, bool cur_next);

        void calcuateRho(pcl::PointCloud<pcl::PointXYZ>::Ptr pcl_in, bool cur_next);

        void makeRangeImageAndPtsPerPixel(bool cur_next);

        void makeRangeImageAndPtsPerPixel_dR(bool cur_next);
        
        void interpRangeImage(bool cur_next);

        void interpRangeImage_dR(bool cur_next);

        void interpPts(pcl::PointCloud<pcl::PointXYZ>::Ptr pcl_in, bool cur_next);

        void reset();
};


#endif