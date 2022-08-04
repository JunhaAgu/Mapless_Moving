#include "user_param.h"
#include <string>

UserParam::UserParam()
{
    this->getUserSettingParameters();
};

UserParam::~UserParam()
{
    // destructor
};

void UserParam::getUserSettingParameters()
{
    std::string data_type = "KITTI"; // "CARLA"

    cloud_filter_param_.h_factor_ = 5;
    cloud_filter_param_.v_factor_ = 1;
    cloud_filter_param_.azimuth_res_ = 0.08;

    this->calVangle(data_type);

    ground_segment_param_.downsample_size = 10;
    ransac_param_.iter_ = 50;
    ransac_param_.thr_ = 0.1;
    ransac_param_.a_thr_ = 0.1;
    ransac_param_.b_thr_[0] = -0.5;
    ransac_param_.b_thr_[1] = -1.2;
    ransac_param_.min_inlier_ = 5;
    ransac_param_.n_sample_ = 2;

    sensor_spec_.channel_ = 64;

    image_param_.height_    = sensor_spec_.channel_ / cloud_filter_param_.v_factor_ ;
    image_param_.width_     = 360.0 / cloud_filter_param_.azimuth_res_ + 1;

    obejct_param_.thr_object_ = 30;
    obejct_param_.alpha_ = 0.3;
    obejct_param_.beta_ = 0.1;
};

void UserParam::calVangle(std::string& data_type)
{
    // sensor_spec_kitti_.v_angle_
    if (data_type == "KITTI")
    {
        float inter_top = (2.5 - (-8.0)) / (32 / cloud_filter_param_.v_factor_ - 1);
        for (int i = 0; i < 32 / cloud_filter_param_.v_factor_; ++i)
        {
            if (i == 31)
            {
                sensor_spec_.v_angle_.push_back(-8.0);
            }
            else
            {
                sensor_spec_.v_angle_.push_back(2.5 - inter_top * i);
            }
        }
        float inter_bottom = (-8.50 - (-23.8)) / (32 / cloud_filter_param_.v_factor_ - 1);
        for (int i = 0; i < 32 / cloud_filter_param_.v_factor_; ++i)
        {

            if (i == 31)
            {
                sensor_spec_.v_angle_.push_back(-23.8);
            }
            else
            {
                sensor_spec_.v_angle_.push_back(-8.50 - inter_bottom * i);
            }
        }
    }
    else if (data_type == "CARLA")
    {
        float inter = (2.0 - (-24.8)) / (64);
        for (int i = 0; i < 64 / cloud_filter_param_.v_factor_; ++i)
        {
            if (i == 63)
            {
                sensor_spec_.v_angle_.push_back(-24.8);
            }
            else
            {
                sensor_spec_.v_angle_.push_back(2.0 - inter * i);
            }
        }
    }
};