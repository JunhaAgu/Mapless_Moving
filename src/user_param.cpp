#include "user_param.h"
#include <string>

UserParam::UserParam()
{
    // this->getUserSettingParameters();
};

UserParam::~UserParam()
{
    // destructor
};

void UserParam::getUserSettingParameters(std::string& data_type)
{
    this->dataset_name_ = data_type;

    cloud_filter_param_.h_factor_ = 5;
    cloud_filter_param_.v_factor_ = 1;
    cloud_filter_param_.azimuth_res_ = 0.08;

    this->calVangle(data_type);
    
    ground_segment_param_.downsample_size = 10;
    ransac_param_.iter_ = 30;
    ransac_param_.thr_ = 0.1;
    ransac_param_.a_thr_ = 0.1;
    ransac_param_.b_thr_[0] = -0.5;
    ransac_param_.b_thr_[1] = -1.2;
    ransac_param_.min_inlier_ = 5;
    ransac_param_.n_sample_ = 2; // line y=ax+b

    sensor_spec_.channel_ = 64;

    image_param_.height_    = sensor_spec_.channel_ / cloud_filter_param_.v_factor_ ;
    image_param_.width_     = 360.0 / (cloud_filter_param_.azimuth_res_ * cloud_filter_param_.h_factor_) + 1;

    object_param_.thr_object_ = 40;
    object_param_.alpha_ = 0.3;
    object_param_.beta_ = 0.1;
    object_param_.coef_accum_w_[0] = 0.5;
    object_param_.coef_accum_w_[1] = 0.9;
    
    segment_param_.weight_factor_ = 0.95;
    segment_param_.seg_deg_ = 10; //degree
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
        sensor_spec_.lidar_elevation_criteria_[0] = 2.5; //0.043633231299858; //2.5;
        sensor_spec_.lidar_elevation_criteria_[1] = -8.0; //-0.139626340159546; //-8.0;
        sensor_spec_.lidar_elevation_criteria_[2] = -8.5; //-0.148352986419518; //-8.5;
        sensor_spec_.lidar_elevation_criteria_[3] = -23.8; //-0.415388361974650; //-23.8;
        sensor_spec_.lidar_elevation_line0_[0] = -2.9523810; //-1.691589680862436e+02; //-2.9523810; //(0-31)/(2.5-(-8.0))*(x-2.5)+0
        sensor_spec_.lidar_elevation_line0_[1] = +7.3809524;  //+7.380952380952365; //+7.3809524;  //(0-31)/(2.5-(-8.0))*(x-2.5)+0
        sensor_spec_.lidar_elevation_line1_[0] = -2.0261438; //-1.160894879023238e+02; //-2.0261438;  //(32-63)/(-8.5-(-23.8))*(x-(-8.5))+32
        sensor_spec_.lidar_elevation_line1_[1] = +14.7777778; //+14.777777777777754; //+14.7777778; //(32-63)/(-8.5-(-23.8))*(x-(-8.5))+32
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
        sensor_spec_.lidar_elevation_criteria_[0] = 2.0;
        sensor_spec_.lidar_elevation_criteria_[1] = -11.1873016;
        sensor_spec_.lidar_elevation_criteria_[2] = -11.6126984;
        sensor_spec_.lidar_elevation_criteria_[3] = -24.8;
        sensor_spec_.lidar_elevation_line0_[0] = -2.3507463;    //(0-63)/(+2-(-24.8))*(x-2)+0
        sensor_spec_.lidar_elevation_line0_[1] = +4.7014925;    //(0-63)/(+2-(-24.8))*(x-2)+0
        sensor_spec_.lidar_elevation_line1_[0] = -2.3507463;    //(0-63)/(+2-(-24.8))*(x-2)+0
        sensor_spec_.lidar_elevation_line1_[1] = +4.7014925;    //(0-63)/(+2-(-24.8))*(x-2)+0
    }
};