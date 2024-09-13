#ifndef _USER_PARAM_H_
#define _USER_PARAM_H_

#include <vector>
#include <string>

struct CloudFilterParam
{
    int h_factor_;
    int v_factor_;
    float azimuth_res_;
};

struct SensorSpec
{
    std::vector<float> v_angle_;
    int channel_;
    // 2.5 -8 -8.5 -23.8
    float lidar_elevation_criteria_[4];
    
    float lidar_elevation_line0_[2];
    float lidar_elevation_line1_[2];
};

struct GroundSegmentParam
{
    int downsample_size; 
};

struct RansacParam
{
    int iter_;
    float thr_;
    float a_thr_;
    float b_thr_[2];
    int min_inlier_;
    int n_sample_;
};

struct ImageParam
{
    int height_;
    int width_;
};

struct ObjectParam
{
    int thr_object_;
    float alpha_;
    float beta_;
    float coef_accum_w_[2];
};

struct SegmentParam
{
    float weight_factor_;
    float seg_deg_;
};

class UserParam
{
    friend class MaplessDynamic;
    friend class CloudFrame;
    friend class SegmentGround;

    private:

    public:
        std::string dataset_name_; //from roslaunch --> ros_wrapper --> mapless_moving
        CloudFilterParam cloud_filter_param_;
        SensorSpec sensor_spec_;
        RansacParam ransac_param_;
        GroundSegmentParam ground_segment_param_;
        ImageParam image_param_;
        ObjectParam object_param_;
        SegmentParam segment_param_;

    public:
        UserParam();
        ~UserParam();
        void getUserSettingParameters(std::string& data_type);
        void calVangle(std::string& data_type);
};
#endif