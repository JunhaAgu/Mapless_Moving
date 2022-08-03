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

class UserParam
{
    friend class MaplessDynamic;
    friend class CloudFrame;
    friend class SegmentGround;

    private:

    public:
        CloudFilterParam cloud_filter_param_;
        SensorSpec sensor_spec_;
        RansacParam ransac_param_;
        GroundSegmentParam ground_segment_param_;

    public:
        UserParam();
        ~UserParam();
        void getUserSettingParameters();
        void calVangle(std::string& data_type);
};
#endif