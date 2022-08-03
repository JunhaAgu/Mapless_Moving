#ifndef _SEGMENT_GROUND_H_
#define _SEGMENT_GROUND_H_

#include "defines.h"
#include "user_param.h"

#include <vector>
#include <string>
#include <numeric>
#include <random>

class MaplessDynamic;
class UserParam;

class SegmentGround
{
    private:
        int downsample_size_;
        int iter_;
        float thr_;
        float a_thr_;
        float b_thr_[2];
        int mini_inlier_;
        int n_sample_;
        
    public:
        SegmentGround(const std::unique_ptr<UserParam>& user_param);
        ~SegmentGround();

        void fastsegmentGround(StrRhoPts* str_in, cv::Mat& groundPtsIdx_next);

        void ransacLine(std::vector<float>& points_rho, std::vector<float>& points_z, /*output*/ bool mask_inlier[], int num_seg);
};

#endif