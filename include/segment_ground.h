#ifndef _SEGMENT_GROUND_H_
#define _SEGMENT_GROUND_H_

#include <numeric>
#include <random>
#include <string>
#include <vector>

#include "cloud_frame.h"
#include "defines.h"
#include "user_param.h"

class MaplessDynamic;
class UserParam;
class CloudFrame;

class SegmentGround {
   private:
    int downsample_size_;
    int iter_;
    float thr_;
    float a_thr_;
    float b_thr_[2];
    int mini_inlier_;
    int n_sample_;

    int img_height_;
    int img_width_;

    std::string dataset_name_;

   public:
    cv::Mat groundPtsIdx_next_;
    std::random_device rd;
    std::mt19937 gen_;

   public:
    SegmentGround(const std::unique_ptr<UserParam>& user_param);
    ~SegmentGround();

    void fastsegmentGround(std::unique_ptr<CloudFrame>& CloudFrame_in);

    void ransacLine(std::vector<float>& points_rho,
                    std::vector<float>& points_z,
                    /*output*/ bool mask_inlier[], std::vector<float>& line_a,
                    std::vector<float>& line_b, int num_seg);

    void reset();
};

#endif