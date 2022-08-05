#ifndef _IMAGE_FILL_H_
#define _IMAGE_FILL_H_

#include "defines.h"
#include "user_param.h"
#include "cloud_frame.h"
#include <vector>

#include <opencv2/core.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/opencv.hpp>

#include <algorithm>

class MaplessDynamic;
class UserParam;
class CloudFrame;

class ImageFill
{
private:
    int img_height_;
    int img_width_;

public:
    std::vector<int> object_row_;
    std::vector<int> object_col_;
    std::vector<float> object_rho_roi_;

    std::vector<int> filled_object_row_;
    std::vector<int> filled_object_col_;
    std::vector<float> filled_object_rho_roi_;

    std::vector<int> rho_zero_filled_value_row_;
    std::vector<int> rho_zero_filled_value_col_;
    std::vector<float> rho_zero_filled_value_rho_roi_;

    std::vector<float> max_his_filled_object_rho_roi_;

    std::vector<int> disconti_row_;
    std::vector<int> disconti_col_;

    cv::MatND histogram_;

public:
    ImageFill(const std::unique_ptr<UserParam> &user_param);
    ~ImageFill();

    void plugImageZeroHoles(cv::Mat& accumulated_dRdt, cv::Mat& accumulated_dRdt_score, std::unique_ptr<CloudFrame>& CloudFrame_next, cv::Mat& groundPtsIdx_next, int object_threshold);

    void interpAndfill_image(cv::Mat& input_img, cv::Mat& filled_bin);
};

#endif