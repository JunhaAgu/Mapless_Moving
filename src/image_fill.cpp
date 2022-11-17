#include "image_fill.h"

ImageFill::ImageFill(const std::unique_ptr<UserParam>& user_param)
{
    img_height_ = user_param->image_param_.height_;
    img_width_ = user_param->image_param_.width_;
    thr_object_ = user_param->object_param_.thr_object_;

    row_.reserve(100000);
    col_.reserve(100000);

    object_row_.reserve(1000000);
    object_col_.reserve(1000000);
    // object_rho_roi_.reserve(1000000);

    // filled_object_row_.reserve(1000000);
    // filled_object_col_.reserve(1000000);
    filled_object_rho_roi_.reserve(1000000);

    rho_zero_filled_row_.reserve(1000000);
    rho_zero_filled_col_.reserve(1000000);
    rho_zero_filled_rho_roi_.reserve(1000000);

    max_his_filled_object_rho_roi_.reserve(1000000);

    // disconti_row_.reserve(1000000);
    // disconti_col_.reserve(1000000);
};

ImageFill::~ImageFill()
{

};

void ImageFill::plugImageZeroHoles(cv::Mat& accumulated_dRdt, cv::Mat& accumulated_dRdt_score, std::unique_ptr<CloudFrame>& CloudFrame_next, float object_factor)
{
    int n_row = accumulated_dRdt.rows;
    int n_col = accumulated_dRdt.cols;
    float* ptr_accumulated_dRdt         = accumulated_dRdt.ptr<float>(0);
    float* ptr_accumulated_dRdt_score   = accumulated_dRdt_score.ptr<float>(0);
    cv::Mat dRdt_bin        = cv::Mat::zeros(img_height_,img_width_, CV_8UC1);
    cv::Mat dRdt_score_bin  = cv::Mat::zeros(img_height_,img_width_, CV_8UC1);
    uchar* ptr_dRdt_bin         = dRdt_bin.ptr<uchar>(0);
    uchar* ptr_dRdt_score_bin   = dRdt_score_bin.ptr<uchar>(0);
    // padding
    cv::Mat dRdt_bin_pad        = cv::Mat::zeros(img_height_+2,img_width_+2, CV_8UC1);
    cv::Mat dRdt_score_bin_pad  = cv::Mat::zeros(img_height_+2,img_width_+2, CV_8UC1);

    float* ptr_img_rho = CloudFrame_next->str_rhopts_->img_rho.ptr<float>(0);

    cv::Mat rho_zero_value = cv::Mat::zeros(img_height_, img_width_, CV_8UC1);
    uchar *ptr_rho_zero_value = rho_zero_value.ptr<uchar>(0);

    cv::Mat input_img_mask = cv::Mat::zeros(img_height_, img_width_, CV_8UC1);
    uchar *ptr_input_img_mask = input_img_mask.ptr<uchar>(0);

    bool det_func = false;
    // when i=0
    for (int j = 0; j < n_col; ++j)
    {
        if (*(ptr_accumulated_dRdt + j) != 0)
        {
            *(ptr_dRdt_bin + j) = 255;
            det_func = true;
        }

        if (*(ptr_accumulated_dRdt_score + j) != 0)
        {
            *(ptr_dRdt_score_bin + j) = 255;
        }
    }

    for (int i=1; i<n_row; ++i)
    {
        int i_ncols = i * n_col;
        for (int j=0; j<n_col; ++j)
        {   
            const int i_ncols_j = i_ncols + j;         
            if (*(ptr_accumulated_dRdt + i_ncols_j) > 0)
            {
                *(ptr_dRdt_bin + i_ncols_j) = 255;
                *(ptr_input_img_mask + i_ncols_j) = 255;
                det_func = true;
            }
            else{}

            if (*(ptr_accumulated_dRdt_score + i_ncols_j) != 0)
            {
                *(ptr_dRdt_score_bin + i_ncols_j) = 255;
            }
            else{}

            if (*(ptr_img_rho + i_ncols_j) == 0)
            {
                *(ptr_rho_zero_value + i_ncols_j) = 255;
            }
            else{}
        }
    }
    
    if (det_func == false)
    {
        return;
    }

    // imfill 
    //invert dRdt_bin
    cv::Mat dRdt_bin_inv        = cv::Mat::zeros(img_height_, img_width_, CV_8UC1);
    cv::Mat dRdt_bin_inv_pad    = cv::Mat::zeros(img_height_ + 2, img_width_ + 2, CV_8UC1);
    // zero padding
    cv::copyMakeBorder(dRdt_bin, dRdt_bin_pad, 1, 1, 1, 1, cv::BORDER_CONSTANT, cv::Scalar(0));
    
    cv::bitwise_not(dRdt_bin_pad, dRdt_bin_inv_pad);
    cv::floodFill(dRdt_bin_inv_pad, cv::Point(0, 0), cv::Scalar(0));
    cv::Mat dRdt_bin_filled_pad = (dRdt_bin_pad | dRdt_bin_inv_pad);
    // crop
    cv::Mat dRdt_bin_filled = dRdt_bin_filled_pad(cv::Range(0 + 1, img_height_ + 2 - 1), cv::Range(0 + 1, img_width_ + 2 -1));

    interpAndfill_image(accumulated_dRdt, dRdt_bin_filled);

    //invert dRdt_score_bin
    cv::Mat dRdt_score_bin_inv      = cv::Mat::zeros(img_height_, img_width_, CV_8UC1);
    cv::Mat dRdt_score_bin_inv_pad  = cv::Mat::zeros(img_height_ + 2, img_width_ + 2, CV_8UC1);
    // zero padding
    cv::copyMakeBorder(dRdt_score_bin, dRdt_score_bin_pad, 1, 1, 1, 1, cv::BORDER_CONSTANT, cv::Scalar(0));

    cv::bitwise_not(dRdt_score_bin_pad, dRdt_score_bin_inv_pad);
    cv::floodFill(dRdt_score_bin_inv_pad, cv::Point(0, 0), cv::Scalar(0));
    cv::Mat dRdt_score_bin_filled_pad = (dRdt_score_bin_pad | dRdt_score_bin_inv_pad);
    // crop
    cv::Mat dRdt_score_bin_filled = dRdt_score_bin_filled_pad(cv::Range(0 + 1, img_height_ + 2 - 1), cv::Range(0 + 1, img_width_ + 2 - 1));

    interpAndfill_image(accumulated_dRdt_score, dRdt_score_bin_filled);

    // for (int i=1; i<n_row; ++i)
    // {
    //     int i_ncols = i * n_col;
    //     for (int j=0; j<n_col; ++j)
    //     {
    //         const int i_ncols_j = i_ncols + j;
    //         if (*(ptr_img_rho + i_ncols_j) == 0)
    //         {
    //             *(ptr_rho_zero_value + i_ncols_j) = 255;
    //         }

    //         if (*(ptr_accumulated_dRdt + i_ncols_j) > 0)
    //         {
    //             *(ptr_input_img_mask + i_ncols_j) = 255;
    //         }
    //     }
    // }

    // for (int j = 0; j < n_col; ++j)
    // {
    //     *(ptr_rho_zero_value + j) = 0;
    // }

    cv::Mat input_img_tmp = accumulated_dRdt.clone();
    float* ptr_input_img_tmp = input_img_tmp.ptr<float>(0);

    // Label objects
    cv::Mat connect_input = (rho_zero_value | input_img_mask);
    cv::Mat object_label = cv::Mat::zeros(img_height_, img_width_, CV_32SC1);
    cv::Mat stats, centroids;

    int n_label = cv::connectedComponentsWithStats(connect_input, object_label, stats, centroids, 8);
    int* ptr_object_label = object_label.ptr<int>(0);

    int* ptr_object_row         = object_row_.data();
    int* ptr_object_col         = object_col_.data();
    // float* ptr_object_rho_roi   = object_rho_roi_.data();

    cv::Mat zero_candidate      = cv::Mat::zeros(img_height_,img_width_, CV_8UC1);
    uchar *ptr_zero_candidate   = zero_candidate.ptr<uchar>(0);

    // int* ptr_filled_object_row          = filled_object_row_.data();
    // int* ptr_filled_object_col          = filled_object_col_.data();
    float* ptr_filled_object_rho_roi    = filled_object_rho_roi_.data();

    int* ptr_rho_zero_filled_row          = rho_zero_filled_row_.data();
    int* ptr_rho_zero_filled_col          = rho_zero_filled_col_.data();
    float* ptr_rho_zero_filled_rho_roi    = rho_zero_filled_rho_roi_.data();

    cv::Mat object_area             = cv::Mat::zeros(img_height_, img_width_, CV_8UC1);
    cv::Mat object_area_filled      = cv::Mat::zeros(img_height_, img_width_, CV_8UC1);
    cv::Mat filled_object_rho_mat   = cv::Mat::zeros(img_height_, img_width_, CV_32FC1);

    // padding
    cv::Mat object_area_pad             = cv::Mat::zeros(img_height_+2, img_width_+2, CV_8UC1);
    cv::Mat object_area_filled_pad      = cv::Mat::zeros(img_height_+2, img_width_+2, CV_8UC1);

    uchar *ptr_object_area              = object_area.ptr<uchar>(0);
    uchar *ptr_object_area_filled       = object_area_filled.ptr<uchar>(0);
    float *ptr_filled_object_rho_mat    = filled_object_rho_mat.ptr<float>(0);

    cv::Mat object_area_inv     = cv::Mat::zeros(img_height_, img_width_, CV_8UC1);
    uchar *ptr_object_area_inv  = object_area_inv.ptr<uchar>(0);

    cv::Mat object_area_inv_pad = cv::Mat::zeros(img_height_ + 2, img_width_ + 2, CV_8UC1);

    int obj_left = 0;
    int obj_top = 0;
    int obj_width = 0;
    int obj_height = 0;

    // timer::tic();
    for (int object_idx = 0; object_idx < n_label; ++object_idx)
    {
        if (object_idx==0) //0: background
        {
            continue;
        }
        
        object_row_.resize(0);
        object_col_.resize(0);
        // object_rho_roi_.resize(0);
        // filled_object_row_.resize(0);
        // filled_object_col_.resize(0);
        filled_object_rho_roi_.resize(0);
        max_his_filled_object_rho_roi_.resize(0);
        rho_zero_filled_row_.resize(0);
        rho_zero_filled_col_.resize(0);
        rho_zero_filled_rho_roi_.resize(0);
        // disconti_row_.resize(0);
        // disconti_col_.resize(0);
        object_area             = cv::Mat::zeros(img_height_, img_width_, CV_8UC1);
        object_area_filled      = cv::Mat::zeros(img_height_, img_width_, CV_8UC1);
        filled_object_rho_mat   = cv::Mat::zeros(img_height_, img_width_, CV_32FC1);

        object_area_pad             = cv::Mat::zeros(img_height_+2, img_width_+2, CV_8UC1);
        object_area_filled_pad      = cv::Mat::zeros(img_height_+2, img_width_+2, CV_8UC1);

        obj_left    = stats.at<int>(object_idx, cv::CC_STAT_LEFT);
        obj_top     = stats.at<int>(object_idx, cv::CC_STAT_TOP);
        obj_width   = stats.at<int>(object_idx, cv::CC_STAT_WIDTH);
        obj_height  = stats.at<int>(object_idx, cv::CC_STAT_HEIGHT);


        for (int i = obj_top; i < obj_top + obj_height; ++i)
        {
            int i_ncols = i * n_col;
            for (int j = obj_left; j < obj_left + obj_width; ++j)
            {
                if (*(ptr_object_label + i_ncols + j) == object_idx)
                {
                    object_row_.push_back(i);
                    object_col_.push_back(j);
                    // object_rho_roi_.push_back(*(ptr_img_rho + i_ncols + j));
                    *(ptr_object_area + i_ncols + j) = 255;
                }
            }
        }

        // if (object_row_.size() < thr_object_ || *(ptr_object_area + 1*n_col+0) == 255)
        if (object_row_.size() < thr_object_ * object_factor)
        {
            for (int i = 0; i < object_row_.size(); ++i)
            {
                const int ptr_object_row_ncol_col = ptr_object_row[i] * n_col + ptr_object_col[i];
                // *(ptr_rho_zero_value            + ptr_object_row_ncol_col)  = 0;
                *(ptr_accumulated_dRdt          + ptr_object_row_ncol_col)  = 0.0;
                *(ptr_accumulated_dRdt_score    + ptr_object_row_ncol_col)  = 0.0;
            }
            continue;
        }
        else
        {
            float connect_zero_mean = 0.0;
            int n_connet_zero = 0;
            // object_area_inv = cv::Mat::zeros(img_height_, img_width_, CV_8UC1);
            // object_area_inv_pad = cv::Mat::zeros(img_height_+2, img_width_+2, CV_8UC1);            
            // zero padding
            cv::copyMakeBorder(object_area, object_area_pad, 1,1,1,1, cv::BORDER_CONSTANT, cv::Scalar(0));

            cv::bitwise_not(object_area_pad, object_area_inv_pad);
            cv::floodFill(object_area_inv_pad, cv::Point(0, 0), cv::Scalar(0));
            object_area_filled_pad = (object_area_pad | object_area_inv_pad);
            // crop
            object_area_filled = object_area_filled_pad(cv::Range(0+1,img_height_+2-1), cv::Range(0+1,img_width_+2-1));
            for (int i = 0; i < n_row; ++i)
            {
                int i_ncols = i * n_col;
                for (int j = 0; j < n_col; ++j)
                {
                    if (*(ptr_object_area_filled + i_ncols + j) != 0 && *(ptr_input_img_mask + i_ncols + j) != 0)
                    {
                        connect_zero_mean += *(ptr_accumulated_dRdt + i_ncols + j);
                        n_connet_zero += 1;
                    }
                }
            }
            if (n_connet_zero > 0)
            {
                connect_zero_mean /= (float)n_connet_zero;

                for (int i = 0; i < n_row; ++i)
                {
                    int i_ncols = i * n_col;
                    for (int j = 0; j < n_col; ++j)
                    {
                        const int i_ncols_j = i_ncols + j;
                        if (*(ptr_object_area_filled + i_ncols_j) != 0)
                        {
                            if (*(ptr_input_img_mask + i_ncols_j) == 0)
                            {
                                *(ptr_accumulated_dRdt + i_ncols_j) = connect_zero_mean;
                                *(ptr_accumulated_dRdt_score + i_ncols_j) = 1;
                            }

                            const float& img_rho_tmp = *(ptr_img_rho + i_ncols_j);
                            if (img_rho_tmp != 0 && *(ptr_input_img_tmp + i_ncols_j) != 0)
                            {
                                // filled_object_row_.push_back(i);
                                // filled_object_col_.push_back(j);
                                filled_object_rho_roi_.push_back(img_rho_tmp);
                                *(ptr_filled_object_rho_mat + i_ncols_j) = img_rho_tmp;
                            }
                            rho_zero_filled_row_.push_back(i);
                            rho_zero_filled_col_.push_back(j);
                            rho_zero_filled_rho_roi_.push_back(img_rho_tmp);
                        }
                    }
                }
            }
            else
            {
                // continue;
                for (int i = 0; i < n_row; ++i)
                {
                    int i_ncols = i * n_col;
                    for (int j = 0; j < n_col; ++j)
                    {
                        const int i_ncols_j = i_ncols + j;
                        if (*(ptr_object_area_filled + i_ncols_j) != 0)
                        {
                            const float& img_rho_tmp = *(ptr_img_rho + i_ncols_j);
                            if (img_rho_tmp != 0 && *(ptr_input_img_tmp + i_ncols_j) != 0)
                            {
                                // filled_object_row_.push_back(i);
                                // filled_object_col_.push_back(j);
                                filled_object_rho_roi_.push_back(img_rho_tmp);
                                *(ptr_filled_object_rho_mat + i_ncols_j) = img_rho_tmp;
                            }
                            rho_zero_filled_row_.push_back(i);
                            rho_zero_filled_col_.push_back(j);
                            rho_zero_filled_rho_roi_.push_back(img_rho_tmp);
                        }
                    }
                }
            }

            if (filled_object_rho_roi_.size()<2)
            {
                continue;
            }

            if (filled_object_rho_roi_.size()<1)
            {
                for (int i = 0; i < object_row_.size(); ++i)
                {
                    *(ptr_accumulated_dRdt + ptr_object_row[i] * n_col + ptr_object_col[i]) = 0;
                }
                continue;
            }
            else{}

            float his_range_max = *max_element(filled_object_rho_roi_.begin(), filled_object_rho_roi_.end());
            float his_range_min = *min_element(filled_object_rho_roi_.begin(), filled_object_rho_roi_.end());
            float his_range[] = {his_range_min, his_range_max};
            const int* channel_numbers = {0};
            const float* his_ranges= his_range;
            int number_bins = 50;

            cv::calcHist(&filled_object_rho_mat, 1, channel_numbers, cv::Mat(), histogram_, 1, &number_bins, &his_ranges);
            int max_n = 0;
            int max_idx = 100;
            for (int p = 0; p < number_bins; ++p)
            {
                if (max_n < histogram_.at<float>(p))
                {
                    max_n = histogram_.at<float>(p);
                    max_idx = p;
                }
            }
            float his_interval = (his_range_max-his_range_min)/(float)number_bins;
            float bin_range_min = his_range_min + (float)(max_idx)*his_interval;
            float bin_range_max = his_range_min + (float)(max_idx+1)*his_interval;
            float range_min = 0.0;
            float range_max = 0.0;

            // for (int p = 0; p<filled_object_rho_roi_.size(); ++p)
            // {
            //     if (ptr_filled_object_rho_roi[p]>bin_range_min && ptr_filled_object_rho_roi[p]<bin_range_max)
            //     {
            //         max_his_filled_object_rho_roi_.push_back(ptr_filled_object_rho_roi[p]);
            //     }
            // }
            // float max_his_average = 0.0;
            // for (int i=0; i<max_his_filled_object_rho_roi_.size(); ++i)
            // {
            //     max_his_average += max_his_filled_object_rho_roi_[i];
            // }
            // max_his_average = max_his_average/(float)max_his_filled_object_rho_roi_.size();

            if ((bin_range_min - 10.0) < 0.0)
            {
                range_min = 0.0;
            }
            else
            {
                range_min = bin_range_min - 10.0;
            }
            range_max = bin_range_max + 10.0;

            //
            // for (int i = 0; i < n_row; ++i)
            // {
            //     int i_ncols = i * n_col;
            //     for (int j = 0; j < n_col; ++j)
            //     {
            //         if (*(ptr_object_area_filled + i_ncols + j) != 0)
            //         {
            //             rho_zero_filled_row_.push_back(i);
            //             rho_zero_filled_col_.push_back(j);
            //             rho_zero_filled_rho_roi_.push_back(*(ptr_img_rho + i_ncols + j));
            //         }
            //     }
            // }

            for (int i = 0; i < rho_zero_filled_row_.size(); ++i)
            {
                const float& rho_zero_filled_rho_roi_tmp = ptr_rho_zero_filled_rho_roi[i];
                if ((rho_zero_filled_rho_roi_tmp < range_min) || (rho_zero_filled_rho_roi_tmp > range_max))
                {
                    const int idx_tmp = ptr_rho_zero_filled_row[i] * n_col + ptr_rho_zero_filled_col[i];
                    if (*(ptr_object_area_filled + idx_tmp) != 0)
                    {
                        *(ptr_accumulated_dRdt + idx_tmp) = 0.0;
                        *(ptr_accumulated_dRdt_score + idx_tmp) = 0.0;
                    }
                }
            }

            // for (int i = 0; i < rho_zero_filled_row_.size(); ++i)
            // {
            //     float& rho_zero_filled_rho_roi_tmp = ptr_rho_zero_filled_rho_roi[i];
            //     if ((rho_zero_filled_rho_roi_tmp < range_min) || (rho_zero_filled_rho_roi_tmp > range_max))
            //     {
            //         if (*(ptr_object_area_filled + ptr_rho_zero_filled_row[i] * n_col + ptr_rho_zero_filled_col[i]) != 0)
            //         {
            //             disconti_row_.push_back(ptr_rho_zero_filled_row[i]);
            //             disconti_col_.push_back(ptr_rho_zero_filled_col[i]);
            //         }
            //     }
            // }

            // for (int i = 0; i < disconti_row_.size(); ++i)
            // {
            //     const int& disconti_row_tmp = disconti_row_[i];
            //     const int& disconti_col_tmp = disconti_col_[i];

            //     *(ptr_accumulated_dRdt + disconti_row_tmp * n_col + disconti_col_tmp)       = 0.0;
            //     *(ptr_accumulated_dRdt_score + disconti_row_tmp * n_col + disconti_col_tmp) = 0.0;
            // }
        }
    } // end for object_idx
    // double dt_obj = timer::toc(); // milliseconds
    // ROS_INFO_STREAM("elapsed time for 'dt_obj' :" << dt_obj << " [ms]");
}

void ImageFill::interpAndfill_image(cv::Mat& input_img, cv::Mat& filled_bin)
{
    int n_col = input_img.cols;
    int n_row = input_img.rows;

    float* ptr_input_img = input_img.ptr<float>(0);
    uchar* ptr_filled_bin = filled_bin.ptr<uchar>(0);

    cv::Mat input_img_cp = input_img.clone();
    float* ptr_input_img_cp = input_img_cp.ptr<float>(0);

    float left_dir = 0;
    float right_dir = 0;
    float up_dir = 0;
    float down_dir = 0;

    int cnt_left = 1;
    int cnt_right = 1;
    int cnt_up = 1;
    int cnt_down = 1;

    std::vector<float> four_dir(4);

    float min_4_dir = 0.0 ;

    row_.resize(0);
    col_.resize(0);

    for (int i=0; i<n_row; ++i)
    {
        int i_ncols = i * n_col;
        for (int j=0; j<n_col; ++j)
        {
            if ((*(ptr_input_img + i_ncols + j) == 0) && (*(ptr_filled_bin + i_ncols + j) > 0))
            {
                row_.push_back(i);
                col_.push_back(j);
            }
        }
    }

    if (row_.size()==0)
    {
        return;
    }

    for (int k=0; k<row_.size(); ++k)
    {
        int i = row_[k];
        int j = col_[k];

        left_dir = 0.0;
        right_dir = 0.0;
        up_dir = 0.0;
        down_dir = 0.0;

        cnt_left = 1;
        cnt_right = 1;
        cnt_up = 1;
        cnt_down = 1;

        const int i_ncols_j = i * n_col + j;
        // left
        while (left_dir == 0.0)
        {
            if ((j - cnt_left) < 0)
            {
                break;
            }
            left_dir = *(ptr_input_img + i_ncols_j - cnt_left); //*(ptr_input_img + i * n_col + (j - cnt_left));
            cnt_left += 1;
        } // end while
        // right
        while (right_dir == 0.0)
        {
            if ((j + cnt_right) > n_col - 1)
            {
                break;
            }
            right_dir = *(ptr_input_img + i_ncols_j + cnt_right); // *(ptr_input_img + i * n_col + (j + cnt_right));
            cnt_right += 1;
        } // end while
        // up
        while (up_dir == 0.0)
        {
            if ((i - cnt_up) < 0)
            {
                break;
            }
            up_dir = *(ptr_input_img + i_ncols_j - (cnt_up * n_col)); //(i - cnt_up) * n_col + j);
            cnt_up += 1;
        } // end while
        // down
        while (down_dir == 0.0)
        {
            if ((i + cnt_down) > n_row - 1)
            {
                break;
            }
            down_dir = *(ptr_input_img + i_ncols_j + (cnt_down * n_col)); //(i + cnt_down) * n_col + j);
            cnt_down += 1;
        } // end while

        four_dir[0] = (left_dir);
        four_dir[1] = (right_dir);
        four_dir[2] = (up_dir);
        four_dir[3] = (down_dir);
        min_4_dir = *min_element(four_dir.begin(), four_dir.end());
        *(ptr_input_img_cp + i_ncols_j) = min_4_dir;
    }
    input_img_cp.copyTo(input_img);
}