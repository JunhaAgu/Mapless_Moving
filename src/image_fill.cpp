#include "image_fill.h"

ImageFill::ImageFill(const std::unique_ptr<UserParam>& user_param)
{
    img_height_ = user_param->image_param_.height_;
    img_width_ = user_param->image_param_.width_;
    object_threshold_ = user_param->obejct_param_.thr_object_;

    object_row_.reserve(1000000);
    object_col_.reserve(1000000);
    object_rho_roi_.reserve(1000000);

    filled_object_row_.reserve(1000000);
    filled_object_col_.reserve(1000000);
    filled_object_rho_roi_.reserve(1000000);

    rho_zero_filled_value_row_.reserve(1000000);
    rho_zero_filled_value_col_.reserve(1000000);
    rho_zero_filled_value_rho_roi_.reserve(1000000);

    max_his_filled_object_rho_roi_.reserve(1000000);

    disconti_row_.reserve(1000000);
    disconti_col_.reserve(1000000);
};

ImageFill::~ImageFill()
{

};

void ImageFill::plugImageZeroHoles(cv::Mat& accumulated_dRdt, cv::Mat& accumulated_dRdt_score, std::unique_ptr<CloudFrame>& CloudFrame_next)
{
    int n_row = accumulated_dRdt.rows;
    int n_col = accumulated_dRdt.cols;
    float* ptr_accumulated_dRdt         = accumulated_dRdt.ptr<float>(0);
    float* ptr_accumulated_dRdt_score   = accumulated_dRdt_score.ptr<float>(0);
    cv::Mat dRdt_bin        = cv::Mat::zeros(img_height_,img_width_, CV_8UC1);
    cv::Mat dRdt_score_bin  = cv::Mat::zeros(img_height_,img_width_, CV_8UC1);
    uchar* ptr_dRdt_bin         = dRdt_bin.ptr<uchar>(0);
    uchar* ptr_dRdt_score_bin   = dRdt_score_bin.ptr<uchar>(0);

    float* ptr_img_rho = CloudFrame_next->str_rhopts_->img_rho.ptr<float>(0);

    for (int i=0; i<n_row; ++i)
    {
        int i_ncols = i * n_col;
        for (int j=0; j<n_col; ++j)
        {            
            if (*(ptr_accumulated_dRdt + i_ncols + j) != 0)
            {
                *(ptr_dRdt_bin + i_ncols + j) = 255;
            }
        }
    }

    for (int i = 0; i < n_row; ++i)
    {
        int i_ncols = i * n_col;
        for (int j = 0; j < n_col; ++j)
        {
            if (*(ptr_accumulated_dRdt_score + i_ncols + j) != 0)
            {
                *(ptr_dRdt_score_bin + i_ncols + j) = 255;
            }
        }
    }


    // imfill 
    //invert dRdt_bin
    cv::Mat dRdt_bin_inv = cv::Mat::zeros(img_height_, img_width_, CV_8UC1);
    cv::bitwise_not(dRdt_bin, dRdt_bin_inv);
    uchar *ptr_dRdt_bin_inv = dRdt_bin_inv.ptr<uchar>(0);
    int n_row_minus1_n_col = (n_row - 1) * n_col;
    for (int j = 0; j < n_col; ++j)
    {
        *(ptr_dRdt_bin_inv + n_row_minus1_n_col + j) = 255;
    }
    cv::floodFill(dRdt_bin_inv, cv::Point(0,0), cv::Scalar(0));
    cv::Mat dRdt_bin_filled = (dRdt_bin | dRdt_bin_inv);
    interpAndfill_image(accumulated_dRdt, dRdt_bin_filled);

    cv::Mat dRdt_score_bin_inv = cv::Mat::zeros(img_height_, img_width_, CV_8UC1);
    cv::bitwise_not(dRdt_score_bin, dRdt_score_bin_inv);
    uchar *ptr_dRdt_score_bin_inv = dRdt_score_bin_inv.ptr<uchar>(0);
    for (int j = 0; j < n_col; ++j)
    {
        *(ptr_dRdt_score_bin_inv + n_row_minus1_n_col + j) = 255;
    }
    cv::floodFill(dRdt_score_bin_inv, cv::Point(0,0), cv::Scalar(0));
    cv::Mat dRdt_score_bin_filled = (dRdt_score_bin | dRdt_score_bin_inv);
    interpAndfill_image(accumulated_dRdt_score, dRdt_score_bin_filled);

    // cv::imshow("accumulated_dRdt", dRdt_bin_filled);
    // cv::waitKey(0);
    // exit(0);

    cv::Mat rho_zero_value = cv::Mat::zeros(img_height_, img_width_, CV_8UC1);
    uchar* ptr_rho_zero_value = rho_zero_value.ptr<uchar>(0);

    for (int i=1; i<n_row; ++i)
    {
        int i_ncols = i * n_col;
        for (int j=0; j<n_col; ++j)
        {
            if (*(ptr_img_rho + i_ncols + j) == 0)
            {
                *(ptr_rho_zero_value + i_ncols + j) = 255;
            }
        }
    }

    cv::Mat input_img_mask = cv::Mat::zeros(img_height_, img_width_, CV_8UC1);
    uchar* ptr_input_img_mask = input_img_mask.ptr<uchar>(0);

    for (int i=0; i<n_row; ++i)
    {
        int i_ncols = i * n_col;
        for (int j=0; j<n_col; ++j)
        {
            if (*(ptr_accumulated_dRdt + i_ncols + j) > 0)
            {
                *(ptr_input_img_mask + i_ncols + j) = 255;
            }
        }
    }

    cv::Mat input_img_tmp = accumulated_dRdt.clone();
    float* ptr_input_img_tmp = input_img_tmp.ptr<float>(0);

    // Label objects
    cv::Mat connect_input = (rho_zero_value | input_img_mask);
    cv::Mat object_label = cv::Mat::zeros(img_height_, img_width_, CV_32SC1);
    cv::Mat stats, centroids;

    int n_label = cv::connectedComponentsWithStats(connect_input, object_label, stats, centroids, 8);
    int* ptr_object_label = object_label.ptr<int>(0);

    int* ptr_object_row = object_row_.data();
    int* ptr_object_col = object_col_.data();
    float* ptr_object_rho_roi = object_rho_roi_.data();

    cv::Mat zero_candidate = cv::Mat::zeros(img_height_,img_width_, CV_8UC1);
    uchar *ptr_zero_candidate = zero_candidate.ptr<uchar>(0);

    int* ptr_filled_object_row = filled_object_row_.data();
    int* ptr_filled_object_col = filled_object_col_.data();
    float* ptr_filled_object_rho_roi = filled_object_rho_roi_.data();

    int* ptr_rho_zero_filled_value_row = rho_zero_filled_value_row_.data();
    int* ptr_rho_zero_filled_value_col = rho_zero_filled_value_col_.data();
    float* ptr_rho_zero_filled_value_rho_roi = rho_zero_filled_value_rho_roi_.data();

    cv::Mat object_area = cv::Mat::zeros(img_height_, img_width_, CV_8UC1);
    uchar *ptr_object_area = object_area.ptr<uchar>(0);
    cv::Mat object_area_filled = cv::Mat::zeros(img_height_, img_width_, CV_8UC1);
    uchar *ptr_object_area_filled = object_area_filled.ptr<uchar>(0);
    cv::Mat filled_object_rho_mat = cv::Mat::zeros(img_height_, img_width_, CV_32FC1);
    float *ptr_filled_object_rho_mat = filled_object_rho_mat.ptr<float>(0);

    cv::Mat object_area_inv = cv::Mat::zeros(img_height_, img_width_, CV_8UC1);
    uchar *ptr_object_area_inv = object_area_inv.ptr<uchar>(0);
    
    for (int object_idx = 0; object_idx < n_label; ++object_idx)
    {
    
        if (object_idx==0) //0: background
        {
            continue;
        }

        object_row_.resize(0);
        object_col_.resize(0);
        object_rho_roi_.resize(0);
        filled_object_row_.resize(0);
        filled_object_col_.resize(0);
        filled_object_rho_roi_.resize(0);
        max_his_filled_object_rho_roi_.resize(0);
        rho_zero_filled_value_row_.resize(0);
        rho_zero_filled_value_col_.resize(0);
        rho_zero_filled_value_rho_roi_.resize(0);
        disconti_row_.resize(0);
        disconti_col_.resize(0);
        object_area             = cv::Mat::zeros(img_height_, img_width_, CV_8UC1);
        object_area_filled      = cv::Mat::zeros(img_height_, img_width_, CV_8UC1);
        filled_object_rho_mat   = cv::Mat::zeros(img_height_, img_width_, CV_32FC1);

        int left = stats.at<int>(object_idx, cv::CC_STAT_LEFT);
        int top = stats.at<int>(object_idx, cv::CC_STAT_TOP);
        int width = stats.at<int>(object_idx, cv::CC_STAT_WIDTH);
        int height = stats.at<int>(object_idx, cv::CC_STAT_HEIGHT);

        for (int i = top; i < top+height; ++i)
        {
            int i_ncols = i * n_col;
            for (int j = left; j < left+width; ++j)
            {
                if (*(ptr_object_label + i_ncols + j) == object_idx)
                {
                    object_row_.push_back(i);
                    object_col_.push_back(j);
                    object_rho_roi_.push_back(*(ptr_img_rho + i_ncols + j));
                    *(ptr_object_area + i_ncols + j) = 255;
                }
            }
        }

        if (object_row_.size() < object_threshold_)
        {
            for (int i = 0; i < object_row_.size(); ++i)
            {
                *(ptr_rho_zero_value + ptr_object_row[i] * n_col + ptr_object_col[i]) = 0;
                *(ptr_accumulated_dRdt + ptr_object_row[i] * n_col + ptr_object_col[i]) = 0.0;
            }
            continue;
        }
        else
        {
            float connect_zero_mean = 0.0;
            int n_connet_zero = 0;

            // cv::Mat object_area_inv = cv::Mat::zeros(img_height_, img_width_, CV_8UC1);
            cv::bitwise_not(object_area, object_area_inv);

            for (int j = 0; j < n_col; ++j)
            {
                *(ptr_object_area_inv + (img_height_-1) * n_col + j) = 255;
            }
            cv::floodFill(object_area_inv, cv::Point(0, 0), cv::Scalar(0));
            object_area_filled = (object_area | object_area_inv);

            for (int i=0; i<n_row; ++i)
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
                        if (*(ptr_object_area_filled + i_ncols + j) != 0 && *(ptr_input_img_mask + i_ncols + j) == 0)
                        {
                            *(ptr_accumulated_dRdt + i_ncols + j) = connect_zero_mean;
                        }
                    }
                }
            }
            else
            {
                // continue;
            }

            for (int i = 0; i < n_row; ++i)
            {
                int i_ncols = i * n_col;
                for (int j = 0; j < n_col; ++j)
                {
                    if (*(ptr_img_rho + i_ncols + j) != 0 && *(ptr_input_img_tmp + i_ncols + j) != 0 && *(ptr_object_area_filled + i_ncols + j) != 0)
                    {
                        filled_object_row_.push_back(i);
                        filled_object_col_.push_back(j);
                        filled_object_rho_roi_.push_back(*(ptr_img_rho + i_ncols + j));
                        *(ptr_filled_object_rho_mat + i_ncols + j) = *(ptr_img_rho + i_ncols + j);
                    }
                    else{}
                }
            }

            if (filled_object_rho_roi_.size()<2)
            {
                continue;
            }

            if (filled_object_row_.size()<1)
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

            for (int p = 0; p<filled_object_rho_roi_.size(); ++p)
            {
                if (ptr_filled_object_rho_roi[p]>bin_range_min && ptr_filled_object_rho_roi[p]<bin_range_max)
                {
                    max_his_filled_object_rho_roi_.push_back(ptr_filled_object_rho_roi[p]);
                }
            }
            float max_his_average = 0.0;
            for (int i=0; i<max_his_filled_object_rho_roi_.size(); ++i)
            {
                max_his_average += max_his_filled_object_rho_roi_[i];
            }
            max_his_average = max_his_average/(float)max_his_filled_object_rho_roi_.size();

            if ((max_his_average - 10.0) < 0.0)
            {
                range_min = 0.0;
            }
            else{
                range_min = max_his_average - 10.0;
            }
            range_max = max_his_average + 10.0;

            //
            for (int i = 0; i < n_row; ++i)
            {
                int i_ncols = i * n_col;
                for (int j = 0; j < n_col; ++j)
                {
                    if (*(ptr_object_area_filled + i_ncols + j) != 0 && *(ptr_img_rho + i_ncols + j) != 0)
                    {
                        rho_zero_filled_value_row_.push_back(i);
                        rho_zero_filled_value_col_.push_back(j);
                        rho_zero_filled_value_rho_roi_.push_back(*(ptr_img_rho + i_ncols + j));
                    }
                }
            }

            for (int i = 0; i < rho_zero_filled_value_row_.size(); ++i)
            {
                if ((ptr_rho_zero_filled_value_rho_roi[i] < range_min) || (ptr_rho_zero_filled_value_rho_roi[i] > range_max))
                {
                    if (*(ptr_object_area_filled + ptr_rho_zero_filled_value_row[i] * n_col + ptr_rho_zero_filled_value_col[i]) != 0)
                    {
                        disconti_row_.push_back(ptr_rho_zero_filled_value_row[i]);
                        disconti_col_.push_back(ptr_rho_zero_filled_value_col[i]);
                    }
                }
            }

            for (int i = 0; i < disconti_row_.size(); ++i)
            {
                *(ptr_accumulated_dRdt + disconti_row_[i] * n_col + disconti_col_[i]) = 0;
            }

        }
    } // end for object_idx
}

void ImageFill::interpAndfill_image(cv::Mat& input_img, cv::Mat& filled_bin)
{
    int n_col = input_img.cols;
    int n_row = input_img.rows;
    std::vector<int> row;
    std::vector<int> col;
    row.reserve(100000);
    col.reserve(100000);

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

    float min = 0.0 ;

    for (int i=0; i<n_row; ++i)
    {
        int i_ncols = i * n_col;
        for (int j=0; j<n_col; ++j)
        {
            if ((*(ptr_input_img + i_ncols + j) == 0) && (*(ptr_filled_bin + i_ncols + j) > 0))
            {
                row.push_back(i);
                col.push_back(j);
            }
        }
    }

    if (row.size()==0)
    {
        return;
    }

    for (int k=0; k<row.size(); ++k)
    {
        int i = row[k];
        int j = col[k];

        left_dir = 0.0;
        right_dir = 0.0;
        up_dir = 0.0;
        down_dir = 0.0;

        cnt_left = 1;
        cnt_right = 1;
        cnt_up = 1;
        cnt_down = 1;
        // left
        while (left_dir == 0.0)
        {
            if ((j - cnt_left) < 0)
            {
                break;
            }
            left_dir = *(ptr_input_img + i * n_col + (j - cnt_left));
            cnt_left += 1;
        } // end while
        // right
        while (right_dir == 0.0)
        {
            if ((j + cnt_right) > n_col - 1)
            {
                break;
            }
            right_dir = *(ptr_input_img + i * n_col + (j + cnt_right));
            cnt_right += 1;
        } // end while
        // up
        while (up_dir == 0.0)
        {
            if ((i - cnt_up) < 0)
            {
                break;
            }
            up_dir = *(ptr_input_img + (i - cnt_up) * n_col + j);
            cnt_up += 1;
        } // end while
        // down
        while (down_dir == 0.0)
        {
            if ((i + cnt_down) > n_row - 1)
            {
                break;
            }
            down_dir = *(ptr_input_img + (i + cnt_down) * n_col + j);
            cnt_down += 1;
        } // end while
        four_dir[0] = (left_dir);
        four_dir[1] = (right_dir);
        four_dir[2] = (up_dir);
        four_dir[3] = (down_dir);
        min = *min_element(four_dir.begin(), four_dir.end());
        *(ptr_input_img_cp + i * n_col + j) = min;
    }
    input_img_cp.copyTo(input_img);
}