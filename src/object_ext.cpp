#include "object_ext.h"

ObjectExt::ObjectExt(const std::unique_ptr<UserParam>& user_param) {
    img_height_ = user_param->image_param_.height_;
    img_width_ = user_param->image_param_.width_;

    thr_object_ = user_param->object_param_.thr_object_;
    alpha_ = user_param->object_param_.alpha_;
    beta_ = user_param->object_param_.beta_;
    coef_accum_w_[0] = user_param->object_param_.coef_accum_w_[0];
    coef_accum_w_[1] = user_param->object_param_.coef_accum_w_[1];

    weight_factor_ = user_param->segment_param_.weight_factor_;
    seg_deg_ = user_param->segment_param_.seg_deg_;

    idx_row_.reserve(500000);
    idx_col_.reserve(500000);
    check_.reserve(500000);

    object_row_.reserve(500000);
    object_col_.reserve(500000);
    object_rho_roi_.reserve(500000);

    // max_his_object_rho_roi_.reserve(500000);

    disconti_row_.reserve(500000);
    disconti_col_.reserve(500000);

    diff_object_area_conti_row_.reserve(500000);
    diff_object_area_conti_col_.reserve(500000);

    diff_z_.reserve(500000);
};

ObjectExt::~ObjectExt(){

};

void ObjectExt::filterOutAccumdR(
    std::unique_ptr<CloudFrame>& CloudFrame_next,
    std::unique_ptr<CloudFrame>& CloudFrame_cur_warped,
    cv::Mat& accumulated_dRdt, cv::Mat& accumulated_dRdt_score, cv::Mat& dRdt) {
    int n_row = CloudFrame_next->str_rhopts_->img_rho.rows;
    int n_col = CloudFrame_next->str_rhopts_->img_rho.cols;

    float* ptr_next_img_rho =
        CloudFrame_next->str_rhopts_->img_rho.ptr<float>(0);
    float* ptr_cur_warped_img_rho =
        CloudFrame_cur_warped->str_rhopts_->img_rho.ptr<float>(0);
    float* ptr_accumulated_dRdt = accumulated_dRdt.ptr<float>(0);
    float* ptr_dRdt = dRdt.ptr<float>(0);

    // Accumulate the occlusion
    for (int i = 0; i < n_row; ++i) {
        int i_ncols = i * n_col;
        for (int j = 0; j < n_col; ++j) {
            if (*(ptr_next_img_rho + i_ncols + j) < 10.0) {
                *(ptr_accumulated_dRdt + i_ncols + j) =
                    coef_accum_w_[0] * (*(ptr_accumulated_dRdt + i_ncols + j)) +
                    *(ptr_dRdt + i_ncols + j);
            } else  // >10
            {
                *(ptr_accumulated_dRdt + i_ncols + j) =
                    coef_accum_w_[1] * (*(ptr_accumulated_dRdt + i_ncols + j)) +
                    *(ptr_dRdt + i_ncols + j);
            }
        }
    }

    // Extract candidate for objects
    for (int i = 0; i < n_row; ++i) {
        int i_ncols = i * n_col;
        for (int j = 0; j < n_col; ++j) {
            if ((*(ptr_next_img_rho + i_ncols + j) > 40) ||
                (*(ptr_accumulated_dRdt + i_ncols + j) <
                 alpha_ * (*(ptr_next_img_rho + i_ncols + j))) ||
                (*(ptr_next_img_rho + i_ncols + j) == 0) ||
                (*(ptr_cur_warped_img_rho + i_ncols + j) == 0) ||
                (*(ptr_dRdt + i_ncols + j) <
                 (-beta_ * (*(ptr_next_img_rho + i_ncols + j))))) {
                *(ptr_accumulated_dRdt + i_ncols + j) = 0;
            }
        }
    }
}

void ObjectExt::extractObjectCandidate(
    cv::Mat& accumulated_dRdt, std::unique_ptr<CloudFrame>& CloudFrame_next,
    float object_factor) {
    int n_col = accumulated_dRdt.cols;
    int n_row = accumulated_dRdt.rows;

    cv::Mat object_mask = cv::Mat::zeros(img_height_, img_width_, CV_8UC1);
    uchar* ptr_object_mask = object_mask.ptr<uchar>(0);

    float* ptr_accumulated_dRdt = accumulated_dRdt.ptr<float>(0);
    float* ptr_img_z = CloudFrame_next->str_rhopts_->img_z.ptr<float>(0);
    float* ptr_img_rho = CloudFrame_next->str_rhopts_->img_rho.ptr<float>(0);

    cv::Mat object_label = cv::Mat::zeros(img_height_, img_width_, CV_32SC1);
    int* ptr_object_label = object_label.ptr<int>(0);

    for (int i = 0; i < n_row; ++i) {
        int i_ncols = i * n_col;
        for (int j = 0; j < n_col; ++j) {
            if (*(ptr_accumulated_dRdt + i_ncols + j) > 0) {
                *(ptr_object_mask + i_ncols + j) = 255;
            }
        }
    }

    // Label objects in 2D image
    cv::Mat stats, centroids;
    int n_label = cv::connectedComponentsWithStats(object_mask, object_label,
                                                   stats, centroids, 8);

    if (n_label == 0) {
        return;
    }

    for (int object_idx = 0; object_idx < n_label; ++object_idx) {
        if (object_idx == 0)  // background
        {
            continue;
        }
        object_row_.resize(0);
        object_col_.resize(0);
        object_rho_roi_.resize(0);
        // max_his_object_rho_roi_.resize(0);
        disconti_row_.resize(0);
        disconti_col_.resize(0);
        diff_object_area_conti_row_.resize(0);
        diff_object_area_conti_col_.resize(0);
        diff_z_.resize(0);
        cv::Mat object_rho_mat =
            cv::Mat::zeros(img_height_, img_width_, CV_32FC1);
        float* ptr_object_rho_mat = object_rho_mat.ptr<float>(0);

        int obj_left = stats.at<int>(object_idx, cv::CC_STAT_LEFT);
        int obj_top = stats.at<int>(object_idx, cv::CC_STAT_TOP);
        int obj_width = stats.at<int>(object_idx, cv::CC_STAT_WIDTH);
        int obj_height = stats.at<int>(object_idx, cv::CC_STAT_HEIGHT);

        for (int i = obj_top; i < obj_top + obj_height; ++i) {
            int i_ncols = i * n_col;
            for (int j = obj_left; j < obj_left + obj_width; ++j) {
                if (*(ptr_object_label + i_ncols + j) == object_idx) {
                    object_row_.push_back(i);
                    object_col_.push_back(j);
                    object_rho_roi_.push_back(*(ptr_img_rho + i_ncols + j));
                    *(ptr_object_rho_mat + i_ncols + j) =
                        *(ptr_img_rho + i_ncols + j);  // for calcHist
                }
            }
        }

        if (object_row_.size() < thr_object_ * object_factor) {
            for (int i = 0; i < object_row_.size(); ++i) {
                *(ptr_accumulated_dRdt + object_row_[i] * n_col +
                  object_col_[i]) = 0.0;
            }
            continue;
        } else {
            float his_range_max =
                *max_element(object_rho_roi_.begin(), object_rho_roi_.end());
            float his_range_min =
                *min_element(object_rho_roi_.begin(), object_rho_roi_.end());

            float his_range[] = {his_range_min, his_range_max};
            const int* channel_numbers = {0};
            const float* his_ranges = his_range;
            int number_bins = 50;
            // std::cout<<object_rho_roi_.size()<<std::endl;
            // std::cout<<his_range[0]<<std::endl;
            // std::cout<<his_range[1]<<std::endl;
            cv::calcHist(&object_rho_mat, 1, channel_numbers, cv::Mat(),
                         histogram_, 1, &number_bins, &his_ranges);

            int max_n = 0;
            int max_idx = 100;
            for (int p = 0; p < number_bins; ++p) {
                if (max_n <= histogram_.at<float>(p)) {
                    max_n = histogram_.at<float>(p);
                    // std::cout << "p: " <<p << " " << "max_n: " << max_n<<
                    // std::endl;
                    max_idx = p;
                }
            }
            // his_range_max = ceil(his_range_max * 100) / 100;
            // his_range_min = floor(his_range_min * 100) / 100;
            float his_interval =
                (his_range_max - his_range_min) / (float)number_bins;
            // his_interval = ceil(his_interval*1000) / 1000;
            float bin_range_min = his_range_min + (float)(max_idx)*his_interval;
            float bin_range_max =
                his_range_min + (float)(max_idx + 1) * his_interval;
            float range_min = 0.0;
            float range_max = 0.0;
            // float max_his_average = 0.0;

            // for (int p = 0; p<object_rho_roi_.size(); ++p)
            // {
            //     if (object_rho_roi_[p]>bin_range_min &&
            //     object_rho_roi_[p]<bin_range_max)
            //     {
            //         max_his_object_rho_roi_.push_back(object_rho_roi_[p]);
            //     }
            // }
            // for (int i=0; i<max_his_object_rho_roi_.size(); ++i)
            // {
            //     max_his_average += max_his_object_rho_roi_[i];
            // }
            // max_his_average =
            // max_his_average/(float)max_his_object_rho_roi_.size();

            if ((bin_range_min - 1.0) < 0.0) {
                range_min = bin_range_min;
            } else {
                range_min = bin_range_min - 0.8;
            }
            range_max = bin_range_max + 1.0;

            // std::cout << "#: "<<object_row_.size() << " " <<"range_min: "
            // <<range_min << " " << "range_max: "<<range_max <<std::endl;

            for (int i = 0; i < object_row_.size(); ++i) {
                if ((object_rho_roi_[i] < range_min) ||
                    (object_rho_roi_[i] > range_max)) {
                    disconti_row_.push_back(object_row_[i]);
                    disconti_col_.push_back(object_col_[i]);
                } else {
                    diff_object_area_conti_row_.push_back(object_row_[i]);
                    diff_object_area_conti_col_.push_back(object_col_[i]);
                }
            }
            // std::cout << "object_row_.size(): " << object_row_.size() << " "
            // << "disconti_row_.size(): " << disconti_row_.size() <<std::endl;
            if ((object_row_.size() - disconti_row_.size()) <
                thr_object_ * object_factor) {
                for (int i = 0; i < object_row_.size(); ++i) {
                    *(ptr_accumulated_dRdt + object_row_[i] * n_col +
                      object_col_[i]) = 0;
                }
                continue;
            } else {
                for (int i = 0; i < disconti_row_.size(); ++i) {
                    *(ptr_accumulated_dRdt + disconti_row_[i] * n_col +
                      disconti_col_[i]) = 0;
                }
            }

            for (int i = 0; i < diff_object_area_conti_row_.size(); ++i) {
                if (*(ptr_img_z + diff_object_area_conti_row_[i] * n_col +
                      diff_object_area_conti_col_[i]) != 0) {
                    diff_z_.push_back(*(ptr_img_z +
                                        diff_object_area_conti_row_[i] * n_col +
                                        diff_object_area_conti_col_[i]));
                }
            }

            float mean_diff_z = 0.0;
            for (int i = 0; i < diff_z_.size(); ++i) {
                mean_diff_z += diff_z_[i];
            }
            mean_diff_z = mean_diff_z / (float)diff_z_.size();

            float std_diff_z = 0.0;
            for (int i = 0; i < diff_z_.size(); ++i) {
                std_diff_z +=
                    (diff_z_[i] - mean_diff_z) * (diff_z_[i] - mean_diff_z);
            }
            std_diff_z = sqrt(1.0 / ((float)diff_z_.size() - 1.0) * std_diff_z);
            if (std_diff_z < 0.07) {
                for (int i = 0; i < object_row_.size(); ++i) {
                    *(ptr_accumulated_dRdt + object_row_[i] * n_col +
                      object_col_[i]) = 0;
                }
            }
            // std::cout<<"   ====================    " <<std::endl;
            // std::cout<<histogram_<<std::endl;
            // std::cout<<max_idx<<std::endl;
            // std::cout<<bin_range_min<<" " << bin_range_max << std::endl;
            // std::cout<<max_his_average<< std::endl;
            // std::cout<<(float)max_his_object_rho_roi_.size() << std::endl;
        }
        // cv::imshow("accumulated_dRdt", accumulated_dRdt);
        // cv::waitKey(0);

    }  // end for object_idx
    // cv::imshow("accumulated_dRdt", accumulated_dRdt);
    // cv::waitKey(0);
    // exit(0);
}

void ObjectExt::checkSegment(cv::Mat& accumulated_dRdt,
                             std::unique_ptr<CloudFrame>& CloudFrame_next,
                             cv::Mat& groundPtsIdx_next) {
    int n_col = CloudFrame_next->str_rhopts_->img_rho.cols;
    int n_row = CloudFrame_next->str_rhopts_->img_rho.rows;
    float* ptr_accumulated_dRdt = accumulated_dRdt.ptr<float>(0);
    float* ptr_img_rho = CloudFrame_next->str_rhopts_->img_rho.ptr<float>(0);
    uchar* ptr_groundPtsIdx_next = groundPtsIdx_next.ptr<uchar>(0);
    bool isempty = true;

    cv::Mat roi_up = cv::Mat::zeros(img_height_, img_width_, CV_8UC1);
    uchar* ptr_roi_up = roi_up.ptr<uchar>(0);

    idx_row_.resize(0);
    idx_col_.resize(0);
    check_.resize(0);

    for (int i = 0; i < n_row; ++i) {
        int i_ncols = i * n_col;
        for (int j = 0; j < n_col; ++j) {
            if (*(ptr_accumulated_dRdt + i_ncols + j) != 0) {
                idx_row_.push_back(i);
                idx_col_.push_back(j);
                check_.push_back(0);
                isempty = false;
                *(ptr_roi_up + i_ncols + j) = 1;
            }
        }
    }

    if (isempty == true) {
        return;
    }

    float azimuth_res = (float)360.0 / (float)n_col;
    int cnt = 0;
    int row = 0;
    int col = 0;
    float R = 0.0;

    float d1 = 0.0;
    float d2 = 0.0;

    while (1) {
        if (check_[cnt] == 0) {
            row = idx_row_[cnt];
            col = idx_col_[cnt];
            R = *(ptr_img_rho + row * n_col + col);
            int i = 0;
            int j = 0;
            int ii = 0;
            int jj = 0;
            for (int idx_cw = 1; idx_cw < 5; ++idx_cw) {
                switch (idx_cw) {
                    case 1:
                        i = row - 1;
                        j = col;
                        break;
                    case 2:
                        i = row;
                        j = col + 1;
                        break;
                    case 3:
                        i = row + 1;
                        j = col;
                        break;
                    case 4:
                        i = row;
                        j = col - 1;
                        break;
                }

                if ((i < 0) || (i > n_row - 1) || (j < 0) || (j > n_col - 1)) {
                    continue;
                }

                int i_ncols_j = i * n_col + j;
                if ((*(ptr_roi_up + i_ncols_j) == 0) &&
                    (*(ptr_img_rho + i_ncols_j) > 0)) {
                    if (*(ptr_groundPtsIdx_next + i_ncols_j) == 255) {
                        continue;
                    }

                    d1 = std::max(R, *(ptr_img_rho + i_ncols_j));
                    d2 = std::min(R, *(ptr_img_rho + i_ncols_j));
                    if (atan2(d2 * sinf(azimuth_res * D2R),
                              (d1 - d2 * cosf(azimuth_res * D2R))) >
                        seg_deg_ * D2R) {
                        idx_row_.push_back(i);
                        idx_col_.push_back(j);
                        check_.push_back(0);
                        *(ptr_roi_up + i_ncols_j) = 1;
                        *(ptr_accumulated_dRdt + i_ncols_j) =
                            weight_factor_ *
                            *(ptr_accumulated_dRdt + row * n_col + col);
                    }
                } else if ((*(ptr_roi_up + i_ncols_j) == 0) &&
                           (*(ptr_img_rho + i_ncols_j) == 0)) {
                    if (*(ptr_groundPtsIdx_next + i_ncols_j) == 255) {
                        continue;
                    }

                    switch (idx_cw) {
                        case 1:
                            ii = row - 2;
                            jj = col;
                            break;
                        case 2:
                            ii = row;
                            jj = col + 2;
                            break;
                        case 3:
                            ii = row + 2;
                            jj = col;
                            break;
                        case 4:
                            ii = row;
                            jj = col - 2;
                            break;
                    }

                    if ((ii < 0) || (ii > n_row - 1) || (jj < 0) ||
                        (jj > n_col - 1)) {
                        continue;
                    }

                    int ii_ncols_jj = ii * n_col + jj;
                    if (*(ptr_groundPtsIdx_next + ii_ncols_jj) == 255) {
                        continue;
                    }

                    if (*(ptr_roi_up + ii_ncols_jj) == 1) {
                    } else {
                        d1 = std::max(R, *(ptr_img_rho + ii_ncols_jj));
                        d2 = std::min(R, *(ptr_img_rho + ii_ncols_jj));

                        if (atan2f(d2 * sinf(2.0 * azimuth_res * D2R),
                                   (d1 - d2 * cosf(2.0 * azimuth_res * D2R))) >
                            seg_deg_ * D2R) {
                            idx_row_.push_back(ii);
                            idx_col_.push_back(jj);

                            check_.push_back(0);
                            *(ptr_roi_up + ii_ncols_jj) = 1;
                            *(ptr_accumulated_dRdt + ii_ncols_jj) =
                                weight_factor_ *
                                (*(ptr_accumulated_dRdt + row * n_col + col));
                            switch (idx_cw) {
                                case 1:
                                    *(ptr_accumulated_dRdt + (ii + 1) * n_col +
                                      jj) = weight_factor_ *
                                            (*(ptr_accumulated_dRdt +
                                               row * n_col + col));
                                    break;
                                case 2:
                                    *(ptr_accumulated_dRdt + ii * n_col +
                                      (jj - 1)) = weight_factor_ *
                                                  (*(ptr_accumulated_dRdt +
                                                     row * n_col + col));
                                    break;
                                case 3:
                                    *(ptr_accumulated_dRdt + (ii - 1) * n_col +
                                      jj) = weight_factor_ *
                                            (*(ptr_accumulated_dRdt +
                                               row * n_col + col));
                                    break;
                                case 4:
                                    *(ptr_accumulated_dRdt + ii * n_col +
                                      (jj + 1)) = weight_factor_ *
                                                  (*(ptr_accumulated_dRdt +
                                                     row * n_col + col));
                                    break;
                                default:
                                    break;
                            }
                        }
                    }
                }
            }

            if (*(ptr_groundPtsIdx_next + row * n_col + col) == 255) {
                *(ptr_roi_up + row * n_col + col) = 0;
            } else {
            }

            check_[cnt] = 1;
        }  // end for if(check_[cnt]==0)

        cnt += 1;
        if (cnt == check_.size()) {
            break;
        }
    }  // end for while

    // cv::imshow("accumulated_dRdt", accumulated_dRdt);
    // cv::waitKey(0);
    // exit(0);
}

void ObjectExt::updateAccum(cv::Mat& accumulated_dRdt,
                            cv::Mat& accumulated_dRdt_score) {
    int n_row = accumulated_dRdt.rows;
    int n_col = accumulated_dRdt.cols;
    float* ptr_accumulated_dRdt = accumulated_dRdt.ptr<float>(0);
    float* ptr_accumulated_dRdt_score = accumulated_dRdt_score.ptr<float>(0);

    for (int i = 0; i < n_row; ++i) {
        int i_ncols = i * n_col;
        for (int j = 0; j < n_col; ++j) {
            if (*(ptr_accumulated_dRdt + i_ncols + j) != 0) {
                *(ptr_accumulated_dRdt_score + i_ncols + j) += 1;
            }

            if (*(ptr_accumulated_dRdt_score + i_ncols + j) > 2.0) {
                *(ptr_accumulated_dRdt + i_ncols + j) *= 5.0;
                if (*(ptr_accumulated_dRdt + i_ncols + j) > 1e3) {
                    *(ptr_accumulated_dRdt + i_ncols + j) = 1e3;
                } else {
                }
            } else {
            }
        }
    }
}

void ObjectExt::updateAccumdRdt(std::unique_ptr<CloudFrame>& CloudFrame_next,
                                cv::Mat& accumulated_dRdt,
                                cv::Mat& accumulated_dRdt_score, cv::Mat& dRdt,
                                cv::Mat& groundPtsIdx_next) {
    int n_row = accumulated_dRdt.rows;
    int n_col = accumulated_dRdt.cols;

    float* ptr_accumulated_dRdt = accumulated_dRdt.ptr<float>(0);
    float* ptr_accumulated_dRdt_score = accumulated_dRdt_score.ptr<float>(0);
    float* ptr_next_img_rho =
        CloudFrame_next->str_rhopts_->img_rho.ptr<float>(0);
    float* ptr_dRdt = dRdt.ptr<float>(0);

    for (int i = 0; i < n_row; ++i) {
        int i_ncols = i * n_col;
        for (int j = 0; j < n_col; ++j) {
            if (*(ptr_next_img_rho + i_ncols + j) != 0) {
            } else {
                *(ptr_accumulated_dRdt + i_ncols + j) = 0.0;
            }
        }
    }

    bool n_accumulated_dRdt = false;

    for (int i = 0; i < n_row; ++i) {
        int i_ncols = i * n_col;
        for (int j = 0; j < n_col; ++j) {
            if (*(ptr_accumulated_dRdt + i_ncols + j) != 0) {
                n_accumulated_dRdt = true;
                break;
            }
        }
    }

    if (n_accumulated_dRdt == false) {
        for (int i = 0; i < n_row; ++i) {
            int i_ncols = i * n_col;
            for (int j = 0; j < n_col; ++j) {
                if ((*(ptr_dRdt + i_ncols + j) < 0.0) &&
                    *(ptr_dRdt + i_ncols + j) >
                        -(0.1 * *(ptr_next_img_rho + i_ncols + j)) &&
                    (*(ptr_accumulated_dRdt_score + i_ncols + j) > 1.0)) {
                    *(ptr_accumulated_dRdt + i_ncols + j) =
                        -*(ptr_dRdt + i_ncols + j);
                } else {
                }
            }
        }
        this->checkSegment(accumulated_dRdt, CloudFrame_next,
                           groundPtsIdx_next);
    }
}

void ObjectExt::updateAccumScore(cv::Mat& accumulated_dRdt,
                                 cv::Mat& accumulated_dRdt_score) {
    int n_row = accumulated_dRdt.rows;
    int n_col = accumulated_dRdt.cols;

    float* ptr_accumulated_dRdt = accumulated_dRdt.ptr<float>(0);
    float* ptr_accumulated_dRdt_score = accumulated_dRdt_score.ptr<float>(0);

    for (int i = 0; i < n_row; ++i) {
        int i_ncols = i * n_col;
        for (int j = 0; j < n_col; ++j) {
            if (*(ptr_accumulated_dRdt + i_ncols + j) == 0) {
                *(ptr_accumulated_dRdt_score + i_ncols + j) = 0;
            } else {
            }
        }
    }
}
