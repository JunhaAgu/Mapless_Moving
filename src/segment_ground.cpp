#include "segment_ground.h"

SegmentGround::SegmentGround(const std::unique_ptr<UserParam>& user_param) {
    img_height_ = user_param->image_param_.height_;
    img_width_ = user_param->image_param_.width_;
    dataset_name_ = user_param->dataset_name_;

    downsample_size_ = user_param->ground_segment_param_.downsample_size;

    // for RANSAC
    iter_ = user_param->ransac_param_.iter_;
    thr_ = user_param->ransac_param_.thr_;
    a_thr_ = user_param->ransac_param_.a_thr_;        // abs
    b_thr_[0] = user_param->ransac_param_.b_thr_[0];  // under
    b_thr_[1] = user_param->ransac_param_.b_thr_[1];
    mini_inlier_ = user_param->ransac_param_.min_inlier_;
    n_sample_ = user_param->ransac_param_.n_sample_;

    groundPtsIdx_next_ = cv::Mat::zeros(img_height_, img_width_, CV_8UC1);

    std::mt19937 gen_(rd());
};

SegmentGround::~SegmentGround(){

};

void SegmentGround::fastsegmentGround(
    std::unique_ptr<CloudFrame>& CloudFrame_in) {
    int n_row = CloudFrame_in->str_rhopts_->img_rho.rows;
    int n_col = CloudFrame_in->str_rhopts_->img_rho.cols;
    float* ptr_img_z = CloudFrame_in->str_rhopts_->img_z.ptr<float>(0);
    float* ptr_img_rho = CloudFrame_in->str_rhopts_->img_rho.ptr<float>(0);
    uchar* ptr_groundPtsIdx_next = groundPtsIdx_next_.ptr<uchar>(0);

    int n_col_sample = std::round(n_col / downsample_size_);

    cv::Mat pts_z_sample = cv::Mat::zeros(n_row, n_col_sample, CV_32FC1);
    cv::Mat rho_sample = cv::Mat::zeros(n_row, n_col_sample, CV_32FC1);
    float* ptr_pts_z_sample = pts_z_sample.ptr<float>(0);
    float* ptr_rho_sample = rho_sample.ptr<float>(0);

    std::vector<int> col_range;
    std::vector<float> pts_z_col_range;
    col_range.reserve(downsample_size_);
    pts_z_col_range.reserve(downsample_size_);

    std::vector<float> rho_col_range;
    rho_col_range.reserve(downsample_size_);

    float min_z = 0.0;
    int min_z_idx = 0;

    int cnt_z_zero = 0;

    for (int i = 0; i < n_row; ++i) {
        int i_ncols = i * n_col;
        int i_ncols_sample = i * n_col_sample;

        for (int j = 0; j < n_col_sample; ++j) {
            col_range.resize(0);
            pts_z_col_range.resize(0);
            cnt_z_zero = 0;

            int j_downsample_size = j * downsample_size_;
            for (int k = 0; k < downsample_size_; ++k) {
                col_range.push_back(j_downsample_size + k);
                pts_z_col_range.push_back(
                    *(ptr_img_z + i_ncols + j_downsample_size + k));
                if (pts_z_col_range[k] == 0) {
                    pts_z_col_range[k] = 1e2;
                    cnt_z_zero += 1;
                }
            }

            if (cnt_z_zero == downsample_size_) {
                continue;
            }

            min_z =
                *min_element(pts_z_col_range.begin(), pts_z_col_range.end());
            min_z_idx =
                min_element(pts_z_col_range.begin(), pts_z_col_range.end()) -
                pts_z_col_range.begin();

            *(ptr_pts_z_sample + i_ncols_sample + j) = min_z;
            *(ptr_rho_sample + i_ncols_sample + j) =
                *(ptr_img_rho + i_ncols + col_range[0] + min_z_idx);
        }
    }

    std::vector<float> points_rho;
    std::vector<float> points_z;
    bool mask_inlier[n_row];
    points_rho.reserve(n_row);
    points_z.reserve(n_row);
    std::vector<float> line_a;
    std::vector<float> line_b;
    line_a.reserve(n_col_sample);
    line_b.reserve(n_col_sample);

    cv::Mat mask_inlier_mat = cv::Mat::zeros(n_row, n_col_sample, CV_32FC1);
    float* ptr_mask_inlier_mat = mask_inlier_mat.ptr<float>(0);

    for (int j = 0; j < n_col_sample; ++j) {
        points_rho.resize(0);
        points_z.resize(0);

        for (int i = 0; i < n_row; ++i) {
            mask_inlier[i] = false;
            points_rho.push_back(*(ptr_rho_sample + i * n_col_sample + j));
            points_z.push_back(*(ptr_pts_z_sample + i * n_col_sample + j));
        }

        // timer::tic();
        ransacLine(points_rho, points_z, /*output*/ mask_inlier, line_a, line_b,
                   j);
        // double tt = timer::toc();
        // ROS_INFO_STREAM("AA :" << tt << " [ms]");
        // std::cout<< " " <<std::endl;

        for (int i = 0; i < n_row; ++i) {
            if (mask_inlier[i] == true) {
                *(ptr_mask_inlier_mat + i * n_col_sample + j) = 255;
            }
        }
    }

    // exit(0);
    // std::cout << mask_inlier_mat <<std::endl;
    // exit(0);

    // cv::imshow("mat", mask_inlier_mat);
    // cv::waitKey(0);

    // cv::FileStorage fs_w("/home/junhakim/mask_inlier_mat.yaml",
    // cv::FileStorage::WRITE); fs_w << "matImage" << mask_inlier_mat;
    // fs_w.release();
    // exit(0);

    ////////////////////////////////////////////////////////
    //////////////////////////// upscale: sample size -> original image size
    float rep_z_value = 0.0;
    float thr_dist = 0.1;
    float thr_z = 2.0;
    if (dataset_name_ =="CARLA")
    {
        thr_dist = 0.2;
        thr_z = 3.0;
    }

    for (int i = 0; i < n_row; ++i) {
        int i_ncols = i * n_col;
        for (int j = 0; j < n_col_sample; ++j) {
            col_range.resize(0);
            pts_z_col_range.resize(0);
            rho_col_range.resize(0);
            int j_downsample_size = j * downsample_size_;
            if (*(ptr_mask_inlier_mat + i * n_col_sample + j) == 255) {
                rep_z_value = *(ptr_pts_z_sample + i * n_col_sample + j);
                if (rep_z_value == 0) {
                    continue;
                } else {
                }

                double den_ls = 1.0 / std::sqrt(line_a[j] * line_a[j] + 1);
                float residual_leastsquare = 0.0;
                float line_updown = 0.0;

                for (int k = 0; k < downsample_size_; ++k) {
                    col_range.push_back(j_downsample_size + k);
                    pts_z_col_range.push_back(
                        *(ptr_img_z + i_ncols + j_downsample_size + k));
                    rho_col_range.push_back(
                        *(ptr_img_rho + i_ncols + j_downsample_size + k));

                    residual_leastsquare =
                        std::abs(line_a[j] * rho_col_range[k] -
                                 pts_z_col_range[k] + line_b[j]) *
                        den_ls;  // t(1)=-1
                    line_updown = line_a[j] * rho_col_range[k] -
                                  pts_z_col_range[k] + line_b[j];

                    if ((pts_z_col_range[k] != 0) &&
                        (residual_leastsquare < thr_dist)) {
                        *(ptr_groundPtsIdx_next + i_ncols + col_range[k]) = 255;
                    } else if ((line_updown > 0) && (line_updown < thr_z)) {
                        *(ptr_groundPtsIdx_next + i_ncols + col_range[k]) = 255;
                    } else {
                        *(ptr_groundPtsIdx_next + i_ncols + col_range[k]) = 0;
                    }
                }
            }
        }
    }

    /* for debug */
    // cv::imshow("groundPtsIdx_next", groundPtsIdx_next_);
    // cv::waitKey(0);
}

void SegmentGround::ransacLine(std::vector<float>& points_rho,
                               std::vector<float>& points_z,
                               /*output*/ bool mask_inlier[],
                               std::vector<float>& line_a,
                               std::vector<float>& line_b, int num_seg) {
    // timer::tic();
    float* ptr_points_rho = points_rho.data();
    float* ptr_points_z = points_z.data();
    float* ptr_line_a = line_a.data();
    float* ptr_line_b = line_b.data();

    // copy points_rho vector
    std::vector<float> points_rho_dup;
    points_rho_dup.resize(points_rho.size());
    std::copy(points_rho.begin(), points_rho.end(), points_rho_dup.begin());
    float* ptr_points_rho_dup = points_rho_dup.data();

    // copy points_z vector
    std::vector<float> points_z_dup;
    points_z_dup.resize(points_rho.size());
    std::copy(points_z.begin(), points_z.end(), points_z_dup.begin());
    float* ptr_points_z_dup = points_z_dup.data();

    int max_range = 100;  // 80
    int n_bin_per_seg = points_rho.size();

    bool flag_mask_temp = false;

    // std::vector<float> points_rho_sort_temp;
    std::vector<float> points_z_sort_temp;
    // points_rho_sort_temp.reserve(n_bin_per_seg);
    points_z_sort_temp.reserve(n_bin_per_seg);

    std::vector<int> idx_non_zero_mask_temp;
    idx_non_zero_mask_temp.reserve(n_bin_per_seg);
    int* ptr_idx_non_zero_mask_temp = idx_non_zero_mask_temp.data();

    std::vector<int> valid_points_idx;
    valid_points_idx.reserve(max_range);
    int* ptr_valid_points_idx = valid_points_idx.data();

    int kk;

    for (int i = 0; i < max_range; ++i) {
        flag_mask_temp = false;
        // points_rho_sort_temp.resize(0);
        points_z_sort_temp.resize(0);
        idx_non_zero_mask_temp.resize(0);
        float min_z = 0.0;
        int min_z_idx = 0;

        for (int j = 0; j < n_bin_per_seg; ++j) {
            if (ptr_points_rho[j] > i && ptr_points_rho[j] < i + 1.0) {
                flag_mask_temp = true;

                // points_rho_sort_temp.push_back(ptr_points_rho[j]);
                points_z_sort_temp.push_back(ptr_points_z[j]);
                idx_non_zero_mask_temp.push_back(j);
            } else {
            }
        }
        if (flag_mask_temp == true) {
            min_z = *min_element(points_z_sort_temp.begin(),
                                 points_z_sort_temp.end());
            min_z_idx = min_element(points_z_sort_temp.begin(),
                                    points_z_sort_temp.end()) -
                        points_z_sort_temp.begin();
            if (min_z > -1.2) {
                continue;
            }
            valid_points_idx.push_back(ptr_idx_non_zero_mask_temp[min_z_idx]);
        }
    }

    std::vector<float> points_valid_rho;
    points_valid_rho.reserve(points_rho.size());
    float* ptr_points_valid_rho = points_valid_rho.data();

    std::vector<float> points_valid_z;
    points_valid_z.reserve(points_rho.size());
    float* ptr_points_valid_z = points_valid_z.data();

    for (int i = 0; i < valid_points_idx.size(); ++i) {
        points_valid_rho.push_back(ptr_points_rho[ptr_valid_points_idx[i]]);
        points_valid_z.push_back(ptr_points_z[ptr_valid_points_idx[i]]);
    }

    if (points_valid_rho.size() < 2) {
        // ROS_INFO_STREAM("Not enough pts for RANSAC");
        // std::cout << "number of segment: " << num_seg << std::endl;
        return;
    }

    int n_pts_valid = points_valid_rho.size();

    // bool id_good_fit[iter_];
    bool ini_inlier[iter_][n_pts_valid];
    // bool mask[iter_][n_pts_valid];
    float residual[iter_][n_pts_valid];
    int inlier_cnt[iter_];

    std::vector<pcl::PointCloud<pcl::PointXY>> inlier;
    inlier.resize(iter_);

    float line_A[iter_];
    float line_B[iter_];

    for (int i = 0; i < iter_; ++i) {
        // id_good_fit[i] = false;
        inlier_cnt[i] = 0;
        for (int j = 0; j < n_pts_valid; ++j) {
            ini_inlier[i][j] = false;
            // mask[i][j] = false;
            residual[i][j] = 0.0;
        }

        inlier[i].reserve(points_rho.size());
        line_A[i] = 0.0;
        line_B[i] = 0.0;
    }

    // bool id_good_fit[iter_];
    // for (int i = 0; i < iter_; ++i)
    // {
    //     id_good_fit[i] = false;
    // }

    // bool ini_inlier[iter_][n_pts_valid];
    // for (int i = 0; i < iter_; ++i)
    // {
    //     for (int j = 0; j < n_pts_valid; ++j)
    //     {
    //         ini_inlier[i][j] = false;
    //     }
    // }

    // bool mask[iter_][n_pts_valid];
    // for (int i = 0; i < iter_; ++i)
    // {
    //     for (int j = 0; j < n_pts_valid; ++j)
    //     {
    //         mask[i][j] = false;
    //     }
    // }

    // float residual[iter_][n_pts_valid];
    // for (int i = 0; i < iter_; ++i)
    // {
    //     for (int j = 0; j < n_pts_valid; ++j)
    //     {
    //         residual[i][j] = 0.0;
    //     }
    // }

    // int inlier_cnt[iter_];
    // for (int i = 0; i < iter_; ++i)
    // {
    //     inlier_cnt[i] = 0;
    // }

    // for (int i=0; i<iter_; ++i)
    // {
    //     inlier[i].reserve(points_rho.size());
    // }

    // float line_A[iter_];
    // for (int i=0; i<iter_; ++i)
    // {
    //     line_A[i] = 0.0;
    // }
    // float line_B[iter_];
    // for (int i=0; i<iter_; ++i)
    // {
    //     line_B[i] = 0.0;
    // }

    int n1 = 0;
    int n2 = 0;
    float x1 = 0.0, x2 = 0.0, y1 = 0.0, y2 = 0.0;

    // std::random_device rd;
    // std::mt19937 gen_(rd());
    std::uniform_int_distribution<int> dis(0, n_pts_valid - 1);

    for (int m = 0; m < iter_; ++m) {
        inlier_cnt[m] = 1;

        while (1) {
            n1 = dis(gen_);
            n2 = dis(gen_);
            if (n1 != n2) {
                // std::cout<< 0 << "~" <<n_pts_valid-1<<": "<<n1<<",
                // "<<n2<<std::endl;
                break;
            }
        }

        x1 = ptr_points_valid_rho[n1];
        y1 = ptr_points_valid_z[n1];
        x2 = ptr_points_valid_rho[n2];
        y2 = ptr_points_valid_z[n2];

        line_A[m] = (y2 - y1) / (x2 - x1);
        line_B[m] = -line_A[m] * x1 + y1;

        if (line_A[m] < 0.0) {
            if (line_A[m] < -a_thr_ || line_B[m] > b_thr_[0]) {
                continue;
            } else {
            }
        } else if (line_A[m] > 0.0) {
            if (line_A[m] > a_thr_ || line_B[m] > b_thr_[1]) {
                continue;
            } else {
            }
        } else {
        }
        double den = 1.0 / std::sqrt(line_A[m] * line_A[m] + 1.0);
        pcl::PointXY point_xy;

        for (int i = 0; i < n_pts_valid; ++i) {
            residual[m][i] = std::abs(line_A[m] * ptr_points_valid_rho[i] -
                                      ptr_points_valid_z[i] + line_B[m]) *
                             den;

            if (residual[m][i] < thr_) {
                ini_inlier[m][i] = true;
                // mask[m][i] = true;
                point_xy.x = ptr_points_valid_rho[i];
                point_xy.y = ptr_points_valid_z[i];
                inlier[m].push_back(point_xy);
                inlier_cnt[m] += 1;
            }
        }

        // for (int j=0; j<n_pts_valid; ++j)
        // {
        //     if (ini_inlier[m][j]==true)
        //     {
        //         point_xy.x = ptr_points_valid_rho[j];
        //         point_xy.y = ptr_points_valid_z[j];
        //         inlier[m].push_back(point_xy);
        //         inlier_cnt[m] += 1;
        //     }
        // }

        // if ((inlier_cnt[m] - 1) > mini_inlier_)
        // {
        //     id_good_fit[m] = true;
        // }
    }

    int max_inlier_cnt = 0;
    std::vector<int> max_inlier_cnt_index;
    max_inlier_cnt_index.reserve(100);

    max_inlier_cnt = *std::max_element(inlier_cnt, inlier_cnt + iter_);

    for (int i = 0; i < iter_; ++i) {
        if (inlier_cnt[i] == max_inlier_cnt) {
            max_inlier_cnt_index.push_back(i);
        }
    }

    float mini_pre = 1e3;

    if (max_inlier_cnt_index.size() == 0) {
        return;
    }

    int id_mini = 0;
    int max_inlier_cnt_index_1 = 0;
    float inv_n_pts_valid = 1 / n_pts_valid;
    if (max_inlier_cnt_index.size() > 1) {
        int n_candi = max_inlier_cnt_index.size();
        for (int i_candi = 0; i_candi < n_candi; ++i_candi) {
            float mean_residual = 0.0;
            for (int k = 0; k < n_pts_valid; ++k) {
                mean_residual += residual[max_inlier_cnt_index[i_candi]][k];
            }

            mean_residual *= inv_n_pts_valid;

            float mini = std::min(mean_residual, mini_pre);

            if (mini < mini_pre) {
                id_mini = i_candi;
            }
            mini_pre = mini;
        }
        max_inlier_cnt_index_1 = max_inlier_cnt_index[id_mini];
    } else {
        max_inlier_cnt_index_1 = max_inlier_cnt_index[0];
    }
    int best_n_inlier = inlier_cnt[max_inlier_cnt_index_1] - 1;

    // if (best_n_inlier < 3)
    // {
    //     // ROS_INFO_STREAM("inlier pts < 3");
    //     // std::cout << "number of segment: " << num_seg << std::endl;
    //     // return;
    // }
    if (best_n_inlier == 0) {
        // ROS_INFO_STREAM("# of inlier pts = 0");
        return;
    }

    Eigen::MatrixXf A = Eigen::MatrixXf::Zero(best_n_inlier, 3);
    for (int i = 0; i < best_n_inlier; ++i) {
        A(i, 0) = inlier[max_inlier_cnt_index_1][i].x;
        A(i, 1) = inlier[max_inlier_cnt_index_1][i].y;
        A(i, 2) = 1;
    }

    Eigen::Vector3f t;
    Eigen::JacobiSVD<Eigen::MatrixXf> svd(
        A, Eigen::ComputeFullU | Eigen::ComputeFullV);
    t = svd.matrixV().col(2);
    // std::cout << svd.matrixU() << std::endl;
    // std::cout << "***************" << std::endl;
    // std::cout << svd.matrixV() << std::endl;
    // std::cout << "***************" << std::endl;
    // std::cout << svd.singularValues() << std::endl;
    // exit(0);

    // Eigen::JacobiSVD<Eigen::MatrixXf> svd(A, Eigen::ComputeThinU |
    // Eigen::ComputeThinV);
    // // cout << "Its singular values are:" << endl << svd.singularValues() <<
    // endl;
    // // cout << "Its left singular vectors are the columns of the thin U
    // matrix:" << endl << svd.matrixU() << endl;
    // // cout << "Its right singular vectors are the columns of the thin V
    // matrix:" << endl << svd.matrixV() << endl;

    t = -t / t(1, 0);

    // Eigen::Vector2f line;
    // line << t(0), t(2); //y = ax + b

    line_a[num_seg] = t(0);
    line_b[num_seg] = t(2);

    float residual_leastsquare = 0.0;
    float line_updown = 0.0;
    // int cnt_inlier = 0;

    double den_ls = 1.0 / std::sqrt(t(0) * t(0) + t(1) * t(1));
    for (int i = 0; i < points_rho_dup.size(); ++i) {
        residual_leastsquare = std::abs(t(0) * ptr_points_rho_dup[i] -
                                        ptr_points_z_dup[i] + t(2)) *
                               den_ls;  // t(1)=-1
        line_updown = t(0) * ptr_points_rho_dup[i] - ptr_points_z_dup[i] + t(2);

        if ((residual_leastsquare < 2.0 * thr_) || (line_updown > 0)) {
            mask_inlier[i] = true;
            // cnt_inlier += 1;
        }
    }
    // output: mask_inlier
    // std::cout << t(0) << " " << t(2) << " # of inlier: " << cnt_inlier
    // <<std::endl; double dt_toc2 = timer::toc(); // milliseconds
    // ROS_INFO_STREAM("AA :" << dt_toc2 << " [ms]");
}

void SegmentGround::reset() {
    this->groundPtsIdx_next_ = cv::Mat::zeros(img_height_, img_width_, CV_8UC1);
}