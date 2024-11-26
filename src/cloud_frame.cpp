#include "cloud_frame.h"

CloudFrame::CloudFrame(const std::unique_ptr<UserParam>& user_param) {
    // user param
    // ///////////////////////////////////////////////////////////////////////////////////
    dataset_name_ = user_param->dataset_name_;

    azimuth_res_ = user_param->cloud_filter_param_.azimuth_res_ *
                   (float)user_param->cloud_filter_param_.h_factor_;
    az_step_ = 1.0f / azimuth_res_;
    n_horizontal_ = 360 * az_step_ + 1;
    n_vertical_ = 64 / user_param->cloud_filter_param_.v_factor_;

    for (int i = 0; i < user_param->sensor_spec_.v_angle_.size(); ++i) {
        v_angle_.push_back(user_param->sensor_spec_.v_angle_[i]);
    }
    lidar_elevation_criteria_[0] =
        user_param->sensor_spec_.lidar_elevation_criteria_[0];
    lidar_elevation_criteria_[1] =
        user_param->sensor_spec_.lidar_elevation_criteria_[1];
    lidar_elevation_criteria_[2] =
        user_param->sensor_spec_.lidar_elevation_criteria_[2];
    lidar_elevation_criteria_[3] =
        user_param->sensor_spec_.lidar_elevation_criteria_[3];
    lidar_elevation_line0_[0] =
        user_param->sensor_spec_.lidar_elevation_line0_[0];
    lidar_elevation_line0_[1] =
        user_param->sensor_spec_.lidar_elevation_line0_[1];
    lidar_elevation_line1_[0] =
        user_param->sensor_spec_.lidar_elevation_line1_[0];
    lidar_elevation_line1_[1] =
        user_param->sensor_spec_.lidar_elevation_line1_[1];

    img_height_ = user_param->image_param_.height_;
    img_width_ = user_param->image_param_.width_;
    ///////////////////////////////////////////////////////////////////////////////////

    //
    str_rhopts_ = std::make_shared<StrRhoPts>();

    str_rhopts_->rho.reserve(5000000);
    str_rhopts_->phi.reserve(5000000);
    str_rhopts_->theta.reserve(5000000);
    str_rhopts_->img_rho = cv::Mat::zeros(img_height_, img_width_, CV_32FC1);
    str_rhopts_->img_index = cv::Mat::zeros(img_height_, img_width_, CV_32SC1);

    str_rhopts_->img_x = cv::Mat::zeros(img_height_, img_width_, CV_32FC1);
    str_rhopts_->img_y = cv::Mat::zeros(img_height_, img_width_, CV_32FC1);
    str_rhopts_->img_z = cv::Mat::zeros(img_height_, img_width_, CV_32FC1);

    // str_rhopts_->pts_per_pixel_n.resize(img_height_ * img_width_);

    int size_img_vector = img_height_ * img_width_;
    str_rhopts_->pts_per_pixel_index.resize(size_img_vector);
    str_rhopts_->pts_per_pixel_rho.resize(size_img_vector);
    str_rhopts_->pts_per_pixel_index_valid.resize(size_img_vector);
    for (int i = 0; i < size_img_vector; ++i) {
        str_rhopts_->pts_per_pixel_index[i].reserve(5000);
    }
    for (int i = 0; i < size_img_vector; ++i) {
        str_rhopts_->pts_per_pixel_rho[i].reserve(5000);
    }
    for (int i = 0; i < size_img_vector; ++i) {
        str_rhopts_->pts_per_pixel_index_valid[i].reserve(5000);
    }
    // str_rhopts_->pts        =
    // boost::make_shared<pcl::PointCloud<pcl::PointXYZI>>();
    str_rhopts_->img_restore_mask =
        cv::Mat::zeros(img_height_, img_width_, CV_32SC1);
    str_rhopts_->img_restore_warp_mask =
        cv::Mat::zeros(img_height_, img_width_, CV_32SC1);
};

CloudFrame::~CloudFrame(){
    // destructor
};

void CloudFrame::genRangeImages(PointCloudwithTime::Ptr pcl_in, bool cur_next) {
    n_pts_ = pcl_in->size();

    ////////////////// calculate rho, phi, theta
    // timer::tic();
    calcuateRho(pcl_in, cur_next);
    // double dt_calRho = timer::toc(); // milliseconds
    // ROS_INFO_STREAM("elapsed time for 'calcuateRho' :" << dt_calRho << "
    // [ms]");

    ////////////////// make range image and Pts per pixel
    // timer::tic();
    makeRangeImageAndPtsPerPixel(cur_next);
    // double dt_makeRangeImage = timer::toc(); // milliseconds
    // ROS_INFO_STREAM("elapsed time for 'makeRangeImageAndPtsPerPixel' :" <<
    // dt_makeRangeImage << " [ms]");

    ////////////////// fill range image using interpolation
    // timer::tic();
    interpRangeImage(cur_next);
    // double dt_interpRho = timer::toc(); // milliseconds
    // ROS_INFO_STREAM("elapsed time for 'interpRangeImage' :" << dt_interpRho
    // << " [ms]");

    ////////////////// fill pts corresponding to filled range image (no affect
    /// the original pts)
    // timer::tic();
    interpPts(pcl_in, cur_next);
    // double dt_interpPts = timer::toc(); // milliseconds
    // ROS_INFO_STREAM("elapsed time for 'interpPts' :" << dt_interpPts << "
    // [ms]");
}

void CloudFrame::genRangeImages_dR(PointCloudwithTime::Ptr pcl_in,
                                   bool cur_next) {
    n_pts_ = pcl_in->size();

    // timer::tic();
    calcuateRho(pcl_in, cur_next);
    // double dt_1 = timer::toc(); // milliseconds
    // ROS_INFO_STREAM("elapsed time for 'calcuateRho' :" << dt_1 << " [ms]");

    // timer::tic();
    makeRangeImageAndPtsPerPixel_dR(cur_next);
    // double dt_2 = timer::toc(); // milliseconds
    // ROS_INFO_STREAM("elapsed time for 'makeRangeImageAndPtsPerPixel_dR' :" << dt_2 << " [ms]");

    // timer::tic();
    interpRangeImage_dR(cur_next);
    // double dt_3 = timer::toc(); // milliseconds
    // ROS_INFO_STREAM("elapsed time for 'interpRangeImage_dR' :" << dt_3 << " [ms]");
}

void CloudFrame::genRangeImages_noComp(PointCloudwithTime::Ptr pcl_in,
                                       bool cur_next) {
    n_pts_ = pcl_in->size();

    calcuateRho(pcl_in, cur_next);

    makeRangeImageAndPtsPerPixel(cur_next);
}

void CloudFrame::calcuateRho(PointCloudwithTime::Ptr pcl_in, bool cur_next) {
    const float twopi = 2.0f * M_PI;
    const float offset_theta = M_PI;
    const int  n_pts = pcl_in->size();

    const float M_PI_plus_offset_theta = M_PI + offset_theta;
    const float twopi_plus_offset_theta = twopi + offset_theta;

    float invrhocos = 0.0;
    float cospsi = 0.0;
    float sinpsi = 0.0;

    // // Resize (n_pts)
    // str_rhopts_->rho.resize(n_pts);
    // str_rhopts_->phi.resize(n_pts);
    // str_rhopts_->theta.resize(n_pts);

    float* ptr_rho = str_rhopts_->rho.data();
    float* ptr_phi = str_rhopts_->phi.data();
    float* ptr_theta = str_rhopts_->theta.data();

    for (int i = 0; i < n_pts; ++i, ++ptr_rho, ++ptr_phi, ++ptr_theta) {
        const slam::PointXYZT& pclpxyzi_tmp = pcl_in->points[i];
        const float& x_tmp = pclpxyzi_tmp.x;
        const float& y_tmp = pclpxyzi_tmp.y;
        const float& z_tmp = pclpxyzi_tmp.z;

        float& rho_tmp = *ptr_rho;
        float& phi_tmp = *ptr_phi;
        float& theta_tmp = *ptr_theta;

        // rho_tmp = NORM(x_tmp, y_tmp, z_tmp);
        // phi_tmp = asinf(z_tmp / rho_tmp);
        // invrhocos = 1.0f / (rho_tmp * cosf(phi_tmp));
        
        rho_tmp = NORM(x_tmp, y_tmp, z_tmp);
        const float inv_rho = 1.0f / rho_tmp;

        // compute phi (angle)
        float sin_phi = z_tmp * inv_rho;
        phi_tmp = asinf(sin_phi);
        invrhocos = inv_rho / cosf(phi_tmp);

        cospsi = x_tmp * invrhocos;
        sinpsi = y_tmp * invrhocos;

        // if (cospsi > 1) {
        //     // std::cout << "(cospsi > 1): " << cospsi <<std::endl;
        //     cospsi = 1.0f;

        // } else if (cospsi < -1) {
        //     // std::cout << "(cospsi < -1): " << cospsi <<std::endl;
        //     cospsi = -1.0f;
        // } else {
        // }
        cospsi = fminf(fmaxf(cospsi, -1.0f), 1.0f);

        // if (cospsi >= 0) {
        //     if (sinpsi >= 0)  // 1 quadrant
        //     {
        //         theta_tmp = acosf32(cospsi) + offset_theta;
        //     } else  // 4 quadrant
        //     {
        //         theta_tmp = twopi_plus_offset_theta - acosf32(cospsi);
        //     }
        // } else {
        //     if (sinpsi >= 0)  // 2 quadrant
        //     {
        //         theta_tmp = M_PI_plus_offset_theta - acosf32(-cospsi);
        //     } else  // 3 quadrant
        //     {
        //         theta_tmp = M_PI_plus_offset_theta + acosf32(-cospsi);
        //     }
        // }
        if (cospsi >= 0) {
            if (cospsi > 1){
                theta_tmp = (sinpsi >= 0) ? offset_theta : twopi_plus_offset_theta;
            }
            else{
                theta_tmp = (sinpsi >= 0) ? acosf(cospsi) + offset_theta : twopi_plus_offset_theta - acosf(cospsi);
            }
        } else {
            if (cospsi < -1){
                theta_tmp = (sinpsi >= 0) ? M_PI : twopi;
            }
            else{
                theta_tmp = (sinpsi >= 0) ? M_PI_plus_offset_theta - acosf(-cospsi) : M_PI_plus_offset_theta + acosf(-cospsi);
            }
        }

        if (theta_tmp >= twopi) {
            // theta_tmp = theta_tmp - twopi;
            theta_tmp -= twopi;
        }
        // std::cout << str_rhopts_->rho[i] << " " << str_rhopts_->phi[i] << " "
        // << str_rhopts_->theta[i]<< std::endl;
    }
};

void CloudFrame::calcuateRho_SIMD(PointCloudwithTime::Ptr pcl_in,
                                  bool cur_next) {
    float twopi = 2.0 * M_PI;
    float offset_theta = M_PI;
    __m256 __twopi = _mm256_set1_ps(twopi);
    __m256 __offset_theta = _mm256_set1_ps(offset_theta);

    int n_pts = pcl_in->size();

    float invrhocos = 0.0;
    float cospsi = 0.0;
    float sinpsi = 0.0;

    // Resize (n_pts)
    str_rhopts_->rho.resize(n_pts);
    str_rhopts_->phi.resize(n_pts);
    str_rhopts_->theta.resize(n_pts);

    float* ptr_rho = str_rhopts_->rho.data();
    float* ptr_phi = str_rhopts_->phi.data();
    float* ptr_theta = str_rhopts_->theta.data();

    float M_PI_plus_offset_theta = M_PI + offset_theta;
    float twopi_plus_offset_theta = twopi + offset_theta;
    __m256 M_PI_offset = _mm256_set1_ps(M_PI_plus_offset_theta);
    __m256 twopi_offset = _mm256_set1_ps(twopi_plus_offset_theta);

    int steps = n_pts / 8;
    int n_pts_simd = steps * 8;
    int n_pts_remain = n_pts - n_pts_simd;

    std::cout << "steps: " << steps << std::endl;

    for (int i = 0; i < steps; i += 8) {
        const slam::PointXYZT& pclpxyzi_tmp = pcl_in->points[i];
        const float& x_tmp = pclpxyzi_tmp.x;
        const float& y_tmp = pclpxyzi_tmp.y;
        const float& z_tmp = pclpxyzi_tmp.z;
        // pppp =  _mm_load_ps(points_array[i]);
    }

    for (int i = n_pts_simd; i < n_pts; ++i) {
    }
};

void CloudFrame::makeRangeImageAndPtsPerPixel(bool cur_next) {
    const int n_row = str_rhopts_->img_rho.rows;
    const int n_col = str_rhopts_->img_rho.cols;
    const int n_vertical_minus_1 = n_vertical_ - 1;

    float* ptr_img_rho = str_rhopts_->img_rho.ptr<float>(0);
    int* ptr_img_index = str_rhopts_->img_index.ptr<int>(0);

    float* ptr_rho = str_rhopts_->rho.data();
    float* ptr_phi = str_rhopts_->phi.data();
    float* ptr_theta = str_rhopts_->theta.data();

    const float az_step_R2D = az_step_ * R2D;
    const float criteria0 = lidar_elevation_criteria_[0];
    const float criteria1 = lidar_elevation_criteria_[1];
    const float criteria2 = lidar_elevation_criteria_[2];
    const float criteria3 = lidar_elevation_criteria_[3];
    const float line0_a = lidar_elevation_line0_[0];
    const float line0_b = lidar_elevation_line0_[1];
    const float line1_a = lidar_elevation_line1_[0];
    const float line1_b = lidar_elevation_line1_[1];

    for (int i = 0; i < n_pts_; ++i, ++ptr_phi, ++ptr_theta, ++ptr_rho) {
        float phi_tmp = *ptr_phi * R2D;
        float theta_tmp = *ptr_theta;
        float rho_tmp = *ptr_rho;

        // Determine row index (i_row) based on phi_tmp
        int i_row = 0;
        if (phi_tmp > criteria0) {
            i_row = 0;
        } else if (phi_tmp > criteria1) {
            i_row = static_cast<int>(ceil(line0_a * phi_tmp + line0_b));
        } else if (phi_tmp > criteria2) {
            i_row = 32;
        } else if (phi_tmp > criteria3) {
            i_row = static_cast<int>(ceil(line1_a * phi_tmp + line1_b));
        } else {
            i_row = 63;
        }

        // Check if row is valid
        if (i_row > n_vertical_minus_1 || i_row < 0) {
            continue;
        }

        // Calculate column index (i_col)
        int i_col = static_cast<int>(roundf(theta_tmp * az_step_R2D));

        // Compute flattened index
        int i_row_ncols_i_col = i_row * n_col + i_col;

        // Update image and index data
        float& img_rho_tmp = *(ptr_img_rho + i_row_ncols_i_col);
        if (img_rho_tmp == 0.0f || img_rho_tmp > rho_tmp) {
            img_rho_tmp = rho_tmp;
            *(ptr_img_index + i_row_ncols_i_col) = i;
        }

        // Add point data to per-pixel vectors
        str_rhopts_->pts_per_pixel_index[i_row_ncols_i_col].emplace_back(i);
        str_rhopts_->pts_per_pixel_rho[i_row_ncols_i_col].emplace_back(rho_tmp);
    }
}


void CloudFrame::makeRangeImageAndPtsPerPixel_dR(bool cur_next) {
    int i_row = 0;
    int i_col = 0;

    float* ptr_img_rho = str_rhopts_->img_rho.ptr<float>(0);

    int n_row = str_rhopts_->img_rho.rows;
    int n_col = str_rhopts_->img_rho.cols;

    float* ptr_rho = str_rhopts_->rho.data();
    float* ptr_phi = str_rhopts_->phi.data();
    float* ptr_theta = str_rhopts_->theta.data();
    float* ptr_v_angle = v_angle_.data();

    float az_step_R2D = az_step_ * R2D;

    float phi_R2D = 0.0f;

    int n_vertical_minus_1 = n_vertical_ - 1;

    const float& criteria0 = lidar_elevation_criteria_[0];
    const float& criteria1 = lidar_elevation_criteria_[1];
    const float& criteria2 = lidar_elevation_criteria_[2];
    const float& criteria3 = lidar_elevation_criteria_[3];
    const float& line0_a = lidar_elevation_line0_[0];
    const float& line0_b = lidar_elevation_line0_[1];
    const float& line1_a = lidar_elevation_line1_[0];
    const float& line1_b = lidar_elevation_line1_[1];

    for (int i = 0; i < n_pts_; ++i, ++ptr_phi, ++ptr_theta, ++ptr_rho) {
        float& phi_tmp = *ptr_phi;
        phi_tmp *= R2D;
        float& theta_tmp = *ptr_theta;
        float& rho_tmp = *ptr_rho;

        if (phi_tmp > criteria0)  // 2.5[degree]
        {
            i_row = 0;
        } else if (phi_tmp > criteria1)  // -8.0[degree]
        {
            i_row = (int)ceil((line0_a * phi_tmp + line0_b));
        } else if (phi_tmp > criteria2)  // -8.5[degree]
        {
            i_row = 32;
        } else if (phi_tmp > criteria3)  // -23.8[degree]
        {
            i_row = (int)ceil((line1_a * phi_tmp + line1_b));
        } else {
            i_row = 63;
        }

        i_col = roundf(theta_tmp * az_step_R2D);

        if ((i_row > n_vertical_minus_1) || (i_row < 0)) {
            continue;
        }

        int i_row_ncols_i_col = i_row * n_col + i_col;
        float& img_rho_tmp = *(ptr_img_rho + i_row_ncols_i_col);
        if (img_rho_tmp == 0 || img_rho_tmp > rho_tmp)
        //(str_rhopts_->img_rho.at<float>(i_row,i_col) == 0)
        {
            img_rho_tmp = rho_tmp;
        } else {
        }
    }  // end for
}

void CloudFrame::interpRangeImage(bool cur_next) {
    int n_col = str_rhopts_->img_rho.cols;
    int n_row = str_rhopts_->img_rho.rows;

    cv::Mat img_rho_new = str_rhopts_->img_rho.clone();
    float* ptr_img_rho_new = img_rho_new.ptr<float>(0);

    float* ptr_img_rho = str_rhopts_->img_rho.ptr<float>(0);
    int* ptr_img_index = str_rhopts_->img_index.ptr<int>(0);
    int* ptr_img_restore_mask = str_rhopts_->img_restore_mask.ptr<int>(0);

    int i_ncols = 0;
    int i_minus_ncols = 0;
    int i_plus_ncols = 0;

    int n_horizontal_minus_2 = (n_horizontal_ - 2);

    // for (int i = 23; i < 36; ++i){
    for (int i = 35; i > 22; --i) {
        i_ncols = i * n_col;
        i_minus_ncols = i_ncols - n_col;  // i_minus_ncols = (i - 1) * n_col;
        i_plus_ncols = i_ncols + n_col;   // i_plus_ncols = (i + 1) * n_col;

        for (int j = 2; j < n_horizontal_minus_2; ++j) {
            if (*(ptr_img_rho + i_ncols + j) == 0) {
                if ((*(ptr_img_rho + i_minus_ncols + j) != 0)) {
                    if ((*(ptr_img_rho + i_plus_ncols + j) != 0)) {
                        if (fabsf32(*(ptr_img_rho + i_minus_ncols + j) -
                                    *(ptr_img_rho + i_plus_ncols + j)) < 0.1) {
                            *(ptr_img_restore_mask + i_ncols + j) = 1;
                            *(ptr_img_rho_new + i_ncols + j) =
                                (*(ptr_img_rho + i_minus_ncols + j) +
                                 *(ptr_img_rho + i_plus_ncols + j)) *
                                0.5;
                        } else {
                            *(ptr_img_restore_mask + i_ncols + j) = 10;
                            if (cur_next == false) {
                                *(ptr_img_rho_new + i_ncols + j) =
                                    std::min(*(ptr_img_rho + i_minus_ncols + j),
                                             *(ptr_img_rho + i_plus_ncols + j));
                            } else {
                                *(ptr_img_rho_new + i_ncols + j) =
                                    std::max(*(ptr_img_rho + i_minus_ncols + j),
                                             *(ptr_img_rho + i_plus_ncols + j));
                            }
                        }
                    } else if ((*(ptr_img_rho + (i + 2) * n_col + j) != 0)) {
                        if (fabsf32(*(ptr_img_rho + i_minus_ncols + j) -
                                    *(ptr_img_rho + i_plus_ncols + n_col + j)) <
                            0.1) {
                            *(ptr_img_restore_mask + i_ncols + j) = 2;
                            *(ptr_img_restore_mask + i_plus_ncols + j) = 3;
                            *(ptr_img_rho_new + i_ncols + j) =
                                *(ptr_img_rho + i_minus_ncols + j) *
                                    (0.6666667) +
                                *(ptr_img_rho + i_plus_ncols + n_col + j) *
                                    (0.3333333);
                            *(ptr_img_rho_new + i_plus_ncols + j) =
                                *(ptr_img_rho + i_minus_ncols + j) *
                                    (0.3333333) +
                                *(ptr_img_rho + i_plus_ncols + n_col + j) *
                                    (0.6666667);
                        } else {
                            *(ptr_img_restore_mask + i_ncols + j) = 20;
                            *(ptr_img_restore_mask + i_plus_ncols + j) = 30;
                            if (cur_next == false) {
                                float min_rho = std::min(
                                    *(ptr_img_rho + i_minus_ncols + j),
                                    *(ptr_img_rho + i_plus_ncols + n_col + j));
                                *(ptr_img_rho_new + i_ncols + j) = min_rho;
                                *(ptr_img_rho_new + i_plus_ncols + j) = min_rho;
                            } else {
                                float max_rho = std::max(
                                    *(ptr_img_rho + i_minus_ncols + j),
                                    *(ptr_img_rho + i_plus_ncols + n_col + j));
                                *(ptr_img_rho_new + i_ncols + j) = max_rho;
                                *(ptr_img_rho_new + i_plus_ncols + j) = max_rho;
                            }
                        }
                    } else {
                    }
                }
            }  // end if
            else {
            }

            if (*(ptr_img_rho + i_ncols + (j - 1)) != 0) {
                if ((*(ptr_img_rho + i_ncols + (j + 1)) != 0)) {
                    if (fabsf32(*(ptr_img_rho + i_ncols + (j - 1)) -
                                *(ptr_img_rho + i_ncols + (j + 1))) < 0.05) {
                        *(ptr_img_restore_mask + i_ncols + j) = 4;
                        *(ptr_img_rho_new + i_ncols + j) =
                            (*(ptr_img_rho + i_ncols + (j - 1)) +
                             *(ptr_img_rho + i_ncols + (j + 1))) *
                            0.5;
                    } else {
                    }
                } else if ((*(ptr_img_rho + i_ncols + (j + 2)) != 0)) {
                    if (fabsf32(*(ptr_img_rho + i_ncols + (j - 1)) -
                                *(ptr_img_rho + i_ncols + (j + 2))) < 0.05) {
                        *(ptr_img_restore_mask + i_ncols + j) = 5;
                        *(ptr_img_restore_mask + i_ncols + (j + 1)) = 6;
                        *(ptr_img_rho_new + i_ncols + j) =
                            *(ptr_img_rho + i_ncols + (j - 1)) * (0.6666667) +
                            *(ptr_img_rho + i_ncols + (j + 2)) * (0.3333333);
                        *(ptr_img_rho_new + i_ncols + (j + 1)) =
                            *(ptr_img_rho + i_ncols + (j - 1)) * (0.3333333) +
                            *(ptr_img_rho + i_ncols + (j + 2)) * (0.6666667);
                    } else {
                    }
                } else {
                }
            } else {
            }

        }  // end col

    }  // end row

    img_rho_new.copyTo(str_rhopts_->img_rho);
}

void CloudFrame::interpRangeImage_dR(bool cur_next) {
    int n_col = str_rhopts_->img_rho.cols;
    int n_row = str_rhopts_->img_rho.rows;

    cv::Mat img_rho_new = str_rhopts_->img_rho.clone();
    float* ptr_img_rho_new = img_rho_new.ptr<float>(0);

    float* ptr_img_rho = str_rhopts_->img_rho.ptr<float>(0);
    int i_ncols = 0;
    int i_minus_ncols = 0;
    int i_plus_ncols = 0;

    int n_horizontal_minus_2 = (n_horizontal_ - 2);

    for (int i = 23; i < 36; ++i)
    // for (int i = 35; i > 22; --i)
    {
        i_ncols = i * n_col;
        i_minus_ncols = i_ncols - n_col;  // i_minus_ncols = (i - 1) * n_col;
        i_plus_ncols = i_ncols + n_col;   // i_plus_ncols = (i + 1) * n_col;

        for (int j = 2; j < n_horizontal_minus_2; ++j) {
            if (*(ptr_img_rho + i_ncols + j) == 0) {
                if ((*(ptr_img_rho + i_minus_ncols + j) != 0)) {
                    if ((*(ptr_img_rho + i_plus_ncols + j) != 0)) {
                        if (fabsf32(*(ptr_img_rho + i_minus_ncols + j) -
                                    *(ptr_img_rho + i_plus_ncols + j)) < 0.1) {
                            *(ptr_img_rho_new + i_ncols + j) =
                                (*(ptr_img_rho + i_minus_ncols + j) +
                                 *(ptr_img_rho + i_plus_ncols + j)) *
                                0.5;
                        } else {
                            if (cur_next == false) {
                                *(ptr_img_rho_new + i_ncols + j) =
                                    std::min(*(ptr_img_rho + i_minus_ncols + j),
                                             *(ptr_img_rho + i_plus_ncols + j));
                            } else {
                                *(ptr_img_rho_new + i_ncols + j) =
                                    std::max(*(ptr_img_rho + i_minus_ncols + j),
                                             *(ptr_img_rho + i_plus_ncols + j));
                            }
                        }
                    } else if ((*(ptr_img_rho + (i + 2) * n_col + j) != 0)) {
                        if (fabsf32(*(ptr_img_rho + i_minus_ncols + j) -
                                    *(ptr_img_rho + i_plus_ncols + n_col + j)) <
                            0.1) {
                            *(ptr_img_rho_new + i_ncols + j) =
                                *(ptr_img_rho + i_minus_ncols + j) *
                                    (0.6666667) +
                                *(ptr_img_rho + i_plus_ncols + n_col + j) *
                                    (0.3333333);
                            *(ptr_img_rho_new + i_plus_ncols + j) =
                                *(ptr_img_rho + i_minus_ncols + j) *
                                    (0.3333333) +
                                *(ptr_img_rho + i_plus_ncols + n_col + j) *
                                    (0.6666667);
                        } else {
                            if (cur_next == false) {
                                *(ptr_img_rho_new + i_ncols + j) = std::min(
                                    *(ptr_img_rho + i_minus_ncols + j),
                                    *(ptr_img_rho + i_plus_ncols + n_col + j));
                                *(ptr_img_rho_new + i_plus_ncols + j) =
                                    *(ptr_img_rho_new + i_ncols + j);
                            } else {
                                *(ptr_img_rho_new + i_ncols + j) = std::max(
                                    *(ptr_img_rho + i_minus_ncols + j),
                                    *(ptr_img_rho + i_plus_ncols + n_col + j));
                                *(ptr_img_rho_new + i_plus_ncols + j) =
                                    *(ptr_img_rho_new + i_ncols + j);
                            }
                        }
                    } else {
                    }
                }
            }  // end if
            else {
            }

            if (*(ptr_img_rho + i_ncols + (j - 1)) != 0) {
                if ((*(ptr_img_rho + i_ncols + (j + 1)) != 0)) {
                    if (fabsf32(*(ptr_img_rho + i_ncols + (j - 1)) -
                                *(ptr_img_rho + i_ncols + (j + 1))) < 0.05) {
                        *(ptr_img_rho_new + i_ncols + j) =
                            (*(ptr_img_rho + i_ncols + (j - 1)) +
                             *(ptr_img_rho + i_ncols + (j + 1))) *
                            0.5;
                    } else {
                    }
                } else if ((*(ptr_img_rho + i_ncols + (j + 2)) != 0)) {
                    if (fabsf32(*(ptr_img_rho + i_ncols + (j - 1)) -
                                *(ptr_img_rho + i_ncols + (j + 2))) < 0.05) {
                        *(ptr_img_rho_new + i_ncols + j) =
                            *(ptr_img_rho + i_ncols + (j - 1)) * (0.6666667) +
                            *(ptr_img_rho + i_ncols + (j + 2)) * (0.3333333);
                        *(ptr_img_rho_new + i_ncols + (j + 1)) =
                            *(ptr_img_rho + i_ncols + (j - 1)) * (0.3333333) +
                            *(ptr_img_rho + i_ncols + (j + 2)) * (0.6666667);
                    } else {
                    }
                } else {
                }
            } else {
            }

        }  // end col

    }  // end row

    img_rho_new.copyTo(str_rhopts_->img_rho);
}

void CloudFrame::interpPts(PointCloudwithTime::Ptr pcl_in, bool cur_next) {
    int n_row = str_rhopts_->img_rho.rows;
    int n_col = str_rhopts_->img_rho.cols;

    float* ptr_img_rho = str_rhopts_->img_rho.ptr<float>(0);
    int* ptr_img_index = str_rhopts_->img_index.ptr<int>(0);
    float* ptr_img_x = str_rhopts_->img_x.ptr<float>(0);
    float* ptr_img_y = str_rhopts_->img_y.ptr<float>(0);
    float* ptr_img_z = str_rhopts_->img_z.ptr<float>(0);
    int* ptr_img_restore_mask = str_rhopts_->img_restore_mask.ptr<int>(0);

    for (int i = 0; i < n_vertical_; ++i) {
        int i_ncols = i * n_col;
        for (int j = 0; j < n_horizontal_; ++j) {
            int i_ncols_j = i_ncols + j;
            if (str_rhopts_->pts_per_pixel_rho[i_ncols_j].size() > 0) {
                for (int k = 0;
                     k < (str_rhopts_->pts_per_pixel_rho[i_ncols_j].size());
                     ++k) {
                    if (std::abs((str_rhopts_->pts_per_pixel_rho[i_ncols_j][k] -
                                  *(ptr_img_rho + i_ncols_j))) < 2.0) {
                        str_rhopts_->pts_per_pixel_index_valid[i_ncols_j]
                            .push_back(
                                str_rhopts_->pts_per_pixel_index[i_ncols_j][k]);
                    } else {
                    }
                }
            }  // end if
            else {
            }

            int* ptr_img_index_i_ncols_j = ptr_img_index + i_ncols_j;
            int* ptr_img_index_i_ncols_j_p_n_col =
                ptr_img_index_i_ncols_j + n_col;
            int* ptr_img_index_i_ncols_j_m_n_col =
                ptr_img_index_i_ncols_j - n_col;

            if (*(ptr_img_index_i_ncols_j) != 0) {
                *(ptr_img_x + i_ncols_j) =
                    (*pcl_in)[*(ptr_img_index_i_ncols_j)].x;
                *(ptr_img_y + i_ncols_j) =
                    (*pcl_in)[*(ptr_img_index_i_ncols_j)].y;
                *(ptr_img_z + i_ncols_j) =
                    (*pcl_in)[*(ptr_img_index_i_ncols_j)].z;
            } else {
            }

            switch (*(ptr_img_restore_mask + i_ncols_j)) {
                case 1:
                    *(ptr_img_x + i_ncols_j) =
                        0.5 * ((*pcl_in)[*(ptr_img_index_i_ncols_j_m_n_col)].x +
                               (*pcl_in)[*(ptr_img_index_i_ncols_j_p_n_col)].x);
                    *(ptr_img_y + i_ncols_j) =
                        0.5 * ((*pcl_in)[*(ptr_img_index_i_ncols_j_m_n_col)].y +
                               (*pcl_in)[*(ptr_img_index_i_ncols_j_p_n_col)].y);
                    *(ptr_img_z + i_ncols_j) =
                        0.5 * ((*pcl_in)[*(ptr_img_index_i_ncols_j_m_n_col)].z +
                               (*pcl_in)[*(ptr_img_index_i_ncols_j_p_n_col)].z);
                    break;
                case 10:
                    if (cur_next == false) {
                        if ((*(ptr_img_rho + i_ncols_j - n_col) >
                             *(ptr_img_rho + i_ncols_j + n_col))) {
                            *(ptr_img_x + i_ncols_j) =
                                (*pcl_in)[*(ptr_img_index_i_ncols_j_p_n_col)].x;
                            *(ptr_img_y + i_ncols_j) =
                                (*pcl_in)[*(ptr_img_index_i_ncols_j_p_n_col)].y;
                            *(ptr_img_z + i_ncols_j) =
                                (*pcl_in)[*(ptr_img_index_i_ncols_j_p_n_col)].z;
                        } else {
                            *(ptr_img_x + i_ncols_j) =
                                (*pcl_in)[*(ptr_img_index_i_ncols_j_m_n_col)].x;
                            *(ptr_img_y + i_ncols_j) =
                                (*pcl_in)[*(ptr_img_index_i_ncols_j_m_n_col)].y;
                            *(ptr_img_z + i_ncols_j) =
                                (*pcl_in)[*(ptr_img_index_i_ncols_j_m_n_col)].z;
                        }
                        break;
                    } else {
                        if ((*(ptr_img_rho + i_ncols_j - n_col) <
                             *(ptr_img_rho + i_ncols_j + n_col))) {
                            *(ptr_img_x + i_ncols_j) =
                                (*pcl_in)[*(ptr_img_index_i_ncols_j_p_n_col)].x;
                            *(ptr_img_y + i_ncols_j) =
                                (*pcl_in)[*(ptr_img_index_i_ncols_j_p_n_col)].y;
                            *(ptr_img_z + i_ncols_j) =
                                (*pcl_in)[*(ptr_img_index_i_ncols_j_p_n_col)].z;
                        } else {
                            *(ptr_img_x + i_ncols_j) =
                                (*pcl_in)[*(ptr_img_index_i_ncols_j_m_n_col)].x;
                            *(ptr_img_y + i_ncols_j) =
                                (*pcl_in)[*(ptr_img_index_i_ncols_j_m_n_col)].y;
                            *(ptr_img_z + i_ncols_j) =
                                (*pcl_in)[*(ptr_img_index_i_ncols_j_m_n_col)].z;
                        }
                        break;
                    }

                case 2:
                    *(ptr_img_x + i_ncols_j) =
                        (0.6666667) *
                            (*pcl_in)[*(ptr_img_index_i_ncols_j_m_n_col)].x +
                        (0.3333333) *
                            (*pcl_in)[*(ptr_img_index_i_ncols_j_p_n_col +
                                        n_col)]
                                .x;
                    *(ptr_img_y + i_ncols_j) =
                        (0.6666667) *
                            (*pcl_in)[*(ptr_img_index_i_ncols_j_m_n_col)].y +
                        (0.3333333) *
                            (*pcl_in)[*(ptr_img_index_i_ncols_j_p_n_col +
                                        n_col)]
                                .y;
                    *(ptr_img_z + i_ncols_j) =
                        (0.6666667) *
                            (*pcl_in)[*(ptr_img_index_i_ncols_j_m_n_col)].z +
                        (0.3333333) *
                            (*pcl_in)[*(ptr_img_index_i_ncols_j_p_n_col +
                                        n_col)]
                                .z;
                    break;
                case 20:
                    if (cur_next == false) {
                        if ((*(ptr_img_rho + i_ncols_j - n_col) >
                             *(ptr_img_rho + i_ncols_j + 2 * n_col))) {
                            *(ptr_img_x + i_ncols_j) =
                                (*pcl_in)[*(ptr_img_index_i_ncols_j_p_n_col +
                                            n_col)]
                                    .x;
                            *(ptr_img_y + i_ncols_j) =
                                (*pcl_in)[*(ptr_img_index_i_ncols_j_p_n_col +
                                            n_col)]
                                    .y;
                            *(ptr_img_z + i_ncols_j) =
                                (*pcl_in)[*(ptr_img_index_i_ncols_j_p_n_col +
                                            n_col)]
                                    .z;
                        } else {
                            *(ptr_img_x + i_ncols_j) =
                                (*pcl_in)[*(ptr_img_index_i_ncols_j_m_n_col)].x;
                            *(ptr_img_y + i_ncols_j) =
                                (*pcl_in)[*(ptr_img_index_i_ncols_j_m_n_col)].y;
                            *(ptr_img_z + i_ncols_j) =
                                (*pcl_in)[*(ptr_img_index_i_ncols_j_m_n_col)].z;
                        }
                        break;
                    } else {
                        if ((*(ptr_img_rho + i_ncols_j - n_col) <
                             *(ptr_img_rho + i_ncols_j + 2 * n_col))) {
                            *(ptr_img_x + i_ncols_j) =
                                (*pcl_in)[*(ptr_img_index_i_ncols_j_p_n_col +
                                            n_col)]
                                    .x;
                            *(ptr_img_y + i_ncols_j) =
                                (*pcl_in)[*(ptr_img_index_i_ncols_j_p_n_col +
                                            n_col)]
                                    .y;
                            *(ptr_img_z + i_ncols_j) =
                                (*pcl_in)[*(ptr_img_index_i_ncols_j_p_n_col +
                                            n_col)]
                                    .z;
                        } else {
                            *(ptr_img_x + i_ncols_j) =
                                (*pcl_in)[*(ptr_img_index_i_ncols_j_m_n_col)].x;
                            *(ptr_img_y + i_ncols_j) =
                                (*pcl_in)[*(ptr_img_index_i_ncols_j_m_n_col)].y;
                            *(ptr_img_z + i_ncols_j) =
                                (*pcl_in)[*(ptr_img_index_i_ncols_j_m_n_col)].z;
                        }
                        break;
                    }

                case 3:
                    *(ptr_img_x + i_ncols_j) =
                        (0.3333333) *
                            (*pcl_in)[*(ptr_img_index_i_ncols_j_m_n_col -
                                        n_col)]
                                .x +
                        (0.6666667) *
                            (*pcl_in)[*(ptr_img_index_i_ncols_j_p_n_col)].x;
                    *(ptr_img_y + i_ncols_j) =
                        (0.3333333) *
                            (*pcl_in)[*(ptr_img_index_i_ncols_j_m_n_col -
                                        n_col)]
                                .y +
                        (0.6666667) *
                            (*pcl_in)[*(ptr_img_index_i_ncols_j_p_n_col)].y;
                    *(ptr_img_z + i_ncols_j) =
                        (0.3333333) *
                            (*pcl_in)[*(ptr_img_index_i_ncols_j_m_n_col -
                                        n_col)]
                                .z +
                        (0.6666667) *
                            (*pcl_in)[*(ptr_img_index_i_ncols_j_p_n_col)].z;
                    break;
                case 30:
                    if (cur_next == false) {
                        if ((*(ptr_img_rho + i_ncols_j - 2 * n_col) >
                             *(ptr_img_rho + i_ncols_j + n_col))) {
                            *(ptr_img_x + i_ncols_j) =
                                (*pcl_in)[*(ptr_img_index_i_ncols_j_p_n_col)].x;
                            *(ptr_img_y + i_ncols_j) =
                                (*pcl_in)[*(ptr_img_index_i_ncols_j_p_n_col)].y;
                            *(ptr_img_z + i_ncols_j) =
                                (*pcl_in)[*(ptr_img_index_i_ncols_j_p_n_col)].z;
                        } else {
                            *(ptr_img_x + i_ncols_j) =
                                (*pcl_in)[*(ptr_img_index_i_ncols_j_m_n_col -
                                            n_col)]
                                    .x;
                            *(ptr_img_y + i_ncols_j) =
                                (*pcl_in)[*(ptr_img_index_i_ncols_j_m_n_col -
                                            n_col)]
                                    .y;
                            *(ptr_img_z + i_ncols_j) =
                                (*pcl_in)[*(ptr_img_index_i_ncols_j_m_n_col -
                                            n_col)]
                                    .z;
                        }
                        break;
                    } else {
                        if ((*(ptr_img_rho + i_ncols_j - 2 * n_col) <
                             *(ptr_img_rho + i_ncols_j + n_col))) {
                            *(ptr_img_x + i_ncols_j) =
                                (*pcl_in)[*(ptr_img_index_i_ncols_j_p_n_col)].x;
                            *(ptr_img_y + i_ncols_j) =
                                (*pcl_in)[*(ptr_img_index_i_ncols_j_p_n_col)].y;
                            *(ptr_img_z + i_ncols_j) =
                                (*pcl_in)[*(ptr_img_index_i_ncols_j_p_n_col)].z;
                        } else {
                            *(ptr_img_x + i_ncols_j) =
                                (*pcl_in)[*(ptr_img_index_i_ncols_j_m_n_col -
                                            n_col)]
                                    .x;
                            *(ptr_img_y + i_ncols_j) =
                                (*pcl_in)[*(ptr_img_index_i_ncols_j_m_n_col -
                                            n_col)]
                                    .y;
                            *(ptr_img_z + i_ncols_j) =
                                (*pcl_in)[*(ptr_img_index_i_ncols_j_m_n_col -
                                            n_col)]
                                    .z;
                        }
                        break;
                    }

                case 4:
                    *(ptr_img_x + i_ncols_j) =
                        0.5 * ((*pcl_in)[*(ptr_img_index_i_ncols_j - 1)].x +
                               (*pcl_in)[*(ptr_img_index_i_ncols_j + 1)].x);
                    *(ptr_img_y + i_ncols_j) =
                        0.5 * ((*pcl_in)[*(ptr_img_index_i_ncols_j - 1)].y +
                               (*pcl_in)[*(ptr_img_index_i_ncols_j + 1)].y);
                    *(ptr_img_z + i_ncols_j) =
                        0.5 * ((*pcl_in)[*(ptr_img_index_i_ncols_j - 1)].z +
                               (*pcl_in)[*(ptr_img_index_i_ncols_j + 1)].z);
                    break;
                case 5:
                    *(ptr_img_x + i_ncols_j) =
                        (0.6666667) *
                            (*pcl_in)[*(ptr_img_index_i_ncols_j - 1)].x +
                        (0.3333333) *
                            (*pcl_in)[*(ptr_img_index_i_ncols_j + 2)].x;
                    *(ptr_img_y + i_ncols_j) =
                        (0.6666667) *
                            (*pcl_in)[*(ptr_img_index_i_ncols_j - 1)].y +
                        (0.3333333) *
                            (*pcl_in)[*(ptr_img_index_i_ncols_j + 2)].y;
                    *(ptr_img_z + i_ncols_j) =
                        (0.6666667) *
                            (*pcl_in)[*(ptr_img_index_i_ncols_j - 1)].z +
                        (0.3333333) *
                            (*pcl_in)[*(ptr_img_index_i_ncols_j + 2)].z;
                    break;
                case 6:
                    *(ptr_img_x + i_ncols_j) =
                        (0.3333333) *
                            (*pcl_in)[*(ptr_img_index_i_ncols_j - 2)].x +
                        (0.6666667) *
                            (*pcl_in)[*(ptr_img_index_i_ncols_j + 1)].x;
                    *(ptr_img_y + i_ncols_j) =
                        (0.3333333) *
                            (*pcl_in)[*(ptr_img_index_i_ncols_j - 2)].y +
                        (0.6666667) *
                            (*pcl_in)[*(ptr_img_index_i_ncols_j + 1)].y;
                    *(ptr_img_z + i_ncols_j) =
                        (0.3333333) *
                            (*pcl_in)[*(ptr_img_index_i_ncols_j - 2)].z +
                        (0.6666667) *
                            (*pcl_in)[*(ptr_img_index_i_ncols_j + 1)].z;
                    break;
            }
        }  // end for j
    }      // end for i
    // cv::FileStorage fs_w("/home/junhakim/img_x.yaml",
    // cv::FileStorage::WRITE); fs_w << "matImage" << str_rhopts_->img_x;
    // fs_w.release();
    // cv::FileStorage fs_s("/home/junhakim/img_y.yaml",
    // cv::FileStorage::WRITE); fs_s << "matImage" << str_rhopts_->img_y;
    // fs_s.release();
    // cv::FileStorage fs_q("/home/junhakim/img_z.yaml",
    // cv::FileStorage::WRITE); fs_q << "matImage" << str_rhopts_->img_z;
    // fs_q.release();
    // exit(0);
}

void CloudFrame::reset() {
    std::shared_ptr<StrRhoPts>& str_rhopts_tmp = this->str_rhopts_;
    str_rhopts_tmp->rho.resize(0);
    str_rhopts_tmp->phi.resize(0);
    str_rhopts_tmp->theta.resize(0);
    str_rhopts_tmp->img_rho = cv::Mat::zeros(img_height_, img_width_, CV_32FC1);
    str_rhopts_tmp->img_index =
        cv::Mat::zeros(img_height_, img_width_, CV_32SC1);
    str_rhopts_tmp->img_x = cv::Mat::zeros(img_height_, img_width_, CV_32FC1);
    str_rhopts_tmp->img_y = cv::Mat::zeros(img_height_, img_width_, CV_32FC1);
    str_rhopts_tmp->img_z = cv::Mat::zeros(img_height_, img_width_, CV_32FC1);
    str_rhopts_tmp->img_restore_mask =
        cv::Mat::zeros(img_height_, img_width_, CV_32SC1);
    str_rhopts_tmp->img_restore_warp_mask =
        cv::Mat::zeros(img_height_, img_width_, CV_32SC1);

    // for (int i = 0; i < img_height_ * img_width_; ++i)
    // {
    //     str_rhopts_tmp->pts_per_pixel_n[i] = 0;
    // }

    for (int i = 0; i < img_height_ * img_width_; ++i) {
        if (str_rhopts_tmp->pts_per_pixel_index[i].size() != 0) {
            str_rhopts_tmp->pts_per_pixel_index[i].resize(0);
        }

        if (str_rhopts_tmp->pts_per_pixel_rho[i].size() != 0) {
            str_rhopts_tmp->pts_per_pixel_rho[i].resize(0);
        }

        if (str_rhopts_tmp->pts_per_pixel_index_valid[i].size() != 0) {
            str_rhopts_tmp->pts_per_pixel_index_valid[i].resize(0);
        }
    }
    // for (int i = 0; i < img_height_ * img_width_; ++i)
    // {
    //     if (str_rhopts_tmp->pts_per_pixel_index[i].size() != 0)
    //     {
    //         str_rhopts_tmp->pts_per_pixel_index[i].resize(0);
    //     }
    // }

    // for (int i = 0; i < img_height_ * img_width_; ++i)
    // {
    //     if (str_rhopts_tmp->pts_per_pixel_rho[i].size() != 0)
    //     {
    //         str_rhopts_tmp->pts_per_pixel_rho[i].resize(0);
    //     }
    // }

    // for (int i = 0; i < img_height_ * img_width_; ++i)
    // {
    //     if (str_rhopts_tmp->pts_per_pixel_index_valid[i].size() != 0)
    //     {
    //         str_rhopts_tmp->pts_per_pixel_index_valid[i].resize(0);
    //     }
    // }
}