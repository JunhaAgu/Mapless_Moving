#include "dR_calc.h"

dRCalc::dRCalc(const std::unique_ptr<UserParam>& user_param)
{
    for (int i=0; i<user_param->sensor_spec_.v_angle_.size(); ++i)
    {
        v_angle_.push_back(user_param->sensor_spec_.v_angle_[i]);
    }
    n_ring_     = user_param->image_param_.height_;
    n_radial_   = user_param->image_param_.width_;

    velo_cur_       = boost::make_shared<pcl::PointCloud<pcl::PointXYZ>>();
    cur_pts_warped_ = boost::make_shared<pcl::PointCloud<pcl::PointXYZ>>();
};

dRCalc::~dRCalc()
{

};

void dRCalc::dR_warpPointcloud(std::unique_ptr<CloudFrame>& CloudFrame_next, std::unique_ptr<CloudFrame>& CloudFrame_cur, std::unique_ptr<CloudFrame>& CloudFrame_cur_warped,
                                pcl::PointCloud<pcl::PointXYZ>::Ptr p0, Pose& T01, int cnt_data, cv::Mat& dRdt)
{
    int n_row = CloudFrame_next->str_rhopts_->img_rho.rows;
    int n_col = CloudFrame_next->str_rhopts_->img_rho.cols; 

    float* ptr_cur_img_x = CloudFrame_cur->str_rhopts_->img_x.ptr<float>(0);
    float* ptr_cur_img_y = CloudFrame_cur->str_rhopts_->img_y.ptr<float>(0);
    float* ptr_cur_img_z = CloudFrame_cur->str_rhopts_->img_z.ptr<float>(0);
    // int cnt = 0;

    // representative pts
    for (int i = 0; i < n_row; ++i)
    {
        int i_ncols = i * n_col;
        for (int j = 0; j < n_col; ++j)
        {
            int i_ncols_j = i_ncols + j;
            if ((*(ptr_cur_img_x + i_ncols_j) > 10.0) || (*(ptr_cur_img_x + i_ncols_j) < -10.0) || (*(ptr_cur_img_y + i_ncols_j) > 10.0) || (*(ptr_cur_img_y + i_ncols_j) < -10.0))
            {
                velo_cur_->push_back(pcl::PointXYZ(*(ptr_cur_img_x + i_ncols_j), *(ptr_cur_img_y + i_ncols_j), *(ptr_cur_img_z + i_ncols_j)));
            }
        }
    }
    
    // Far pts are warped by the original pts
    for (int i = 0; i < p0->size(); ++i)
    {
        if( ((*p0)[i].x <= 10) && ((*p0)[i].x >= -10.0) && ((*p0)[i].y <= 10.0) && ((*p0)[i].y >= -10.0) )
        {
            velo_cur_->push_back((*p0)[i]);
        }
    }

    // compensate zero in current rho image for warping
    compensateCurRhoZeroWarp(CloudFrame_cur);

    pcl::transformPointCloud(*velo_cur_, *cur_pts_warped_, T01);
    
    // current warped image
    // CloudFrame_cur_warped->genRangeImages(cur_pts_warped_, false);
    CloudFrame_cur_warped->genRangeImages_dR(cur_pts_warped_, false);

    // fill range image using interpolation
    interpRangeImageMin(CloudFrame_cur_warped);

    // fill pts corresponding to filled range image (no affect the original pts)
    interpPtsWarp(CloudFrame_cur_warped);
    
    // calculate occlusions
    cv::subtract(CloudFrame_cur_warped->str_rhopts_->img_rho, CloudFrame_next->str_rhopts_->img_rho, dRdt); 

    // cv::FileStorage fs_w("/home/junhakim/dRdt.yaml", cv::FileStorage::WRITE);
    // fs_w << "matImage" << dRdt;
    // fs_w.release();
    // exit(0);
}

void dRCalc::compensateCurRhoZeroWarp(std::unique_ptr<CloudFrame>& CloudFrame_cur)
{
    int n_row = n_ring_;
    int n_col = n_radial_;
    float left_dir_rho  = 0;
    float right_dir_rho = 0;
    float up_dir_rho    = 0;
    float down_dir_rho  = 0;
    float* ptr_cur_img_rho = CloudFrame_cur->str_rhopts_->img_rho.ptr<float>(0);

    int cnt_left    = 1;
    int cnt_right   = 1;
    int cnt_up      = 1;
    int cnt_down    = 1;

    float min_rho_4_dir = 0.0 ;
    
    std::vector<float> four_dir;
    four_dir.resize(4);
    std::vector<int> four_cnt;
    four_cnt.resize(4);

    int valid_dir[4];

    float new_phi = 0.0;
    float new_theta = 0.0;
    
    int valid_dir_sum = 0;

    std::vector<float> dir_tmp;
    dir_tmp.reserve(4);

    float min_dir = 0.0;

    float inv_left = 0.0;
    float inv_right = 0.0;
    float inv_up = 0.0;
    float inv_down = 0.0;

    float vec_inv_sum = 0.0;

    std::vector<float> vec_inv;
    vec_inv.reserve(4);

    // cv::FileStorage fs_w("/home/junhakim/img_rho.yaml", cv::FileStorage::WRITE);
    // fs_w << "matImage" << CloudFrame_cur->str_rhopts_->img_rho;
    // fs_w.release();
    // exit(0);
    float new_phi_alpha;
    float new_theta_alpha;
    float min_rho_cos;

    for (int i = 0 + 1; i < n_ring_ - 1; ++i)
    {
        int i_ncols = i * n_col;
        for (int j = 0 + 1; j < n_radial_ - 1; ++j)
        {
            // initialization every iteration
            left_dir_rho = 0.0;
            right_dir_rho = 0.0;
            up_dir_rho = 0.0;
            down_dir_rho = 0.0;

            valid_dir[0] = 0; valid_dir[1] = 0; valid_dir[2] = 0; valid_dir[3] = 0;
            valid_dir_sum = 0;
            dir_tmp.resize(0);
            vec_inv_sum = 0.0;
            min_rho_4_dir = 0.0;
            int i_ncols_j = i_ncols + j;
            if (*(ptr_cur_img_rho + i_ncols_j) == 0)
            {
                    cnt_left = 1;
                    cnt_right = 1;
                    cnt_up = 1;
                    cnt_down = 1;
                    //left
                    while (left_dir_rho == 0.0)
                    {
                        if ((j - cnt_left) < 0)
                        {
                            // left_dir_rho = 100.0;
                            break;
                        }
                        left_dir_rho = *(ptr_cur_img_rho + i_ncols_j - cnt_left);
                        cnt_left += 1;
                    } // end while
                    //right
                    while (right_dir_rho == 0.0)
                    {
                        if ((j + cnt_right) > n_col-1)
                        {
                            // right_dir_rho = 100.0;
                            break;
                        }
                        right_dir_rho = *(ptr_cur_img_rho + i_ncols_j + cnt_right);
                        cnt_right += 1;
                    } // end while
                    //up
                    while (up_dir_rho == 0.0)
                    {
                        if ((i - cnt_up) < 0)
                        {
                            // up_dir_rho = 100.0;
                            break;
                        }
                        up_dir_rho = *(ptr_cur_img_rho + i_ncols_j - (cnt_up * n_col)); //(i - cnt_up) * n_col + j);
                        cnt_up += 1;
                    } // end while
                    //down
                    while (down_dir_rho == 0.0)
                    {
                        if ((i + cnt_down) > n_row-1)
                        {
                            // down_dir_rho = 100.0;
                            break;
                        }
                        down_dir_rho = *(ptr_cur_img_rho + i_ncols_j + (cnt_down * n_col)); //(i + cnt_down) * n_col + j);
                        cnt_down += 1;
                    } // end while
                    four_dir[0] = (left_dir_rho);
                    four_dir[1] = (right_dir_rho);
                    four_dir[2] = (up_dir_rho);
                    four_dir[3] = (down_dir_rho);
                    four_cnt[0] = (cnt_left);
                    four_cnt[1] = (cnt_right);
                    four_cnt[2] = (cnt_up);
                    four_cnt[3] = (cnt_down);
                    for (int i_v=0; i_v<4; ++i_v)
                    {
                        if (four_dir[i_v]!=0 && four_dir[i_v]<100 && four_cnt[i_v]<20)
                        {
                            valid_dir[i_v] = 1;
                            dir_tmp.push_back(four_dir[i_v]);
                        }
                    }
                    valid_dir_sum = valid_dir[0] + valid_dir[1] + valid_dir[2] + valid_dir[3];
                    if (valid_dir_sum < 1)
                    {
                        continue;
                    }
                    
                    min_dir = *min_element(dir_tmp.begin(), dir_tmp.end());
                    for (int i_v=0; i_v<4; ++i_v)
                    {
                        if (valid_dir[i_v] == 1 && four_dir[i_v]<(min_dir+1.0f))
                        {}
                        else
                        {
                            valid_dir[i_v] = 0;
                        }
                    }

                    valid_dir_sum = valid_dir[0] + valid_dir[1] + valid_dir[2] + valid_dir[3];
                    if (valid_dir_sum < 1)
                    {
                        continue;
                    }
                    else if (valid_dir_sum == 1)
                    {
                        min_rho_4_dir = min_dir;
                    }
                    else
                    {
                        vec_inv[0] = 1.0/(float)cnt_left    * (float)valid_dir[0];
                        vec_inv[1] = 1.0/(float)cnt_right   * (float)valid_dir[1];
                        vec_inv[2] = 1.0/(float)cnt_up      * (float)valid_dir[2];
                        vec_inv[3] = 1.0/(float)cnt_down    * (float)valid_dir[3];

                        vec_inv_sum = vec_inv[0] + vec_inv[1] + vec_inv[2] + vec_inv[3];

                        min_rho_4_dir = (four_dir[0]*vec_inv[0] + four_dir[1]*vec_inv[1] + four_dir[2]*vec_inv[2] + four_dir[3]*vec_inv[3]) / vec_inv_sum;
                    }
            // std::cout << min_rho_4_dir << "----" << vec_inv[0] << " " << vec_inv[1] << " " << vec_inv[2] << " " << vec_inv[3] << std::endl;
                    if (min_rho_4_dir > 0.0)
                    {
                        new_phi = v_angle_[i] * D2R;
                        new_theta = 0.4 * (j + 1) * D2R;
                        for (int m = 0; m < 5; ++m)
                        {
                            new_phi_alpha = new_phi + ((float)m - 2.0) * 0.2 * D2R;
                            min_rho_cos = -min_rho_4_dir * cosf(new_phi_alpha);
                            for (int p = 0; p < 5; ++p)
                            {
                                new_theta_alpha = new_theta + ((float)p - 2.0) * 0.08 * D2R;
                                velo_cur_->push_back(pcl::PointXYZ(min_rho_cos * cosf(new_theta_alpha),
                                                                 min_rho_cos * sinf(new_theta_alpha),
                                                                 min_rho_4_dir * sinf(new_phi_alpha)));
                            }
                        }
                    } // end if
                        else{}
            } // end if
            else{}
        } // end for j
    } //end for i
}

void dRCalc::interpRangeImageMin(std::unique_ptr<CloudFrame>& CloudFrame_in)
{
    cv::Mat img_rho_new = CloudFrame_in->str_rhopts_->img_rho.clone();
    float *ptr_img_rho  = CloudFrame_in->str_rhopts_->img_rho.ptr<float>(0);
    float *ptr_img_rho_new = img_rho_new.ptr<float>(0);
    int *ptr_img_restore_warp_mask = CloudFrame_in->str_rhopts_->img_restore_warp_mask.ptr<int>(0);
    int n_col = CloudFrame_in->str_rhopts_->img_rho.cols;
    int n_row = CloudFrame_in->str_rhopts_->img_rho.rows;

    int n_col_2 = 2*n_col;
    int i_ncols = 0;
    int i_ncols_j = 0;

    for (int i = 0 + 2; i < (n_ring_ - 2); ++i)
    {
        i_ncols = i * n_col;
        for (int j = 0 + 2; j < (n_radial_ - 2); ++j)
        {
            i_ncols_j = i_ncols + j;            
            if (*(ptr_img_rho + i_ncols_j) == 0)
            {
                if (*(ptr_img_rho + i_ncols_j - n_col) != 0 && *(ptr_img_rho + i_ncols_j + n_col) != 0)
                {
                    if (fabsf32(*(ptr_img_rho + i_ncols_j - n_col) - *(ptr_img_rho + i_ncols_j + n_col)) < 0.1)
                    {
                        *(ptr_img_restore_warp_mask + i_ncols_j) = 1;
                        *(ptr_img_rho_new + i_ncols_j) = (*(ptr_img_rho + i_ncols_j - n_col) + *(ptr_img_rho + i_ncols_j + n_col)) * 0.5;
                    }
                    else
                    {
                        *(ptr_img_restore_warp_mask + i_ncols_j) = 10;
                        *(ptr_img_rho_new + i_ncols_j) = std::min(*(ptr_img_rho + i_ncols_j - n_col), *(ptr_img_rho + i_ncols_j + n_col));
                    }
                }
                else if (*(ptr_img_rho + i_ncols_j - n_col) != 0 && *(ptr_img_rho + i_ncols_j + n_col_2) != 0)
                {
                    if (fabsf32(*(ptr_img_rho + i_ncols_j - n_col) - *(ptr_img_rho + i_ncols_j + n_col_2)) < 0.1)
                    {
                        *(ptr_img_restore_warp_mask + i_ncols_j)          = 2;
                        *(ptr_img_restore_warp_mask + i_ncols_j + n_col)  = 3;
                        *(ptr_img_rho_new + i_ncols_j)            = *(ptr_img_rho + i_ncols_j - n_col) * (0.6666667) + *(ptr_img_rho + i_ncols_j + n_col_2) * (0.3333333);
                        *(ptr_img_rho_new + i_ncols_j + n_col)    = *(ptr_img_rho + i_ncols_j - n_col) * (0.3333333) + *(ptr_img_rho + i_ncols_j + n_col_2) * (0.6666667);
                    }
                    else
                    {
                        *(ptr_img_restore_warp_mask + i_ncols_j)          = 20;
                        *(ptr_img_restore_warp_mask + i_ncols_j + n_col)  = 30;
                        *(ptr_img_rho_new + i_ncols_j)            = std::min(*(ptr_img_rho + i_ncols_j - n_col), *(ptr_img_rho + i_ncols_j + n_col_2));
                        *(ptr_img_rho_new + i_ncols_j + n_col)    = std::min(*(ptr_img_rho + i_ncols_j - n_col), *(ptr_img_rho + i_ncols_j + n_col_2));
                    }
                }

                if (*(ptr_img_rho + i_ncols_j - 1) != 0 && *(ptr_img_rho + i_ncols_j + 1) != 0)
                {
                    if (fabsf32(*(ptr_img_rho + i_ncols_j - 1) - *(ptr_img_rho + i_ncols_j + 1)) < 0.05)
                    {
                        *(ptr_img_restore_warp_mask + i_ncols_j) = 4;
                        *(ptr_img_rho_new + i_ncols_j) = (*(ptr_img_rho + i_ncols_j - 1) + *(ptr_img_rho + i_ncols_j + 1)) * 0.5;
                    }
                    else{}
                }
                else if (*(ptr_img_rho + i_ncols_j - 1) != 0 && *(ptr_img_rho + i_ncols_j + 2) != 0)
                {
                    if (fabsf32(*(ptr_img_rho + i_ncols_j - 1) - *(ptr_img_rho + i_ncols_j + 2)) < 0.05)
                    {
                        *(ptr_img_restore_warp_mask + i_ncols_j)          = 5;
                        *(ptr_img_restore_warp_mask + i_ncols_j + 1)    = 6;
                        *(ptr_img_rho_new + i_ncols_j)        = *(ptr_img_rho + i_ncols_j - 1) * (0.6666667) + *(ptr_img_rho + i_ncols_j + 2) * (0.3333333);
                        *(ptr_img_rho_new + i_ncols_j + 1)  = *(ptr_img_rho + i_ncols_j - 1) * (0.3333333) + *(ptr_img_rho + i_ncols_j + 2) * (0.6666667);
                    }
                    else{}
                }
            } // end if
            else{}
        } // end col

    } // end row

    img_rho_new.copyTo(CloudFrame_in->str_rhopts_->img_rho);
}

void dRCalc::interpPtsWarp(std::unique_ptr<CloudFrame>& CloudFrame_in)
{
    int n_row = n_ring_;
    int n_col = n_radial_;
    int n_col_2 = 2*n_col;
    float* ptr_img_rho  = CloudFrame_in->str_rhopts_->img_rho.ptr<float>(0);
    float* ptr_img_x    = CloudFrame_in->str_rhopts_->img_x.ptr<float>(0);
    float* ptr_img_y    = CloudFrame_in->str_rhopts_->img_y.ptr<float>(0);
    float* ptr_img_z    = CloudFrame_in->str_rhopts_->img_z.ptr<float>(0);
    int* ptr_img_restore_warp_mask = CloudFrame_in->str_rhopts_->img_restore_warp_mask.ptr<int>(0);

    int i_ncols = 0;
    int i_ncols_j = 0;

    for (int i = 0 + 2; i < n_row - 2; ++i)
    {
        i_ncols = i * n_col;
        for (int j = 0 + 2; j < n_col - 2; ++j)
        {         
            i_ncols_j = i_ncols + j;   
            switch(*(ptr_img_restore_warp_mask + i_ncols_j))
            {
            case 1:
                *(ptr_img_x + i_ncols_j) = 0.5 * (*(ptr_img_x + i_ncols_j - n_col) + *(ptr_img_x + i_ncols_j + n_col));
                *(ptr_img_y + i_ncols_j) = 0.5 * (*(ptr_img_y + i_ncols_j - n_col) + *(ptr_img_y + i_ncols_j + n_col));
                *(ptr_img_z + i_ncols_j) = 0.5 * (*(ptr_img_z + i_ncols_j - n_col) + *(ptr_img_z + i_ncols_j + n_col));
                break;
            case 10:
                if ((*(ptr_img_rho + i_ncols_j - n_col) < *(ptr_img_rho + i_ncols_j + n_col)))
                {
                    *(ptr_img_x + i_ncols_j) = *(ptr_img_x + i_ncols_j - n_col);
                    *(ptr_img_y + i_ncols_j) = *(ptr_img_y + i_ncols_j - n_col);
                    *(ptr_img_z + i_ncols_j) = *(ptr_img_z + i_ncols_j - n_col);
                }
                else
                {
                    *(ptr_img_x + i_ncols_j) = *(ptr_img_x + i_ncols_j + n_col);
                    *(ptr_img_y + i_ncols_j) = *(ptr_img_y + i_ncols_j + n_col);
                    *(ptr_img_z + i_ncols_j) = *(ptr_img_z + i_ncols_j + n_col);
                }
                break;
            case 2:
                *(ptr_img_x + i_ncols_j) = (0.6666667) * (*(ptr_img_x + i_ncols_j - n_col)) + (0.3333333) * (*(ptr_img_x + i_ncols_j + n_col_2));
                *(ptr_img_y + i_ncols_j) = (0.6666667) * (*(ptr_img_y + i_ncols_j - n_col)) + (0.3333333) * (*(ptr_img_y + i_ncols_j + n_col_2));
                *(ptr_img_z + i_ncols_j) = (0.6666667) * (*(ptr_img_z + i_ncols_j - n_col)) + (0.3333333) * (*(ptr_img_z + i_ncols_j + n_col_2));
                break;
            case 20:
                if ((*(ptr_img_rho + i_ncols_j - n_col) < *(ptr_img_rho + i_ncols_j + n_col_2)))
                {
                    *(ptr_img_x + i_ncols_j) = *(ptr_img_x + i_ncols_j - n_col);
                    *(ptr_img_y + i_ncols_j) = *(ptr_img_y + i_ncols_j - n_col);
                    *(ptr_img_z + i_ncols_j) = *(ptr_img_z + i_ncols_j - n_col);
                }
                else
                {
                    *(ptr_img_x + i_ncols_j) = *(ptr_img_x + i_ncols_j + n_col_2);
                    *(ptr_img_y + i_ncols_j) = *(ptr_img_y + i_ncols_j + n_col_2);
                    *(ptr_img_z + i_ncols_j) = *(ptr_img_z + i_ncols_j + n_col_2);
                }
                break;
            case 3:
                *(ptr_img_x + i_ncols_j) = (0.3333333) * (*(ptr_img_x + i_ncols_j - n_col_2)) + (0.6666667) * (*(ptr_img_x + i_ncols_j + n_col ));
                *(ptr_img_y + i_ncols_j) = (0.3333333) * (*(ptr_img_y + i_ncols_j - n_col_2)) + (0.6666667) * (*(ptr_img_y + i_ncols_j + n_col ));
                *(ptr_img_z + i_ncols_j) = (0.3333333) * (*(ptr_img_z + i_ncols_j - n_col_2)) + (0.6666667) * (*(ptr_img_z + i_ncols_j + n_col ));
                break;
            case 30:
                if ((*(ptr_img_rho + i_ncols_j - n_col_2) < *(ptr_img_rho + i_ncols_j + n_col)))
                {
                    *(ptr_img_x + i_ncols_j) = *(ptr_img_x + i_ncols_j - n_col_2);
                    *(ptr_img_y + i_ncols_j) = *(ptr_img_y + i_ncols_j - n_col_2);
                    *(ptr_img_z + i_ncols_j) = *(ptr_img_z + i_ncols_j - n_col_2);
                }
                else
                {
                    *(ptr_img_x + i_ncols_j) = *(ptr_img_x + i_ncols_j + n_col);
                    *(ptr_img_y + i_ncols_j) = *(ptr_img_y + i_ncols_j + n_col);
                    *(ptr_img_z + i_ncols_j) = *(ptr_img_z + i_ncols_j + n_col);
                }
                break;
            case 4:
                *(ptr_img_x + i_ncols_j) = 0.5 * (*(ptr_img_x + i_ncols_j - 1) + (*(ptr_img_x + i_ncols_j + 1)));
                *(ptr_img_y + i_ncols_j) = 0.5 * (*(ptr_img_y + i_ncols_j - 1) + (*(ptr_img_y + i_ncols_j + 1)));
                *(ptr_img_z + i_ncols_j) = 0.5 * (*(ptr_img_z + i_ncols_j - 1) + (*(ptr_img_z + i_ncols_j + 1)));
                break;
            case 5:
                *(ptr_img_x + i_ncols_j) = (0.6666667) * (*(ptr_img_x + i_ncols_j - 1)) + (0.3333333) * (*(ptr_img_x + i_ncols_j + 2));
                *(ptr_img_y + i_ncols_j) = (0.6666667) * (*(ptr_img_y + i_ncols_j - 1)) + (0.3333333) * (*(ptr_img_y + i_ncols_j + 2));
                *(ptr_img_z + i_ncols_j) = (0.6666667) * (*(ptr_img_z + i_ncols_j - 1)) + (0.3333333) * (*(ptr_img_z + i_ncols_j + 2));
                break;
            case 6:
                *(ptr_img_x + i_ncols_j) = (0.3333333) * (*(ptr_img_x + i_ncols_j - 2)) + (0.6666667) * (*(ptr_img_x + i_ncols_j + 1));
                *(ptr_img_y + i_ncols_j) = (0.3333333) * (*(ptr_img_y + i_ncols_j - 2)) + (0.6666667) * (*(ptr_img_y + i_ncols_j + 1));
                *(ptr_img_z + i_ncols_j) = (0.3333333) * (*(ptr_img_z + i_ncols_j - 2)) + (0.6666667) * (*(ptr_img_z + i_ncols_j + 1));
                break;
            }
        } // end for j
    } // end for i
}