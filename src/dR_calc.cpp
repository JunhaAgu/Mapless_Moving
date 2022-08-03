#include "dR_calc.h"

dRCalc::dRCalc(const std::unique_ptr<UserParam>& user_param)
{
    for (int i=0; i<user_param->sensor_spec_.v_angle_.size(); ++i)
    {
        v_angle_.push_back(user_param->sensor_spec_.v_angle_[i]);
    }
};

dRCalc::~dRCalc()
{

};

void dRCalc::dR_warpPointcloud(StrRhoPts* str_next, StrRhoPts* str_cur, pcl::PointCloud<pcl::PointXYZ>& p0, Pose& T01, int cnt_data, StrRhoPts* str_cur_warped, cv::Mat& dRdt,
                                 pcl::PointCloud<pcl::PointXYZ>& velo_cur, pcl::PointCloud<pcl::PointXYZ>::Ptr ptr_cur_pts_warped, std::unique_ptr<CloudFrame>& CloudFrame)
{
    int n_row = str_next->img_rho.rows;
    int n_col = str_next->img_rho.cols;
    int n_ring = n_row;
    int n_radial = n_col;
    float* ptr_cur_img_x = str_cur->img_x.ptr<float>(0);
    float* ptr_cur_img_y = str_cur->img_y.ptr<float>(0);
    float* ptr_cur_img_z = str_cur->img_z.ptr<float>(0);
    // int cnt = 0;

    // timer::tic();
    // representative pts
    for (int i = 0; i < n_row; ++i)
    {
        int i_ncols = i * n_col;
        for (int j = 0; j < n_col; ++j)
        {
            int i_ncols_j = i_ncols + j;
            if ((*(ptr_cur_img_x + i_ncols_j) > 10.0) || (*(ptr_cur_img_x + i_ncols_j) < -10.0) || (*(ptr_cur_img_y + i_ncols_j) > 10.0) || (*(ptr_cur_img_y + i_ncols_j) < -10.0))
            {
                velo_cur.push_back(pcl::PointXYZ(*(ptr_cur_img_x + i_ncols_j), *(ptr_cur_img_y + i_ncols_j), *(ptr_cur_img_z + i_ncols_j)));
            }
        }
    }
    // double dt_1 = timer::toc(); // milliseconds
    // ROS_INFO_STREAM("elapsed time for 'representative pts' :" << dt_1 << " [ms]");

    // timer::tic();
    // Far pts are warped by the original pts
    for (int i = 0; i < p0.size(); ++i)
    {
        if( (p0[i].x < 10) && (p0[i].x > -10.0) && (p0[i].y < 10.0) && (p0[i].y > -10.0) )
        {
            velo_cur.push_back(p0[i]);
        }
    }
    // double dt_2 = timer::toc(); // milliseconds
    // ROS_INFO_STREAM("elapsed time for 'Far pts' :" << dt_2 << " [ms]");

    // compensate zero in current rho image for warping
    // timer::tic();
    compensateCurRhoZeroWarp(str_cur, n_ring, n_radial, v_angle_, velo_cur);
    // double dt_3 = timer::toc(); // milliseconds
    // ROS_INFO_STREAM("elapsed time for 'compensateCurRhoZeroWarp' :" << dt_3 << " [ms]");
    // exit(0);
    // timer::tic();
    pcl::transformPointCloud(velo_cur, *ptr_cur_pts_warped, T01);
    //     double dt_4 = timer::toc(); // milliseconds
    // ROS_INFO_STREAM("elapsed time for 'transformPointCloud' :" << dt_4 << " [ms]");
    
// if (cnt_data == 3)
    // {
        // std::cout << velo_cur.size()<<std::endl;
        // sensor_msgs::PointCloud2 converted_msg_d;
        // pcl::toROSMsg(velo_cur, converted_msg_d);
        // converted_msg_d.header.frame_id = "map";
        // pub_dynamic_pts_.publish(converted_msg_d);

        // sensor_msgs::PointCloud2 converted_msg_s;
        // pcl::toROSMsg(*ptr_cur_pts_warped, converted_msg_s);
        // converted_msg_s.header.frame_id = "map";
        // pub_static_pts_.publish(converted_msg_s);
        // exit(0);
    // }
    // timer::tic();
    // current warped image
    CloudFrame->genRangeImages(*ptr_cur_pts_warped, str_cur_warped, 0);
    //         double dt_5 = timer::toc(); // milliseconds
    // ROS_INFO_STREAM("elapsed time for 'genRangeImages' :" << dt_5 << " [ms]");
    // countZerofloat(str_cur_warped->img_rho);
    // fill range image using interpolation
    // timer::tic();
    interpRangeImageMin(str_cur_warped, n_ring, n_radial);
    // double dt_6 = timer::toc(); // milliseconds
    // ROS_INFO_STREAM("elapsed time for 'interpRangeImageMin' :" << dt_6 << " [ms]");
    // countZerofloat(str_cur_warped->img_rho);
    // fill pts corresponding to filled range image (no affect the original pts)
    // timer::tic();
    interpPtsWarp(str_cur_warped, n_ring, n_radial);
    // double dt_7 = timer::toc(); // milliseconds
    // ROS_INFO_STREAM("elapsed time for 'interpPtsWarp' :" << dt_7 << " [ms]");

    // calculate occlusions
    // timer::tic();
    cv::subtract(str_cur_warped->img_rho, str_next->img_rho, dRdt); 
    // residual_ = str_cur_warped_->img_rho - str_next_->img_rho;
    // double dt_8 = timer::toc(); // milliseconds
    // ROS_INFO_STREAM("elapsed time for 'subtract' :" << dt_8 << " [ms]");
}

void dRCalc::compensateCurRhoZeroWarp(StrRhoPts* str_cur, int n_ring, int n_radial, std::vector<float>& v_angle, pcl::PointCloud<pcl::PointXYZ>& velo_cur)
{
    int n_row = n_ring;
    int n_col = n_radial;
    float left_dir_rho  = 0;
    float right_dir_rho = 0;
    float up_dir_rho    = 0;
    float down_dir_rho  = 0;
    float* ptr_cur_img_rho = str_cur->img_rho.ptr<float>(0);

    int cnt_left    = 1;
    int cnt_right   = 1;
    int cnt_up      = 1;
    int cnt_down    = 1;

    float min_rho_4_dir = 0.0 ;
    
    std::vector<float> four_dir;
    four_dir.resize(4);
    std::vector<int> four_cnt;
    four_cnt.resize(4);

    bool valid_dir[4];

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

    for (int i = 0 + 1; i < n_ring - 1; ++i)
    {
        int i_ncols = i * n_col;
        for (int j = 0 + 1; j < n_radial - 1; ++j)
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
            
            if (*(ptr_cur_img_rho + i_ncols + j) == 0)
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
                            left_dir_rho = 100.0;
                            break;
                        }
                        left_dir_rho = *(ptr_cur_img_rho + i_ncols + (j - cnt_left));
                        cnt_left += 1;
                    } // end while
                    //right
                    while (right_dir_rho == 0.0)
                    {
                        if ((j + cnt_right) > n_col-1)
                        {
                            right_dir_rho = 100.0;
                            break;
                        }
                        right_dir_rho = *(ptr_cur_img_rho + i_ncols + (j + cnt_right));
                        cnt_right += 1;
                    } // end while
                    //up
                    while (up_dir_rho == 0.0)
                    {
                        if ((i - cnt_up) < 0)
                        {
                            up_dir_rho = 100.0;
                            break;
                        }
                        up_dir_rho = *(ptr_cur_img_rho + (i - cnt_up) * n_col + j);
                        cnt_up += 1;
                    } // end while
                    //down
                    while (down_dir_rho == 0.0)
                    {
                        if ((i + cnt_down) > n_row-1)
                        {
                            down_dir_rho = 100.0;
                            break;
                        }
                        down_dir_rho = *(ptr_cur_img_rho + (i + cnt_down) * n_col + j);
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
                        if (four_dir[i_v]!=0 && four_cnt[i_v]<20)
                        {
                            valid_dir[i_v] = 1;
                            dir_tmp.push_back(four_dir[i_v]);
                        }
                    }
                    valid_dir_sum = valid_dir[0] + valid_dir[1] + valid_dir[2] + valid_dir[3];
                    if (valid_dir_sum<1) 
                    {continue;}
                    
                    min_dir = *min_element(dir_tmp.begin(), dir_tmp.end());
                    for (int i_v=0; i_v<4; ++i_v)
                    {
                        if (valid_dir[i_v] == 1 && four_dir[i_v]<(min_dir+1.0f))
                        {
                            valid_dir[i_v] = 1;
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

                        vec_inv_sum = vec_inv[0] + vec_inv[1] +vec_inv[2] +vec_inv[3];

                        for (int i_v=0; i_v<4; ++i_v)
                        {
                            if (vec_inv[i_v]!=0)
                            {
                                min_rho_4_dir += four_dir[i_v]*vec_inv[i_v];
                            }
                        }
                    }


                    // min_rho_4_dir = *min_element(four_dir.begin(), four_dir.end());
                    // min_index = min_element(four_dir.begin(), four_dir.end()) - four_dir.begin();

                    if (min_rho_4_dir > 0.0)
                    {
                        new_phi = v_angle[i] * D2R;
                        new_theta = 0.4 * (j + 1) * D2R;
                        for (int m = 0; m < 5; ++m)
                        {
                            for (int p = 0; p < 5; ++p)
                            {
                                velo_cur.push_back(pcl::PointXYZ(-min_rho_4_dir * cosf(new_phi + (float)(m - 2) * 0.2 * D2R) * cosf(new_theta + (float)(p - 2) * 0.08 * D2R),
                                                                 -min_rho_4_dir * cosf(new_phi + (float)(m - 2) * 0.2 * D2R) * sinf(new_theta + (float)(p - 2) * 0.08 * D2R),
                                                                 min_rho_4_dir * sinf(new_phi + (float)(m - 2) * 0.2 * D2R)));
                            }
                        }
                    } // end if
                        else{}
            } // end if
            else{}
        } // end for j
    } //end for i
    
}

void dRCalc::interpRangeImageMin(StrRhoPts* str_in, int n_ring, int n_radial)
{
    cv::Mat img_rho_new = str_in->img_rho.clone();
    float *ptr_img_rho = str_in->img_rho.ptr<float>(0);
    int *ptr_img_index = str_in->img_index.ptr<int>(0);
    float *ptr_img_rho_new = img_rho_new.ptr<float>(0);
    int *ptr_img_restore_warp_mask = str_in->img_restore_warp_mask.ptr<int>(0);
    int n_col = str_in->img_rho.cols;
    int n_row = str_in->img_rho.rows;

    for (int i = 0 + 2; i < (n_ring - 2); ++i)
    {
        int i_ncols = i * n_col;
        for (int j = 0 + 2; j < (n_radial - 2); ++j)
        {            
            if (*(ptr_img_rho + i_ncols + j) == 0)
            {
                if (*(ptr_img_rho + (i - 1) * n_col + j) != 0 && *(ptr_img_rho + (i + 1) * n_col + j) != 0)
                {
                    if (fabsf32(*(ptr_img_rho + (i - 1) * n_col + j) - *(ptr_img_rho + (i + 1) * n_col + j)) < 0.1)
                    {
                        *(ptr_img_restore_warp_mask + i_ncols + j) = 1;
                        *(ptr_img_rho_new + i_ncols + j) = (*(ptr_img_rho + (i - 1) * n_col + j) + *(ptr_img_rho + (i + 1) * n_col + j)) * 0.5;
                    }
                    else
                    {
                        *(ptr_img_restore_warp_mask + i_ncols + j) = 10;
                        *(ptr_img_rho_new + i_ncols + j) = std::min(*(ptr_img_rho + (i - 1) * n_col + j), *(ptr_img_rho + (i + 1) * n_col + j));
                    }
                }
                else if (*(ptr_img_rho + (i - 1) * n_col + j) != 0 && *(ptr_img_rho + (i + 2) * n_col + j) != 0)
                {
                    if (fabsf32(*(ptr_img_rho + (i - 1) * n_col + j) - *(ptr_img_rho + (i + 2) * n_col + j)) < 0.1)
                    {
                        *(ptr_img_restore_warp_mask + i_ncols + j) = 2;
                        *(ptr_img_restore_warp_mask + (i + 1) * n_col + j) = 3;
                        *(ptr_img_rho_new + i_ncols + j) = *(ptr_img_rho + (i - 1) * n_col + j) * (0.6666667) + *(ptr_img_rho + (i + 2) * n_col + j) * (0.3333333);
                        *(ptr_img_rho_new + (i + 1) * n_col + j) = *(ptr_img_rho + (i - 1) * n_col + j) * (0.3333333) + *(ptr_img_rho + (i + 2) * n_col + j) * (0.6666667);
                    }
                    else
                    {
                        *(ptr_img_restore_warp_mask + i_ncols + j) = 20;
                        *(ptr_img_restore_warp_mask + (i + 1) * n_col + j) = 30;
                        *(ptr_img_rho_new + i_ncols + j) = std::min(*(ptr_img_rho + (i - 1) * n_col + j), *(ptr_img_rho + (i + 2) * n_col + j));
                        *(ptr_img_rho_new + (i + 1) * n_col + j) = std::min(*(ptr_img_rho + (i - 1) * n_col + j), *(ptr_img_rho + (i + 2) * n_col + j));
                    }
                }

                if (*(ptr_img_rho + i_ncols + (j - 1)) != 0 && *(ptr_img_rho + i_ncols + (j + 1)) != 0)
                {
                    if (fabsf32(*(ptr_img_rho + i_ncols + (j - 1)) - *(ptr_img_rho + i_ncols + (j + 1))) < 0.05)
                    {
                        *(ptr_img_restore_warp_mask + i_ncols + j) = 4;
                        *(ptr_img_rho_new + i_ncols + j) = (*(ptr_img_rho + i_ncols + (j - 1)) + *(ptr_img_rho + i_ncols + (j + 1))) * 0.5;
                    }
                    else{}
                }
                else if (*(ptr_img_rho + i_ncols + (j - 1)) != 0 && *(ptr_img_rho + i_ncols + (j + 2)) != 0)
                {
                    if (fabsf32(*(ptr_img_rho + i_ncols + (j - 1)) - *(ptr_img_rho + i_ncols + (j + 2))) < 0.05)
                    {
                        *(ptr_img_restore_warp_mask + i_ncols + j) = 5;
                        *(ptr_img_restore_warp_mask + i_ncols + (j + 1)) = 6;
                        *(ptr_img_rho_new + i_ncols + j) = *(ptr_img_rho + i_ncols + (j - 1)) * (0.6666667) + *(ptr_img_rho + i_ncols + (j + 2)) * (0.3333333);
                        *(ptr_img_rho_new + i_ncols + (j + 1)) = *(ptr_img_rho + i_ncols + (j - 1)) * (0.3333333) + *(ptr_img_rho + i_ncols + (j + 2)) * (0.6666667);
                    }
                    else{}
                }
            } // end if
            else{}
        } // end col

    } // end row

    img_rho_new.copyTo(str_in->img_rho);
}

void dRCalc::interpPtsWarp(StrRhoPts* str_in, int n_ring, int n_radial)
{
    int n_row = n_ring;
    int n_col = n_radial;
    int* ptr_img_index = str_in->img_index.ptr<int>(0);
    float* ptr_img_rho = str_in->img_rho.ptr<float>(0);
    float* ptr_img_x = str_in->img_x.ptr<float>(0);
    float* ptr_img_y = str_in->img_y.ptr<float>(0);
    float* ptr_img_z = str_in->img_z.ptr<float>(0);
    int* ptr_img_restore_warp_mask = str_in->img_restore_warp_mask.ptr<int>(0);

    for (int i = 0 + 2; i < n_row - 2; ++i)
    {
        int i_ncols = i * n_col;
        for (int j = 0 + 2; j < n_col - 2; ++j)
        {            
            if (*(ptr_img_restore_warp_mask + i_ncols + j) == 1)
            {
                *(ptr_img_x + i_ncols + j) = 0.5 * (*(ptr_img_x + (i - 1) * n_col + j) + *(ptr_img_x + (i + 1) * n_col + j));
                *(ptr_img_y + i_ncols + j) = 0.5 * (*(ptr_img_y + (i - 1) * n_col + j) + *(ptr_img_y + (i + 1) * n_col + j));
                *(ptr_img_z + i_ncols + j) = 0.5 * (*(ptr_img_z + (i - 1) * n_col + j) + *(ptr_img_z + (i + 1) * n_col + j));
            }
            else if (*(ptr_img_restore_warp_mask + i_ncols + j) == 10)
            {
                if ((*(ptr_img_rho + (i - 1) * n_col + j) < *(ptr_img_rho + (i + 1) * n_col + j)))
                {
                    *(ptr_img_x + i_ncols + j) = *(ptr_img_x + (i-1) * n_col + j);
                    *(ptr_img_y + i_ncols + j) = *(ptr_img_y + (i-1) * n_col + j);
                    *(ptr_img_z + i_ncols + j) = *(ptr_img_z + (i-1) * n_col + j);
                }
                else
                {
                    *(ptr_img_x + i_ncols + j) = *(ptr_img_x + (i+1) * n_col + j);
                    *(ptr_img_y + i_ncols + j) = *(ptr_img_y + (i+1) * n_col + j);
                    *(ptr_img_z + i_ncols + j) = *(ptr_img_z + (i+1) * n_col + j);
                }
            }
            else if (*(ptr_img_restore_warp_mask + i_ncols + j) == 2)
            {
                *(ptr_img_x + i_ncols + j) = (0.6666667) * (*(ptr_img_x + (i - 1) * n_col + j)) + (0.3333333) * (*(ptr_img_x + (i + 2) * n_col + j));
                *(ptr_img_y + i_ncols + j) = (0.6666667) * (*(ptr_img_y + (i - 1) * n_col + j)) + (0.3333333) * (*(ptr_img_y + (i + 2) * n_col + j));
                *(ptr_img_z + i_ncols + j) = (0.6666667) * (*(ptr_img_z + (i - 1) * n_col + j)) + (0.3333333) * (*(ptr_img_z + (i + 2) * n_col + j));
            }
            else if (*(ptr_img_restore_warp_mask + i_ncols + j) == 20)
            {
                if ((*(ptr_img_rho + (i - 1) * n_col + j) < *(ptr_img_rho + (i + 2) * n_col + j)))
                {
                    *(ptr_img_x + i_ncols + j) = *(ptr_img_x + (i-1) * n_col + j);
                    *(ptr_img_y + i_ncols + j) = *(ptr_img_y + (i-1) * n_col + j);
                    *(ptr_img_z + i_ncols + j) = *(ptr_img_z + (i-1) * n_col + j);
                }
                else
                {
                    *(ptr_img_x + i_ncols + j) = *(ptr_img_x + (i+2) * n_col + j);
                    *(ptr_img_y + i_ncols + j) = *(ptr_img_y + (i+2) * n_col + j);
                    *(ptr_img_z + i_ncols + j) = *(ptr_img_z + (i+2) * n_col + j);
                }
            }
            else if (*(ptr_img_restore_warp_mask + i_ncols + j) == 3)
            {
                *(ptr_img_x + i_ncols + j) = (0.3333333) * (*(ptr_img_x + (i - 2) * n_col + j)) + (0.6666667) * (*(ptr_img_x + (i + 1) * n_col + j));
                *(ptr_img_y + i_ncols + j) = (0.3333333) * (*(ptr_img_y + (i - 2) * n_col + j)) + (0.6666667) * (*(ptr_img_y + (i + 1) * n_col + j));
                *(ptr_img_z + i_ncols + j) = (0.3333333) * (*(ptr_img_z + (i - 2) * n_col + j)) + (0.6666667) * (*(ptr_img_z + (i + 1) * n_col + j));
            }
            else if (*(ptr_img_restore_warp_mask + i_ncols + j) == 30)
            {
                if ((*(ptr_img_rho + (i - 2) * n_col + j) < *(ptr_img_rho + (i + 1) * n_col + j)))
                {
                    *(ptr_img_x + i_ncols + j) = *(ptr_img_x + (i-2) * n_col + j);
                    *(ptr_img_y + i_ncols + j) = *(ptr_img_y + (i-2) * n_col + j);
                    *(ptr_img_z + i_ncols + j) = *(ptr_img_z + (i-2) * n_col + j);
                }
                else
                {
                    *(ptr_img_x + i_ncols + j) = *(ptr_img_x + (i+1) * n_col + j);
                    *(ptr_img_y + i_ncols + j) = *(ptr_img_y + (i+1) * n_col + j);
                    *(ptr_img_z + i_ncols + j) = *(ptr_img_z + (i+1) * n_col + j);
                }
            }
            else if (*(ptr_img_restore_warp_mask + i_ncols + j) == 4)
            {
                *(ptr_img_x + i_ncols + j) = 0.5 * (*(ptr_img_x + i_ncols + (j - 1)) + (*(ptr_img_x + i_ncols + (j + 1))));
                *(ptr_img_y + i_ncols + j) = 0.5 * (*(ptr_img_y + i_ncols + (j - 1)) + (*(ptr_img_y + i_ncols + (j + 1))));
                *(ptr_img_z + i_ncols + j) = 0.5 * (*(ptr_img_z + i_ncols + (j - 1)) + (*(ptr_img_z + i_ncols + (j + 1))));
            }
            else if (*(ptr_img_restore_warp_mask + i_ncols + j) == 5)
            {
                *(ptr_img_x + i_ncols + j) = (0.6666667) * (*(ptr_img_x + i_ncols + (j - 1))) + (0.3333333) * (*(ptr_img_x + i_ncols + (j + 2)));
                *(ptr_img_y + i_ncols + j) = (0.6666667) * (*(ptr_img_y + i_ncols + (j - 1))) + (0.3333333) * (*(ptr_img_y + i_ncols + (j + 2)));
                *(ptr_img_z + i_ncols + j) = (0.6666667) * (*(ptr_img_z + i_ncols + (j - 1))) + (0.3333333) * (*(ptr_img_z + i_ncols + (j + 2)));
            }
            else if (*(ptr_img_restore_warp_mask + i_ncols + j) == 6)
            {
                *(ptr_img_x + i_ncols + j) = (0.3333333) * (*(ptr_img_x + i_ncols + (j - 2))) + (0.6666667) * (*(ptr_img_x + i_ncols + (j + 1)));
                *(ptr_img_y + i_ncols + j) = (0.3333333) * (*(ptr_img_y + i_ncols + (j - 2))) + (0.6666667) * (*(ptr_img_y + i_ncols + (j + 1)));
                *(ptr_img_z + i_ncols + j) = (0.3333333) * (*(ptr_img_z + i_ncols + (j - 2))) + (0.6666667) * (*(ptr_img_z + i_ncols + (j + 1)));
            }
            else{}
        } // end for j
    } // end for i
}