#include "cloud_frame.h"

CloudFrame::CloudFrame(const std::unique_ptr<UserParam>& user_param)
{
    azimuth_res_ = user_param->cloud_filter_param_.azimuth_res_ * (float)user_param->cloud_filter_param_.h_factor_;
    az_step_ = 1.0f/azimuth_res_;
    n_radial_ = 360 * az_step_ + 1;
    n_ring_ = 64 / user_param->cloud_filter_param_.v_factor_;

    for (int i=0; i<user_param->sensor_spec_.v_angle_.size(); ++i)
    {
        v_angle_.push_back(user_param->sensor_spec_.v_angle_[i]);
    }
    // n_pts_ = pcl_in.size();
};

CloudFrame::~CloudFrame()
{
    // destructor
};

void CloudFrame::genRangeImages(pcl::PointCloud<pcl::PointXYZ> &pcl_in, StrRhoPts *str_in, bool cur_next)
{
    n_pts_ = pcl_in.size();

    calcuateRho(pcl_in, str_in);

    makeRangeImageAndPtsPerPixel(str_in, n_pts_, n_ring_, n_radial_, az_step_);

    interpRangeImage(str_in, n_ring_, n_radial_, cur_next);

    interpPts(pcl_in, str_in, n_ring_, n_radial_, cur_next);
}

void CloudFrame::calcuateRho(pcl::PointCloud<pcl::PointXYZ>& pcl_in, StrRhoPts* str_in)
{
    // timer::tic();

    float twopi         = 2.0*M_PI;
    float offset_theta  = M_PI;

    int n_pts = pcl_in.size();
    float invrhocos = 0.0;
    float cospsi = 0.0;
    float sinpsi = 0.0;

    float* ptr_rho = str_in->rho.data();
    float* ptr_phi = str_in->phi.data();
    float* ptr_theta = str_in->theta.data();

    pcl::PointXYZ origin;
    origin.x = 0.0f;
    origin.y = 0.0f;
    origin.z = 0.0f;

    float M_PI_plus_offset_theta = M_PI + offset_theta;
    float twopi_plus_offset_theta = twopi + offset_theta;

    for (int i = 0; i < n_pts; ++i)
    {
        str_in->rho.push_back(NORM(pcl_in.points[i].x, pcl_in.points[i].y, pcl_in.points[i].z));
        str_in->phi.push_back(asin(pcl_in.points[i].z / ptr_rho[i]));
        invrhocos = (float)1.0 / (ptr_rho[i] * cos(ptr_phi[i]));

        cospsi = pcl_in.points[i].x * invrhocos;
        if (cospsi > 1)
        {
            cospsi = (float)1.0;
        }
        else if (cospsi < -1)
        {
            cospsi = -(float)1.0;
        }
        else{}

        sinpsi = pcl_in.points[i].y * invrhocos;

        if (cospsi >= 0)
        {
            if (sinpsi >= 0) // 1 quadrant
            {
                str_in->theta.push_back(acos(cospsi) + offset_theta);
            }
            else // 4 quadrant
            {
                str_in->theta.push_back(twopi_plus_offset_theta - acos(cospsi));
            }
        }
        else
        {
            if (sinpsi >= 0) // 2 quadrant
            {
                str_in->theta.push_back(M_PI_plus_offset_theta - acos(-cospsi));
            }
            else // 3 quadrant
            {
                str_in->theta.push_back(M_PI_plus_offset_theta + acos(-cospsi));
            }
        }

        if (ptr_theta[i] >= twopi)
        {
            ptr_theta[i] = ptr_theta[i] - twopi;
        }
        // std::cout << str_in->rho[i] << " " << str_in->phi[i] << " " << str_in->theta[i]<< std::endl;
    }
    // double dt_slam = timer::toc(); // milliseconds
    // ROS_INFO_STREAM("elapsed time for 'calcuateRho' :" << dt_slam << " [ms]");
}

void CloudFrame::makeRangeImageAndPtsPerPixel(StrRhoPts *str_in, int n_pts, int n_ring, int n_radial, float az_step)
{
    // timer::tic();
    int i_row = 0;
    int i_col = 0;
    float* ptr_img_rho = str_in->img_rho.ptr<float>(0);
    int* ptr_img_index = str_in->img_index.ptr<int>(0);
    int n_row = str_in->img_rho.rows;
    int n_col = str_in->img_rho.cols;

    float* ptr_rho = str_in->rho.data();
    float* ptr_phi = str_in->phi.data();
    float* ptr_theta = str_in->theta.data();
    float* ptr_v_angle = v_angle_.data();

    float az_step_R2D = az_step*R2D;
    
    for (int i = 0; i < n_pts; ++i)
    {   
        float phi_R2D = (ptr_phi[i] * R2D);
        // std::cout << str_in->rho[i] << " " << str_in->phi[i] << " " << str_in->theta[i]<< std::endl;
        // for (int kk = 0; kk < n_ring; ++kk)
        // {
        //     if (ptr_v_angle[kk] < phi_R2D)
        //     {
        //         i_row = kk;
        //         break;
        //     }
        //     else{}
        //     if (kk == (n_ring-1))
        //     {
        //         i_row = n_ring - 1;
        //     }
        //     else{}
        // }
        if (phi_R2D>2.5)
        {
            i_row = 0;
        }
        else if (phi_R2D>-8.0)
        {
            i_row = (int)(-2.9523810*phi_R2D+7.3809524);
        }
        else if (phi_R2D>-8.5)
        {
            i_row = 32;
        }
        else if (phi_R2D>-23.8)
        {
            i_row = (int)(-2.0261438*phi_R2D+14.7777778);
        }
        else
        {
            i_row = 63;
        }
        i_col = roundf(ptr_theta[i]*az_step_R2D);

        if ( (i_row > n_ring-1) || (i_row < 0) )
        {
            continue;
        }
        
        int i_row_ncols_i_col = i_row * n_col + i_col;
        if (*(ptr_img_rho + i_row_ncols_i_col) == 0) //(str_in->img_rho.at<float>(i_row,i_col) == 0)
        {   
            *(ptr_img_rho + i_row_ncols_i_col) = ptr_rho[i];
            *(ptr_img_index + i_row_ncols_i_col) = i;
        }
        else if (*(ptr_img_rho + i_row_ncols_i_col) > ptr_rho[i])
        {
            *(ptr_img_rho + i_row_ncols_i_col) = ptr_rho[i];
            *(ptr_img_index + i_row_ncols_i_col) = i;
        }
        else{}

        str_in->pts_per_pixel_n[i_row_ncols_i_col] += 1;
        str_in->pts_per_pixel_index[i_row_ncols_i_col].push_back(i);
        str_in->pts_per_pixel_rho[i_row_ncols_i_col].push_back(ptr_rho[i]);
    } // end for

    // for(int a=0; a < str_in->pts_per_pixel_rho.size(); ++a)
    // {
    //     // std::cout << str_in->pts_per_pixel_n[a] << std::endl;
    //     for (int b =0; b< str_in->pts_per_pixel_rho[a].size(); ++b)
    //     {
    //         std::cout << str_in->pts_per_pixel_rho[a][b] << " " ;
    //     }
    //     std::cout << std::endl;
    // }
    // exit(0);
    // double dt_slam = timer::toc(); // milliseconds
    // ROS_INFO_STREAM("elapsed time for 'makeRangeImageAndPtsPerPixel' :" << dt_slam << " [ms]");
}

void CloudFrame::interpRangeImage(StrRhoPts* str_in, int n_ring, int n_radial, bool cur_next)
{
    cv::Mat img_rho_new = str_in->img_rho.clone();
    float* ptr_img_rho_new = img_rho_new.ptr<float>(0);

    float* ptr_img_rho = str_in->img_rho.ptr<float>(0);
    int* ptr_img_index = str_in->img_index.ptr<int>(0);
    int* ptr_img_restore_mask = str_in->img_restore_mask.ptr<int>(0);
    int n_col = str_in->img_rho.cols;
    int n_row = str_in->img_rho.rows;

    for (int i = 27; i < 32; i++)
    {
        int i_ncols = i * n_col;
        int i_minus_ncols = (i - 1) * n_col;
        int i_plus_ncols = (i + 1) * n_col;
        for (int j = 0 + 2; j < (n_radial - 2); ++j)
        {
            if (*(ptr_img_rho + i_ncols + j) == 0)
            {
                if ((*(ptr_img_rho + i_minus_ncols + j) != 0))
                {
                    if ((*(ptr_img_rho + i_plus_ncols + j) != 0))
                    {
                        if (fabsf32(*(ptr_img_rho + i_minus_ncols + j) - *(ptr_img_rho + i_plus_ncols + j)) < 0.1)
                        {
                            *(ptr_img_restore_mask + i_ncols + j) = 1;
                            *(ptr_img_rho_new + i_ncols + j) = (*(ptr_img_rho + i_minus_ncols + j) + *(ptr_img_rho + i_plus_ncols + j)) * 0.5;
                        }
                        else
                        {
                            *(ptr_img_restore_mask + i_ncols + j) = 10;
                            if (cur_next == 0)
                            {
                                *(ptr_img_rho_new + i_ncols + j) = std::min(*(ptr_img_rho + i_minus_ncols + j), *(ptr_img_rho + i_plus_ncols + j));
                            }
                            else
                            {
                                *(ptr_img_rho_new + i_ncols + j) = std::max(*(ptr_img_rho + i_minus_ncols + j), *(ptr_img_rho + i_plus_ncols + j));
                            }
                        }
                    }
                    else if ((*(ptr_img_rho + (i + 2) * n_col + j) != 0))
                    {
                        if (fabsf32(*(ptr_img_rho + i_minus_ncols + j) - *(ptr_img_rho + (i + 2) * n_col + j)) < 0.1)
                        {
                            *(ptr_img_restore_mask + i_ncols + j) = 2;
                            *(ptr_img_restore_mask + i_plus_ncols + j) = 3;
                            *(ptr_img_rho_new + i_ncols + j) = *(ptr_img_rho + i_minus_ncols + j) * (0.6666667) + *(ptr_img_rho + (i + 2) * n_col + j) * (0.3333333);
                            *(ptr_img_rho_new + i_plus_ncols + j) = *(ptr_img_rho + i_minus_ncols + j) * (0.3333333) + *(ptr_img_rho + (i + 2) * n_col + j) * (0.6666667);
                        }
                        else
                        {
                            *(ptr_img_restore_mask + i_ncols + j) = 20;
                            *(ptr_img_restore_mask + i_plus_ncols + j) = 30;
                            if (cur_next == 0)
                            {
                                *(ptr_img_rho_new + i_ncols + j)        = std::min(*(ptr_img_rho + i_minus_ncols + j), *(ptr_img_rho + (i + 2) * n_col + j));
                                *(ptr_img_rho_new + i_plus_ncols + j)   = std::min(*(ptr_img_rho + i_minus_ncols + j), *(ptr_img_rho + (i + 2) * n_col + j));
                            }
                            else
                            {
                                *(ptr_img_rho_new + i_ncols + j)        = std::max(*(ptr_img_rho + i_minus_ncols + j), *(ptr_img_rho + (i + 2) * n_col + j));
                                *(ptr_img_rho_new + i_plus_ncols + j)   = std::max(*(ptr_img_rho + i_minus_ncols + j), *(ptr_img_rho + (i + 2) * n_col + j));
                            }
                        }
                    }
                    else{}
                }
            } // end if
            else{}

            if (*(ptr_img_rho + i_ncols + (j - 1)) != 0)
            {
                if ((*(ptr_img_rho + i_ncols + (j + 1)) != 0))
                {
                    if (fabsf32(*(ptr_img_rho + i_ncols + (j - 1)) - *(ptr_img_rho + i_ncols + (j + 1))) < 0.05)
                    {
                        *(ptr_img_restore_mask + i_ncols + j) = 4;
                        *(ptr_img_rho_new + i_ncols + j) = (*(ptr_img_rho + i_ncols + (j - 1)) + *(ptr_img_rho + i_ncols + (j + 1))) * 0.5;
                    }
                    else{}
                }
                else if ((*(ptr_img_rho + i_ncols + (j + 2)) != 0))
                {
                    if (fabsf32(*(ptr_img_rho + i_ncols + (j - 1)) - *(ptr_img_rho + i_ncols + (j + 2))) < 0.05)
                    {
                        *(ptr_img_restore_mask + i_ncols + j) = 5;
                        *(ptr_img_restore_mask + i_ncols + (j + 1)) = 6;
                        *(ptr_img_rho_new + i_ncols + j) = *(ptr_img_rho + i_ncols + (j - 1)) * (0.6666667) + *(ptr_img_rho + i_ncols + (j + 2)) * (0.3333333);
                        *(ptr_img_rho_new + i_ncols + (j + 1)) = *(ptr_img_rho + i_ncols + (j - 1)) * (0.3333333) + *(ptr_img_rho + i_ncols + (j + 2)) * (0.6666667);
                    }
                    else{}
                }
                else{}
            }
            else{}

        } // end col

    } // end row

    img_rho_new.copyTo(str_in->img_rho);
}

void CloudFrame::interpPts(pcl::PointCloud<pcl::PointXYZ>& pcl_in, StrRhoPts* str_in, int n_ring, int n_radial, bool cur_next)
{
    int n_col = str_in->img_rho.cols;
    int n_row = str_in->img_rho.rows;
    
    float* ptr_img_rho = str_in->img_rho.ptr<float>(0);
    int* ptr_img_index = str_in->img_index.ptr<int>(0);
    float* ptr_img_x = str_in->img_x.ptr<float>(0);
    float* ptr_img_y = str_in->img_y.ptr<float>(0);
    float* ptr_img_z = str_in->img_z.ptr<float>(0);
    int* ptr_img_restore_mask = str_in->img_restore_mask.ptr<int>(0);

    for (int i = 0; i < n_ring; ++i)
    {
        int i_ncols = i * n_col;
        for (int j = 0; j < n_radial; ++j)
        {
            int i_ncols_j = i_ncols + j;
            if (str_in->pts_per_pixel_rho[i_ncols_j].size() > 0)
            {
                for (int k = 0; k < (str_in->pts_per_pixel_rho[i_ncols_j].size()); ++k)
                {
                    if (std::abs((str_in->pts_per_pixel_rho[i_ncols_j][k] - *(ptr_img_rho + i_ncols_j))) < 0.5)
                    {
                        str_in->pts_per_pixel_index_valid[i_ncols_j].push_back(str_in->pts_per_pixel_index[i_ncols_j][k]);
                    }
                    else{}
                }
            } // end if
            else{}

            if (*(ptr_img_index + i_ncols_j) != 0)
            {
                *(ptr_img_x + i_ncols_j) = pcl_in[*(ptr_img_index + i_ncols_j)].x;
                *(ptr_img_y + i_ncols_j) = pcl_in[*(ptr_img_index + i_ncols_j)].y;
                *(ptr_img_z + i_ncols_j) = pcl_in[*(ptr_img_index + i_ncols_j)].z;
            }
            else{}

            switch(*(ptr_img_restore_mask + i_ncols_j))
            {
                case 1:
                    *(ptr_img_x + i_ncols_j) = 0.5 * (pcl_in[*(ptr_img_index + i_ncols_j - n_col)].x + pcl_in[*(ptr_img_index + i_ncols_j + n_col)].x);
                    *(ptr_img_y + i_ncols_j) = 0.5 * (pcl_in[*(ptr_img_index + i_ncols_j - n_col)].y + pcl_in[*(ptr_img_index + i_ncols_j + n_col)].y);
                    *(ptr_img_z + i_ncols_j) = 0.5 * (pcl_in[*(ptr_img_index + i_ncols_j - n_col)].z + pcl_in[*(ptr_img_index + i_ncols_j + n_col)].z);
                    break;
                case 10:
                    if (cur_next == 0)
                    {
                        if ((*(ptr_img_rho + i_ncols_j - n_col) > *(ptr_img_rho + i_ncols_j + n_col)))
                        {
                            *(ptr_img_x + i_ncols_j) = pcl_in[*(ptr_img_index + i_ncols_j + n_col)].x;
                            *(ptr_img_y + i_ncols_j) = pcl_in[*(ptr_img_index + i_ncols_j + n_col)].y;
                            *(ptr_img_z + i_ncols_j) = pcl_in[*(ptr_img_index + i_ncols_j + n_col)].z;
                        }
                        else
                        {
                            *(ptr_img_x + i_ncols_j) = pcl_in[*(ptr_img_index + i_ncols_j - n_col)].x;
                            *(ptr_img_y + i_ncols_j) = pcl_in[*(ptr_img_index + i_ncols_j - n_col)].y;
                            *(ptr_img_z + i_ncols_j) = pcl_in[*(ptr_img_index + i_ncols_j - n_col)].z;            
                        }
                        break;
                    }
                    else
                    {
                        if ((*(ptr_img_rho + i_ncols_j - n_col) < *(ptr_img_rho + i_ncols_j + n_col)))
                        {
                            *(ptr_img_x + i_ncols_j) = pcl_in[*(ptr_img_index + i_ncols_j + n_col)].x;
                            *(ptr_img_y + i_ncols_j) = pcl_in[*(ptr_img_index + i_ncols_j + n_col)].y;
                            *(ptr_img_z + i_ncols_j) = pcl_in[*(ptr_img_index + i_ncols_j + n_col)].z;
                        }
                        else
                        {
                            *(ptr_img_x + i_ncols_j) = pcl_in[*(ptr_img_index + i_ncols_j - n_col)].x;
                            *(ptr_img_y + i_ncols_j) = pcl_in[*(ptr_img_index + i_ncols_j - n_col)].y;
                            *(ptr_img_z + i_ncols_j) = pcl_in[*(ptr_img_index + i_ncols_j - n_col)].z;            
                        }
                        break;
                    }

                case 2:
                    *(ptr_img_x + i_ncols_j) = (0.6666667) * pcl_in[*(ptr_img_index + i_ncols_j - n_col)].x + (0.3333333) * pcl_in[*(ptr_img_index + i_ncols_j + 2 * n_col)].x;
                    *(ptr_img_y + i_ncols_j) = (0.6666667) * pcl_in[*(ptr_img_index + i_ncols_j - n_col)].y + (0.3333333) * pcl_in[*(ptr_img_index + i_ncols_j + 2 * n_col)].y;
                    *(ptr_img_z + i_ncols_j) = (0.6666667) * pcl_in[*(ptr_img_index + i_ncols_j - n_col)].z + (0.3333333) * pcl_in[*(ptr_img_index + i_ncols_j + 2 * n_col)].z;
                    break;
                case 20:
                    if (cur_next == 0)
                    {
                        if ((*(ptr_img_rho + i_ncols_j - n_col) > *(ptr_img_rho + i_ncols_j + 2 * n_col)))
                        {                            
                            *(ptr_img_x + i_ncols_j) = pcl_in[*(ptr_img_index + i_ncols_j + 2 * n_col)].x;
                            *(ptr_img_y + i_ncols_j) = pcl_in[*(ptr_img_index + i_ncols_j + 2 * n_col)].y;
                            *(ptr_img_z + i_ncols_j) = pcl_in[*(ptr_img_index + i_ncols_j + 2 * n_col)].z;
                        }
                        else
                        {
                            *(ptr_img_x + i_ncols_j) = pcl_in[*(ptr_img_index + i_ncols_j - n_col)].x;
                            *(ptr_img_y + i_ncols_j) = pcl_in[*(ptr_img_index + i_ncols_j - n_col)].y;
                            *(ptr_img_z + i_ncols_j) = pcl_in[*(ptr_img_index + i_ncols_j - n_col)].z;
                        }
                        break;
                    }
                    else
                    {
                        if ((*(ptr_img_rho + i_ncols_j - n_col) < *(ptr_img_rho + i_ncols_j + 2 * n_col)))
                        {
                            *(ptr_img_x + i_ncols_j) = pcl_in[*(ptr_img_index + i_ncols_j + 2 * n_col)].x;
                            *(ptr_img_y + i_ncols_j) = pcl_in[*(ptr_img_index + i_ncols_j + 2 * n_col)].y;
                            *(ptr_img_z + i_ncols_j) = pcl_in[*(ptr_img_index + i_ncols_j + 2 * n_col)].z;
                        }
                        else
                        {
                            *(ptr_img_x + i_ncols_j) = pcl_in[*(ptr_img_index + i_ncols_j - n_col)].x;
                            *(ptr_img_y + i_ncols_j) = pcl_in[*(ptr_img_index + i_ncols_j - n_col)].y;
                            *(ptr_img_z + i_ncols_j) = pcl_in[*(ptr_img_index + i_ncols_j - n_col)].z;
                        }
                        break;
                    }

                case 3:
                    *(ptr_img_x + i_ncols_j) = (0.3333333) * pcl_in[*(ptr_img_index + i_ncols_j - 2 * n_col)].x + (0.6666667) * pcl_in[*(ptr_img_index + i_ncols_j + n_col)].x;
                    *(ptr_img_y + i_ncols_j) = (0.3333333) * pcl_in[*(ptr_img_index + i_ncols_j - 2 * n_col)].y + (0.6666667) * pcl_in[*(ptr_img_index + i_ncols_j + n_col)].y;
                    *(ptr_img_z + i_ncols_j) = (0.3333333) * pcl_in[*(ptr_img_index + i_ncols_j - 2 * n_col)].z + (0.6666667) * pcl_in[*(ptr_img_index + i_ncols_j + n_col)].z;
                    break;
                case 30:
                    if(cur_next == 0)
                    {
                        if ((*(ptr_img_rho + i_ncols_j - 2 * n_col) > *(ptr_img_rho + i_ncols_j + n_col)))
                        {
                            *(ptr_img_x + i_ncols_j) = pcl_in[*(ptr_img_index + i_ncols_j + n_col)].x;
                            *(ptr_img_y + i_ncols_j) = pcl_in[*(ptr_img_index + i_ncols_j + n_col)].y;
                            *(ptr_img_z + i_ncols_j) = pcl_in[*(ptr_img_index + i_ncols_j + n_col)].z;
                        }
                        else
                        {
                            *(ptr_img_x + i_ncols_j) = pcl_in[*(ptr_img_index + i_ncols_j - 2 * n_col)].x;
                            *(ptr_img_y + i_ncols_j) = pcl_in[*(ptr_img_index + i_ncols_j - 2 * n_col)].y;
                            *(ptr_img_z + i_ncols_j) = pcl_in[*(ptr_img_index + i_ncols_j - 2 * n_col)].z;
                        }
                        break;
                    }
                    else
                    {
                        if ((*(ptr_img_rho + i_ncols_j - 2 * n_col) < *(ptr_img_rho + i_ncols_j + n_col)))
                        {
                            *(ptr_img_x + i_ncols_j) = pcl_in[*(ptr_img_index + i_ncols_j + n_col)].x;
                            *(ptr_img_y + i_ncols_j) = pcl_in[*(ptr_img_index + i_ncols_j + n_col)].y;
                            *(ptr_img_z + i_ncols_j) = pcl_in[*(ptr_img_index + i_ncols_j + n_col)].z;
                        }
                        else
                        {
                            *(ptr_img_x + i_ncols_j) = pcl_in[*(ptr_img_index + i_ncols_j - 2 * n_col)].x;
                            *(ptr_img_y + i_ncols_j) = pcl_in[*(ptr_img_index + i_ncols_j - 2 * n_col)].y;
                            *(ptr_img_z + i_ncols_j) = pcl_in[*(ptr_img_index + i_ncols_j - 2 * n_col)].z;
                        }
                        break;
                    }

                case 4:
                    *(ptr_img_x + i_ncols_j) = 0.5 * (pcl_in[*(ptr_img_index + i_ncols_j - 1)].x + pcl_in[*(ptr_img_index + i_ncols_j + 1)].x);
                    *(ptr_img_y + i_ncols_j) = 0.5 * (pcl_in[*(ptr_img_index + i_ncols_j - 1)].y + pcl_in[*(ptr_img_index + i_ncols_j + 1)].y);
                    *(ptr_img_z + i_ncols_j) = 0.5 * (pcl_in[*(ptr_img_index + i_ncols_j - 1)].z + pcl_in[*(ptr_img_index + i_ncols_j + 1)].z);
                    break;
                case 5:
                    *(ptr_img_x + i_ncols_j) = (0.6666667) * pcl_in[*(ptr_img_index + i_ncols_j - 1)].x + (0.3333333) * pcl_in[*(ptr_img_index + i_ncols_j + 2)].x;
                    *(ptr_img_y + i_ncols_j) = (0.6666667) * pcl_in[*(ptr_img_index + i_ncols_j - 1)].y + (0.3333333) * pcl_in[*(ptr_img_index + i_ncols_j + 2)].y;
                    *(ptr_img_z + i_ncols_j) = (0.6666667) * pcl_in[*(ptr_img_index + i_ncols_j - 1)].z + (0.3333333) * pcl_in[*(ptr_img_index + i_ncols_j + 2)].z;
                    break;
                case 6:
                    *(ptr_img_x + i_ncols_j) = (0.3333333) * pcl_in[*(ptr_img_index + i_ncols_j - 2)].x + (0.6666667) * pcl_in[*(ptr_img_index + i_ncols_j + 1)].x;
                    *(ptr_img_y + i_ncols_j) = (0.3333333) * pcl_in[*(ptr_img_index + i_ncols_j - 2)].y + (0.6666667) * pcl_in[*(ptr_img_index + i_ncols_j + 1)].y;
                    *(ptr_img_z + i_ncols_j) = (0.3333333) * pcl_in[*(ptr_img_index + i_ncols_j - 2)].z + (0.6666667) * pcl_in[*(ptr_img_index + i_ncols_j + 1)].z;
                    break;
            }
        }     // end for j
    }         // end for i
}