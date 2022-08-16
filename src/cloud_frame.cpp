#include "cloud_frame.h"

CloudFrame::CloudFrame(const std::unique_ptr<UserParam>& user_param)
{
    // user param ///////////////////////////////////////////////////////////////////////////////////
    azimuth_res_ = user_param->cloud_filter_param_.azimuth_res_ * (float)user_param->cloud_filter_param_.h_factor_;
    az_step_ = 1.0f/azimuth_res_;
    n_radial_ = 360 * az_step_ + 1;
    n_ring_ = 64 / user_param->cloud_filter_param_.v_factor_;

    for (int i=0; i<user_param->sensor_spec_.v_angle_.size(); ++i)
    {
        v_angle_.push_back(user_param->sensor_spec_.v_angle_[i]);
    }

    img_height_ = user_param->image_param_.height_;
    img_width_ = user_param->image_param_.width_;
    ///////////////////////////////////////////////////////////////////////////////////
    
    //
    str_rhopts_ = std::make_shared<StrRhoPts>();

    str_rhopts_->rho.reserve(5000000);
    str_rhopts_->phi.reserve(5000000);
    str_rhopts_->theta.reserve(5000000);
    str_rhopts_->img_rho   = cv::Mat::zeros(img_height_, img_width_, CV_32FC1);
    str_rhopts_->img_index = cv::Mat::zeros(img_height_, img_width_, CV_32SC1);

    str_rhopts_->img_x     = cv::Mat::zeros(img_height_, img_width_, CV_32FC1);
    str_rhopts_->img_y     = cv::Mat::zeros(img_height_, img_width_, CV_32FC1);
    str_rhopts_->img_z     = cv::Mat::zeros(img_height_, img_width_, CV_32FC1);

    // str_rhopts_->pts_per_pixel_n.resize(img_height_ * img_width_);
    str_rhopts_->pts_per_pixel_index.resize(img_height_ * img_width_);
    str_rhopts_->pts_per_pixel_rho.resize(img_height_ * img_width_);
    str_rhopts_->pts_per_pixel_index_valid.resize(img_height_ * img_width_);
    for (int i=0; i<img_height_*img_width_; ++i)
    {
        str_rhopts_->pts_per_pixel_index[i].reserve(5000);
    }
    for (int i=0; i<img_height_*img_width_; ++i)
    {
        str_rhopts_->pts_per_pixel_rho[i].reserve(5000);
    }
    for (int i=0; i<img_height_*img_width_; ++i)
    {
        str_rhopts_->pts_per_pixel_index_valid[i].reserve(5000);
    }
    str_rhopts_->pts        = boost::make_shared<pcl::PointCloud<pcl::PointXYZ>>();
    str_rhopts_->img_restore_mask      = cv::Mat::zeros(img_height_, img_width_, CV_32SC1);
    str_rhopts_->img_restore_warp_mask = cv::Mat::zeros(img_height_, img_width_, CV_32SC1);
};

CloudFrame::~CloudFrame()
{
    // destructor
};

void CloudFrame::genRangeImages(pcl::PointCloud<pcl::PointXYZ>::Ptr pcl_in, bool cur_next)
{
    n_pts_ = pcl_in->size();

    // timer::tic();
    calcuateRho(pcl_in, cur_next);    
    // double dt_calRho = timer::toc(); // milliseconds
    // ROS_INFO_STREAM("elapsed time for 'calcuateRho' :" << dt_calRho << " [ms]");

    // timer::tic();
    makeRangeImageAndPtsPerPixel(cur_next);
    // double dt_PPP = timer::toc(); // milliseconds
    // ROS_INFO_STREAM("elapsed time for 'makeRangeImageAndPtsPerPixel' :" << dt_PPP << " [ms]");
    
    // if (cur_next==0)
    // {
    // float* ptr_input_mat = this->str_rhopts_->img_rho.ptr<float>(0);
    // int cnt = 0;
    // for (int i = 0; i < 64; ++i)
    // {
    //     int i_ncols = i * 901;
    //     for (int j = 0; j < 901; ++j)
    //     {
    //         if (*(ptr_input_mat + i_ncols + j) != 0)
    //         {
    //             cnt += 1;
    //         }
    //     }
    // }
    // std::cout<<"# of non zero: "<<cnt <<std::endl;
    // exit(0);
    // }

    // timer::tic();
    interpRangeImage(cur_next);
    // double dt_interpRho = timer::toc(); // milliseconds
    // ROS_INFO_STREAM("elapsed time for 'interpRangeImage' :" << dt_interpRho << " [ms]");

    // timer::tic();
    interpPts(pcl_in, cur_next);
    // double dt_interpPts = timer::toc(); // milliseconds
    // ROS_INFO_STREAM("elapsed time for 'interpPts' :" << dt_interpPts << " [ms]");
}

void CloudFrame::genRangeImages_dR(pcl::PointCloud<pcl::PointXYZ>::Ptr pcl_in, bool cur_next)
{
    n_pts_ = pcl_in->size();

    calcuateRho(pcl_in, cur_next);    

    makeRangeImageAndPtsPerPixel_dR(cur_next);

    interpRangeImage_dR(cur_next);
}

void CloudFrame::genRangeImages_noComp(pcl::PointCloud<pcl::PointXYZ>::Ptr pcl_in, bool cur_next)
{
    n_pts_ = pcl_in->size();

    calcuateRho(pcl_in, cur_next);   

    makeRangeImageAndPtsPerPixel(cur_next);
}

void CloudFrame::calcuateRho(pcl::PointCloud<pcl::PointXYZ>::Ptr pcl_in, bool cur_next)
{
    // timer::tic();

    float twopi         = 2.0*M_PI;
    float offset_theta  = M_PI;

    int n_pts = pcl_in->size();
    float invrhocos = 0.0;
    float cospsi = 0.0;
    float sinpsi = 0.0;

    float* ptr_rho = str_rhopts_->rho.data();
    float* ptr_phi = str_rhopts_->phi.data();
    float* ptr_theta = str_rhopts_->theta.data();

    float M_PI_plus_offset_theta = M_PI + offset_theta;
    float twopi_plus_offset_theta = twopi + offset_theta;

    // std::string line;
    // std::ofstream file("/home/junhakim/rhophitheta.txt");

    for (int i = 0; i < n_pts; ++i)
    {
        // str_rhopts_->rho.push_back(sqrt((pcl_in->points[i].x * pcl_in->points[i].x + pcl_in->points[i].y * pcl_in->points[i].y + pcl_in->points[i].z * pcl_in->points[i].z)));
        str_rhopts_->rho.push_back(NORM(pcl_in->points[i].x, pcl_in->points[i].y, pcl_in->points[i].z));
        str_rhopts_->phi.push_back(asinf32(pcl_in->points[i].z / ptr_rho[i]));
        invrhocos = (float)1.0 / (ptr_rho[i] * cosf32(ptr_phi[i]));

        cospsi = pcl_in->points[i].x * invrhocos;
        if (cospsi > 1)
        {
            // std::cout << "(cospsi > 1): " << cospsi <<std::endl;
            cospsi = (float)1.0;
            
        }
        else if (cospsi < -1)
        {
            // std::cout << "(cospsi < -1): " << cospsi <<std::endl;
            cospsi = -(float)1.0;
        }
        else{}

        sinpsi = pcl_in->points[i].y * invrhocos;

        if (cospsi >= 0)
        {
            if (sinpsi >= 0) // 1 quadrant
            {
                str_rhopts_->theta.push_back(acosf32(cospsi) + offset_theta);
            }
            else // 4 quadrant
            {
                str_rhopts_->theta.push_back(twopi_plus_offset_theta - acosf32(cospsi));
            }
        }
        else
        {
            if (sinpsi >= 0) // 2 quadrant
            {
                str_rhopts_->theta.push_back(M_PI_plus_offset_theta - acosf32(-cospsi));
            }
            else // 3 quadrant
            {
                str_rhopts_->theta.push_back(M_PI_plus_offset_theta + acosf32(-cospsi));
            }
        }

        if (ptr_theta[i] >= twopi)
        {
            ptr_theta[i] = ptr_theta[i] - twopi;
        }
        // std::cout << str_rhopts_->rho[i] << " " << str_rhopts_->phi[i] << " " << str_rhopts_->theta[i]<< std::endl;

        // if (cur_next==1)
        // {
        //     if (file.is_open())
        //     {
        //         file << ptr_rho[i] << " " << ptr_phi[i] << " " << ptr_theta[i] <<"\n";
        //     }
        //     else
        //     {
        //         std::cout << "error" << std::endl;
        //     }
        // }
    }
    // std::cout<<str_rhopts_->rho.size() << std::endl;
    // exit(0);
    // double dt_slam = timer::toc(); // milliseconds
    // ROS_INFO_STREAM("elapsed time for 'calcuateRho' :" << dt_slam << " [ms]");

    // if (cur_next == 1)
    // {
    //     file.close();
    //     exit(0);
    // }
}

void CloudFrame::makeRangeImageAndPtsPerPixel(bool cur_next)
{
    // timer::tic();
    int i_row = 0;
    int i_col = 0;

    float* ptr_img_rho = str_rhopts_->img_rho.ptr<float>(0);
    int* ptr_img_index = str_rhopts_->img_index.ptr<int>(0);

    // int* ptr_pts_per_pixel_n = str_rhopts_->pts_per_pixel_n.data();

    int n_row = str_rhopts_->img_rho.rows;
    int n_col = str_rhopts_->img_rho.cols;

    float* ptr_rho      = str_rhopts_->rho.data();
    float* ptr_phi      = str_rhopts_->phi.data();
    float* ptr_theta    = str_rhopts_->theta.data();
    float* ptr_v_angle  = v_angle_.data();

    float az_step_R2D = az_step_*R2D;

    float phi_R2D = 0.0f;

    int n_ring_minus_1 = n_ring_-1;
    int i_row_ncols_i_col = 0;
    // std::string line;
    // std::ofstream file("/home/junhakim/debug_rowcol.txt");
    
    for (int i = 0; i < n_pts_; ++i)
    {
        phi_R2D = (ptr_phi[i] * R2D);

        if (phi_R2D > 2.5) // 2.5[degree]
        {
            i_row = 0;
        }
        else if (phi_R2D > -8.0) // -8.0[degree]
        {
            i_row = (int)ceil((-2.9523810 * phi_R2D + 7.3809524));
        }
        else if (phi_R2D > -8.5) // -8.5[degree]
        {
            i_row = 32;
        }
        else if (phi_R2D > -23.8) // -23.8[degree]
        {
            i_row = (int)ceil((-2.0261438 * phi_R2D + 14.7777778));
        }
        else
        {
            i_row = 63;
        }
        
        // phi_R2D = (ptr_phi[i] * R2D);

        // for (int kk = 0; kk < n_ring_; ++kk)
        // {
        //     if (ptr_v_angle[kk] < phi_R2D || kk == n_ring_-1)
        //     {
        //         i_row = kk;
        //         break;
        //     }
        // }

        i_col = roundf(ptr_theta[i]*az_step_R2D);

        if ( (i_row > n_ring_minus_1) || (i_row < 0) )
        {
            continue;
        }

        // if (cur_next==1)
        // {
        //     if (file.is_open())
        //     {
        //         file << i_row << " " << i_col << "\n";
        //     }
        //     else
        //     {
        //         std::cout << "error" << std::endl;
        //     }
        // }
        
        i_row_ncols_i_col = i_row * n_col + i_col;
        if (*(ptr_img_rho + i_row_ncols_i_col) == 0 || *(ptr_img_rho + i_row_ncols_i_col) > ptr_rho[i]) //(str_rhopts_->img_rho.at<float>(i_row,i_col) == 0)
        {   
            *(ptr_img_rho + i_row_ncols_i_col) = ptr_rho[i];
            *(ptr_img_index + i_row_ncols_i_col) = i;
        }
        // else if (*(ptr_img_rho + i_row_ncols_i_col) > ptr_rho[i])
        // {
        //     *(ptr_img_rho + i_row_ncols_i_col) = ptr_rho[i];
        //     *(ptr_img_index + i_row_ncols_i_col) = i;
        // }
        else{}

        // ptr_pts_per_pixel_n[i_row_ncols_i_col] += 1;
        str_rhopts_->pts_per_pixel_index[i_row_ncols_i_col].push_back(i);
        str_rhopts_->pts_per_pixel_rho[i_row_ncols_i_col].push_back(ptr_rho[i]);
    } // end for
                
                // if (cur_next == 1)
                // {   
                //     file.close();
                //     exit(0);
                // }

    // cv::FileStorage fs_w("/home/junhakim/asdf.yaml", cv::FileStorage::WRITE);
    // fs_w << "matImage" << str_rhopts_->img_rho;
    // fs_w.release();
    // exit(0);

    // cnt = 0;
    // for (int i = 0; i < n_row; ++i)
    // {
    //     int i_ncols = i * n_col;
    //     for (int j = 0; j < n_col; ++j)
    //     {
    //         if (*(ptr_img_index + i_ncols + j) != 0)
    //         {
    //             cnt += 1;
    //         }
    //     }
    // }
    // std::cout<<"# of non zero: "<<cnt <<std::endl;
    // exit(0);
    
    // for(int a=0; a < str_rhopts_->pts_per_pixel_rho.size(); ++a)
    // {
    //     // std::cout << str_rhopts_->pts_per_pixel_n[a] << std::endl;
    //     for (int b =0; b< str_rhopts_->pts_per_pixel_rho[a].size(); ++b)
    //     {
    //         std::cout << str_rhopts_->pts_per_pixel_rho[a][b] << " " ;
    //     }
    //     std::cout << std::endl;
    // }
    // exit(0);
    // double dt_slam = timer::toc(); // milliseconds
    // ROS_INFO_STREAM("elapsed time for 'makeRangeImageAndPtsPerPixel' :" << dt_slam << " [ms]");
}

void CloudFrame::makeRangeImageAndPtsPerPixel_dR(bool cur_next)
{
    int i_row = 0;
    int i_col = 0;

    float* ptr_img_rho = str_rhopts_->img_rho.ptr<float>(0);

    int n_row = str_rhopts_->img_rho.rows;
    int n_col = str_rhopts_->img_rho.cols;

    float* ptr_rho      = str_rhopts_->rho.data();
    float* ptr_phi      = str_rhopts_->phi.data();
    float* ptr_theta    = str_rhopts_->theta.data();
    float* ptr_v_angle  = v_angle_.data();

    float az_step_R2D = az_step_*R2D;

    float phi_R2D = 0.0f;

    int n_ring_minus_1 = n_ring_-1;

    for (int i = 0; i < n_pts_; ++i)
    {
        phi_R2D = (ptr_phi[i] * R2D);

        if (phi_R2D > 2.5) // 2.5[degree]
        {
            i_row = 0;
        }
        else if (phi_R2D > -8.0) // -8.0[degree]
        {
            i_row = (int)ceil((-2.9523810 * phi_R2D + 7.3809524));
        }
        else if (phi_R2D > -8.5) // -8.5[degree]
        {
            i_row = 32;
        }
        else if (phi_R2D > -23.8) // -23.8[degree]
        {
            i_row = (int)ceil((-2.0261438 * phi_R2D + 14.7777778));
        }
        else
        {
            i_row = 63;
        }

        i_col = roundf(ptr_theta[i]*az_step_R2D);

        if ( (i_row > n_ring_minus_1) || (i_row < 0) )
        {
            continue;
        }
        
        int i_row_ncols_i_col = i_row * n_col + i_col;
        if (*(ptr_img_rho + i_row_ncols_i_col) == 0 || *(ptr_img_rho + i_row_ncols_i_col) > ptr_rho[i]) //(str_rhopts_->img_rho.at<float>(i_row,i_col) == 0)
        {   
            *(ptr_img_rho + i_row_ncols_i_col) = ptr_rho[i];
        }
        else{}
    } // end for
}

void CloudFrame::interpRangeImage(bool cur_next)
{    
    int n_col = str_rhopts_->img_rho.cols;
    int n_row = str_rhopts_->img_rho.rows;

    cv::Mat img_rho_new = str_rhopts_->img_rho.clone();
    float* ptr_img_rho_new = img_rho_new.ptr<float>(0);

    float* ptr_img_rho          = str_rhopts_->img_rho.ptr<float>(0);
    int* ptr_img_index          = str_rhopts_->img_index.ptr<int>(0);
    int* ptr_img_restore_mask   = str_rhopts_->img_restore_mask.ptr<int>(0);

    int i_ncols = 0;
    int i_minus_ncols = 0;
    int i_plus_ncols = 0;

    for (int i = 27; i < 32; i++)
    {
        i_ncols = i * n_col;
        i_minus_ncols = (i - 1) * n_col;
        i_plus_ncols  = (i + 1) * n_col;
        for (int j = 0 + 2; j < (n_radial_ - 2); ++j)
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
                            if (cur_next == false)
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
                        if (fabsf32(*(ptr_img_rho + i_minus_ncols + j) - *(ptr_img_rho + i_plus_ncols + n_col + j)) < 0.1)
                        {
                            *(ptr_img_restore_mask + i_ncols + j) = 2;
                            *(ptr_img_restore_mask + i_plus_ncols + j) = 3;
                            *(ptr_img_rho_new + i_ncols + j)        = *(ptr_img_rho + i_minus_ncols + j) * (0.6666667) + *(ptr_img_rho + i_plus_ncols + n_col + j) * (0.3333333);
                            *(ptr_img_rho_new + i_plus_ncols + j)   = *(ptr_img_rho + i_minus_ncols + j) * (0.3333333) + *(ptr_img_rho + i_plus_ncols + n_col + j) * (0.6666667);
                        }
                        else
                        {
                            *(ptr_img_restore_mask + i_ncols + j) = 20;
                            *(ptr_img_restore_mask + i_plus_ncols + j) = 30;
                            if (cur_next == false)
                            {
                                *(ptr_img_rho_new + i_ncols + j)        = std::min(*(ptr_img_rho + i_minus_ncols + j), *(ptr_img_rho + i_plus_ncols + n_col + j));
                                *(ptr_img_rho_new + i_plus_ncols + j)   = std::min(*(ptr_img_rho + i_minus_ncols + j), *(ptr_img_rho + i_plus_ncols + n_col + j));
                            }
                            else
                            {
                                *(ptr_img_rho_new + i_ncols + j)        = std::max(*(ptr_img_rho + i_minus_ncols + j), *(ptr_img_rho + i_plus_ncols + n_col + j));
                                *(ptr_img_rho_new + i_plus_ncols + j)   = std::max(*(ptr_img_rho + i_minus_ncols + j), *(ptr_img_rho + i_plus_ncols + n_col + j));
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

    img_rho_new.copyTo(str_rhopts_->img_rho);
}

void CloudFrame::interpRangeImage_dR(bool cur_next)
{    
    int n_col = str_rhopts_->img_rho.cols;
    int n_row = str_rhopts_->img_rho.rows;

    cv::Mat img_rho_new = str_rhopts_->img_rho.clone();
    float* ptr_img_rho_new = img_rho_new.ptr<float>(0);

    float* ptr_img_rho          = str_rhopts_->img_rho.ptr<float>(0);

    for (int i = 27; i < 32; i++)
    {
        int i_ncols = i * n_col;
        int i_minus_ncols = (i - 1) * n_col;
        int i_plus_ncols  = (i + 1) * n_col;
        for (int j = 0 + 2; j < (n_radial_ - 2); ++j)
        {
            if (*(ptr_img_rho + i_ncols + j) == 0)
            {
                if ((*(ptr_img_rho + i_minus_ncols + j) != 0))
                {
                    if ((*(ptr_img_rho + i_plus_ncols + j) != 0))
                    {
                        if (fabsf32(*(ptr_img_rho + i_minus_ncols + j) - *(ptr_img_rho + i_plus_ncols + j)) < 0.1)
                        {
                            *(ptr_img_rho_new + i_ncols + j) = (*(ptr_img_rho + i_minus_ncols + j) + *(ptr_img_rho + i_plus_ncols + j)) * 0.5;
                        }
                        else
                        {
                            if (cur_next == false)
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
                        if (fabsf32(*(ptr_img_rho + i_minus_ncols + j) - *(ptr_img_rho + i_plus_ncols + n_col + j)) < 0.1)
                        {
                            *(ptr_img_rho_new + i_ncols + j)        = *(ptr_img_rho + i_minus_ncols + j) * (0.6666667) + *(ptr_img_rho + i_plus_ncols + n_col + j) * (0.3333333);
                            *(ptr_img_rho_new + i_plus_ncols + j)   = *(ptr_img_rho + i_minus_ncols + j) * (0.3333333) + *(ptr_img_rho + i_plus_ncols + n_col + j) * (0.6666667);
                        }
                        else
                        {
                            if (cur_next == false)
                            {
                                *(ptr_img_rho_new + i_ncols + j)        = std::min(*(ptr_img_rho + i_minus_ncols + j), *(ptr_img_rho + i_plus_ncols + n_col + j));
                                *(ptr_img_rho_new + i_plus_ncols + j)   = std::min(*(ptr_img_rho + i_minus_ncols + j), *(ptr_img_rho + i_plus_ncols + n_col + j));
                            }
                            else
                            {
                                *(ptr_img_rho_new + i_ncols + j)        = std::max(*(ptr_img_rho + i_minus_ncols + j), *(ptr_img_rho + i_plus_ncols + n_col + j));
                                *(ptr_img_rho_new + i_plus_ncols + j)   = std::max(*(ptr_img_rho + i_minus_ncols + j), *(ptr_img_rho + i_plus_ncols + n_col + j));
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
                        *(ptr_img_rho_new + i_ncols + j) = (*(ptr_img_rho + i_ncols + (j - 1)) + *(ptr_img_rho + i_ncols + (j + 1))) * 0.5;
                    }
                    else{}
                }
                else if ((*(ptr_img_rho + i_ncols + (j + 2)) != 0))
                {
                    if (fabsf32(*(ptr_img_rho + i_ncols + (j - 1)) - *(ptr_img_rho + i_ncols + (j + 2))) < 0.05)
                    {
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

    img_rho_new.copyTo(str_rhopts_->img_rho);
}

void CloudFrame::interpPts(pcl::PointCloud<pcl::PointXYZ>::Ptr pcl_in, bool cur_next)
{
    int n_row = str_rhopts_->img_rho.rows;
    int n_col = str_rhopts_->img_rho.cols;
    
    float* ptr_img_rho = str_rhopts_->img_rho.ptr<float>(0);
    int* ptr_img_index = str_rhopts_->img_index.ptr<int>(0);
    float* ptr_img_x = str_rhopts_->img_x.ptr<float>(0);
    float* ptr_img_y = str_rhopts_->img_y.ptr<float>(0);
    float* ptr_img_z = str_rhopts_->img_z.ptr<float>(0);
    int* ptr_img_restore_mask = str_rhopts_->img_restore_mask.ptr<int>(0);

    for (int i = 0; i < n_ring_; ++i)
    {
        int i_ncols = i * n_col;
        for (int j = 0; j < n_radial_; ++j)
        {
            int i_ncols_j = i_ncols + j;
            if (str_rhopts_->pts_per_pixel_rho[i_ncols_j].size() > 0)
            {
                for (int k = 0; k < (str_rhopts_->pts_per_pixel_rho[i_ncols_j].size()); ++k)
                {
                    if (std::abs((str_rhopts_->pts_per_pixel_rho[i_ncols_j][k] - *(ptr_img_rho + i_ncols_j))) < 0.5)
                    {
                        str_rhopts_->pts_per_pixel_index_valid[i_ncols_j].push_back(str_rhopts_->pts_per_pixel_index[i_ncols_j][k]);
                    }
                    else{}
                }
            } // end if
            else{}

            if (*(ptr_img_index + i_ncols_j) != 0)
            {
                *(ptr_img_x + i_ncols_j) = (*pcl_in)[*(ptr_img_index + i_ncols_j)].x;
                *(ptr_img_y + i_ncols_j) = (*pcl_in)[*(ptr_img_index + i_ncols_j)].y;
                *(ptr_img_z + i_ncols_j) = (*pcl_in)[*(ptr_img_index + i_ncols_j)].z;
            }
            else{}

            switch(*(ptr_img_restore_mask + i_ncols_j))
            {
                case 1:
                    *(ptr_img_x + i_ncols_j) = 0.5 * ((*pcl_in)[*(ptr_img_index + i_ncols_j - n_col)].x + (*pcl_in)[*(ptr_img_index + i_ncols_j + n_col)].x);
                    *(ptr_img_y + i_ncols_j) = 0.5 * ((*pcl_in)[*(ptr_img_index + i_ncols_j - n_col)].y + (*pcl_in)[*(ptr_img_index + i_ncols_j + n_col)].y);
                    *(ptr_img_z + i_ncols_j) = 0.5 * ((*pcl_in)[*(ptr_img_index + i_ncols_j - n_col)].z + (*pcl_in)[*(ptr_img_index + i_ncols_j + n_col)].z);
                    break;
                case 10:
                    if (cur_next == false)
                    {
                        if ((*(ptr_img_rho + i_ncols_j - n_col) > *(ptr_img_rho + i_ncols_j + n_col)))
                        {
                            *(ptr_img_x + i_ncols_j) = (*pcl_in)[*(ptr_img_index + i_ncols_j + n_col)].x;
                            *(ptr_img_y + i_ncols_j) = (*pcl_in)[*(ptr_img_index + i_ncols_j + n_col)].y;
                            *(ptr_img_z + i_ncols_j) = (*pcl_in)[*(ptr_img_index + i_ncols_j + n_col)].z;
                        }
                        else
                        {
                            *(ptr_img_x + i_ncols_j) = (*pcl_in)[*(ptr_img_index + i_ncols_j - n_col)].x;
                            *(ptr_img_y + i_ncols_j) = (*pcl_in)[*(ptr_img_index + i_ncols_j - n_col)].y;
                            *(ptr_img_z + i_ncols_j) = (*pcl_in)[*(ptr_img_index + i_ncols_j - n_col)].z;            
                        }
                        break;
                    }
                    else
                    {
                        if ((*(ptr_img_rho + i_ncols_j - n_col) < *(ptr_img_rho + i_ncols_j + n_col)))
                        {
                            *(ptr_img_x + i_ncols_j) = (*pcl_in)[*(ptr_img_index + i_ncols_j + n_col)].x;
                            *(ptr_img_y + i_ncols_j) = (*pcl_in)[*(ptr_img_index + i_ncols_j + n_col)].y;
                            *(ptr_img_z + i_ncols_j) = (*pcl_in)[*(ptr_img_index + i_ncols_j + n_col)].z;
                        }
                        else
                        {
                            *(ptr_img_x + i_ncols_j) = (*pcl_in)[*(ptr_img_index + i_ncols_j - n_col)].x;
                            *(ptr_img_y + i_ncols_j) = (*pcl_in)[*(ptr_img_index + i_ncols_j - n_col)].y;
                            *(ptr_img_z + i_ncols_j) = (*pcl_in)[*(ptr_img_index + i_ncols_j - n_col)].z;            
                        }
                        break;
                    }

                case 2:
                    *(ptr_img_x + i_ncols_j) = (0.6666667) * (*pcl_in)[*(ptr_img_index + i_ncols_j - n_col)].x + (0.3333333) * (*pcl_in)[*(ptr_img_index + i_ncols_j + 2 * n_col)].x;
                    *(ptr_img_y + i_ncols_j) = (0.6666667) * (*pcl_in)[*(ptr_img_index + i_ncols_j - n_col)].y + (0.3333333) * (*pcl_in)[*(ptr_img_index + i_ncols_j + 2 * n_col)].y;
                    *(ptr_img_z + i_ncols_j) = (0.6666667) * (*pcl_in)[*(ptr_img_index + i_ncols_j - n_col)].z + (0.3333333) * (*pcl_in)[*(ptr_img_index + i_ncols_j + 2 * n_col)].z;
                    break;
                case 20:
                    if (cur_next == false)
                    {
                        if ((*(ptr_img_rho + i_ncols_j - n_col) > *(ptr_img_rho + i_ncols_j + 2 * n_col)))
                        {                            
                            *(ptr_img_x + i_ncols_j) = (*pcl_in)[*(ptr_img_index + i_ncols_j + 2 * n_col)].x;
                            *(ptr_img_y + i_ncols_j) = (*pcl_in)[*(ptr_img_index + i_ncols_j + 2 * n_col)].y;
                            *(ptr_img_z + i_ncols_j) = (*pcl_in)[*(ptr_img_index + i_ncols_j + 2 * n_col)].z;
                        }
                        else
                        {
                            *(ptr_img_x + i_ncols_j) = (*pcl_in)[*(ptr_img_index + i_ncols_j - n_col)].x;
                            *(ptr_img_y + i_ncols_j) = (*pcl_in)[*(ptr_img_index + i_ncols_j - n_col)].y;
                            *(ptr_img_z + i_ncols_j) = (*pcl_in)[*(ptr_img_index + i_ncols_j - n_col)].z;
                        }
                        break;
                    }
                    else
                    {
                        if ((*(ptr_img_rho + i_ncols_j - n_col) < *(ptr_img_rho + i_ncols_j + 2 * n_col)))
                        {
                            *(ptr_img_x + i_ncols_j) = (*pcl_in)[*(ptr_img_index + i_ncols_j + 2 * n_col)].x;
                            *(ptr_img_y + i_ncols_j) = (*pcl_in)[*(ptr_img_index + i_ncols_j + 2 * n_col)].y;
                            *(ptr_img_z + i_ncols_j) = (*pcl_in)[*(ptr_img_index + i_ncols_j + 2 * n_col)].z;
                        }
                        else
                        {
                            *(ptr_img_x + i_ncols_j) = (*pcl_in)[*(ptr_img_index + i_ncols_j - n_col)].x;
                            *(ptr_img_y + i_ncols_j) = (*pcl_in)[*(ptr_img_index + i_ncols_j - n_col)].y;
                            *(ptr_img_z + i_ncols_j) = (*pcl_in)[*(ptr_img_index + i_ncols_j - n_col)].z;
                        }
                        break;
                    }

                case 3:
                    *(ptr_img_x + i_ncols_j) = (0.3333333) * (*pcl_in)[*(ptr_img_index + i_ncols_j - 2 * n_col)].x + (0.6666667) * (*pcl_in)[*(ptr_img_index + i_ncols_j + n_col)].x;
                    *(ptr_img_y + i_ncols_j) = (0.3333333) * (*pcl_in)[*(ptr_img_index + i_ncols_j - 2 * n_col)].y + (0.6666667) * (*pcl_in)[*(ptr_img_index + i_ncols_j + n_col)].y;
                    *(ptr_img_z + i_ncols_j) = (0.3333333) * (*pcl_in)[*(ptr_img_index + i_ncols_j - 2 * n_col)].z + (0.6666667) * (*pcl_in)[*(ptr_img_index + i_ncols_j + n_col)].z;
                    break;
                case 30:
                    if(cur_next == false)
                    {
                        if ((*(ptr_img_rho + i_ncols_j - 2 * n_col) > *(ptr_img_rho + i_ncols_j + n_col)))
                        {
                            *(ptr_img_x + i_ncols_j) = (*pcl_in)[*(ptr_img_index + i_ncols_j + n_col)].x;
                            *(ptr_img_y + i_ncols_j) = (*pcl_in)[*(ptr_img_index + i_ncols_j + n_col)].y;
                            *(ptr_img_z + i_ncols_j) = (*pcl_in)[*(ptr_img_index + i_ncols_j + n_col)].z;
                        }
                        else
                        {
                            *(ptr_img_x + i_ncols_j) = (*pcl_in)[*(ptr_img_index + i_ncols_j - 2 * n_col)].x;
                            *(ptr_img_y + i_ncols_j) = (*pcl_in)[*(ptr_img_index + i_ncols_j - 2 * n_col)].y;
                            *(ptr_img_z + i_ncols_j) = (*pcl_in)[*(ptr_img_index + i_ncols_j - 2 * n_col)].z;
                        }
                        break;
                    }
                    else
                    {
                        if ((*(ptr_img_rho + i_ncols_j - 2 * n_col) < *(ptr_img_rho + i_ncols_j + n_col)))
                        {
                            *(ptr_img_x + i_ncols_j) = (*pcl_in)[*(ptr_img_index + i_ncols_j + n_col)].x;
                            *(ptr_img_y + i_ncols_j) = (*pcl_in)[*(ptr_img_index + i_ncols_j + n_col)].y;
                            *(ptr_img_z + i_ncols_j) = (*pcl_in)[*(ptr_img_index + i_ncols_j + n_col)].z;
                        }
                        else
                        {
                            *(ptr_img_x + i_ncols_j) = (*pcl_in)[*(ptr_img_index + i_ncols_j - 2 * n_col)].x;
                            *(ptr_img_y + i_ncols_j) = (*pcl_in)[*(ptr_img_index + i_ncols_j - 2 * n_col)].y;
                            *(ptr_img_z + i_ncols_j) = (*pcl_in)[*(ptr_img_index + i_ncols_j - 2 * n_col)].z;
                        }
                        break;
                    }

                case 4:
                    *(ptr_img_x + i_ncols_j) = 0.5 * ((*pcl_in)[*(ptr_img_index + i_ncols_j - 1)].x + (*pcl_in)[*(ptr_img_index + i_ncols_j + 1)].x);
                    *(ptr_img_y + i_ncols_j) = 0.5 * ((*pcl_in)[*(ptr_img_index + i_ncols_j - 1)].y + (*pcl_in)[*(ptr_img_index + i_ncols_j + 1)].y);
                    *(ptr_img_z + i_ncols_j) = 0.5 * ((*pcl_in)[*(ptr_img_index + i_ncols_j - 1)].z + (*pcl_in)[*(ptr_img_index + i_ncols_j + 1)].z);
                    break;
                case 5:
                    *(ptr_img_x + i_ncols_j) = (0.6666667) * (*pcl_in)[*(ptr_img_index + i_ncols_j - 1)].x + (0.3333333) * (*pcl_in)[*(ptr_img_index + i_ncols_j + 2)].x;
                    *(ptr_img_y + i_ncols_j) = (0.6666667) * (*pcl_in)[*(ptr_img_index + i_ncols_j - 1)].y + (0.3333333) * (*pcl_in)[*(ptr_img_index + i_ncols_j + 2)].y;
                    *(ptr_img_z + i_ncols_j) = (0.6666667) * (*pcl_in)[*(ptr_img_index + i_ncols_j - 1)].z + (0.3333333) * (*pcl_in)[*(ptr_img_index + i_ncols_j + 2)].z;
                    break;
                case 6:
                    *(ptr_img_x + i_ncols_j) = (0.3333333) * (*pcl_in)[*(ptr_img_index + i_ncols_j - 2)].x + (0.6666667) * (*pcl_in)[*(ptr_img_index + i_ncols_j + 1)].x;
                    *(ptr_img_y + i_ncols_j) = (0.3333333) * (*pcl_in)[*(ptr_img_index + i_ncols_j - 2)].y + (0.6666667) * (*pcl_in)[*(ptr_img_index + i_ncols_j + 1)].y;
                    *(ptr_img_z + i_ncols_j) = (0.3333333) * (*pcl_in)[*(ptr_img_index + i_ncols_j - 2)].z + (0.6666667) * (*pcl_in)[*(ptr_img_index + i_ncols_j + 1)].z;
                    break;
            }
        } // end for j    
    } // end for i       
    // cv::FileStorage fs_w("/home/junhakim/img_x.yaml", cv::FileStorage::WRITE);
    // fs_w << "matImage" << str_rhopts_->img_x;
    // fs_w.release();
    // cv::FileStorage fs_s("/home/junhakim/img_y.yaml", cv::FileStorage::WRITE);
    // fs_s << "matImage" << str_rhopts_->img_y;
    // fs_s.release();
    // cv::FileStorage fs_q("/home/junhakim/img_z.yaml", cv::FileStorage::WRITE);
    // fs_q << "matImage" << str_rhopts_->img_z;
    // fs_q.release();
    // exit(0); 
}

void CloudFrame::reset()
{
    this->str_rhopts_->rho.resize(0);
    this->str_rhopts_->phi.resize(0);
    this->str_rhopts_->theta.resize(0);
    this->str_rhopts_->img_rho = cv::Mat::zeros(img_height_, img_width_, CV_32FC1);
    this->str_rhopts_->img_index = cv::Mat::zeros(img_height_, img_width_, CV_32SC1);
    this->str_rhopts_->img_x = cv::Mat::zeros(img_height_, img_width_, CV_32FC1);
    this->str_rhopts_->img_y = cv::Mat::zeros(img_height_, img_width_, CV_32FC1);
    this->str_rhopts_->img_z = cv::Mat::zeros(img_height_, img_width_, CV_32FC1);
    this->str_rhopts_->img_restore_mask = cv::Mat::zeros(img_height_, img_width_, CV_32SC1);
    this->str_rhopts_->img_restore_warp_mask = cv::Mat::zeros(img_height_, img_width_, CV_32SC1);

    // for (int i = 0; i < img_height_ * img_width_; ++i)
    // {
    //     this->str_rhopts_->pts_per_pixel_n[i] = 0;
    // }

    for (int i = 0; i < img_height_ * img_width_; ++i)
    {
        if (this->str_rhopts_->pts_per_pixel_index[i].size() != 0)
        {
            this->str_rhopts_->pts_per_pixel_index[i].resize(0);
        }
    }

    for (int i = 0; i < img_height_ * img_width_; ++i)
    {
        if (this->str_rhopts_->pts_per_pixel_rho[i].size() != 0)
        {
            this->str_rhopts_->pts_per_pixel_rho[i].resize(0);
        }
    }

    for (int i = 0; i < img_height_ * img_width_; ++i)
    {
        if (this->str_rhopts_->pts_per_pixel_index_valid[i].size() != 0)
        {
            this->str_rhopts_->pts_per_pixel_index_valid[i].resize(0);
        }
    }
}