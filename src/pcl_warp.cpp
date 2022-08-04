#include "pcl_warp.h"

PclWarp::PclWarp(const std::unique_ptr<UserParam>& user_param)
{
    img_height_ = user_param->image_param_.height_;
    img_width_ = user_param->image_param_.width_;

    str_warpPointcloud_ = new StrRhoPts();

    str_warpPointcloud_->rho.reserve(500000);
    str_warpPointcloud_->phi.reserve(500000);
    str_warpPointcloud_->theta.reserve(500000);
    str_warpPointcloud_->img_rho   = cv::Mat::zeros(img_height_, img_width_, CV_32FC1);
    str_warpPointcloud_->img_index = cv::Mat::zeros(img_height_, img_width_, CV_32SC1);

    str_warpPointcloud_->img_x = cv::Mat::zeros(img_height_, img_width_, CV_32FC1);
    str_warpPointcloud_->img_y = cv::Mat::zeros(img_height_, img_width_, CV_32FC1);
    str_warpPointcloud_->img_z = cv::Mat::zeros(img_height_, img_width_, CV_32FC1);

    str_warpPointcloud_->pts_per_pixel_n.resize(img_height_ * img_width_);
    str_warpPointcloud_->pts_per_pixel_index.resize(img_height_ * img_width_);
    str_warpPointcloud_->pts_per_pixel_rho.resize(img_height_ * img_width_);
    str_warpPointcloud_->pts_per_pixel_index_valid.resize(img_height_ * img_width_);
    for (int i=0; i<img_height_*img_width_; ++i)
    {
        str_warpPointcloud_->pts_per_pixel_index[i].reserve(5000);
    }
    for (int i=0; i<img_height_*img_width_; ++i)
    {
        str_warpPointcloud_->pts_per_pixel_rho[i].reserve(5000);
    }
    for (int i=0; i<img_height_*img_width_; ++i)
    {
        str_warpPointcloud_->pts_per_pixel_index_valid[i].reserve(5000);
    }
    str_warpPointcloud_->pts        = boost::make_shared<pcl::PointCloud<pcl::PointXYZ>>();
    str_warpPointcloud_->ptsInImage = boost::make_shared<pcl::PointCloud<pcl::PointXYZ>>();
    str_warpPointcloud_->img_restore_mask = cv::Mat::zeros(img_height_, img_width_, CV_32SC1);
    str_warpPointcloud_->img_restore_warp_mask = cv::Mat::zeros(img_height_, img_width_, CV_32SC1);

    velo_xyz_.reserve(500000);

    pts_warpewd_        = boost::make_shared<pcl::PointCloud<pcl::PointXYZ>>();

};

PclWarp::~PclWarp()
{

};

void PclWarp::warpPointcloud(StrRhoPts *str_cur, const Pose &T01, /*output*/ cv::Mat &mat_in, int cnt_data, std::unique_ptr<CloudFrame>& CloudFrame)
{
    int n_row = str_cur->img_rho.rows;
    int n_col = str_cur->img_rho.cols;
    float *ptr_mat_in = mat_in.ptr<float>(0);
    float *ptr_img_x = str_cur->img_x.ptr<float>(0);
    float *ptr_img_y = str_cur->img_y.ptr<float>(0);
    float *ptr_img_z = str_cur->img_z.ptr<float>(0);
    float *ptr_img_rho = str_cur->img_rho.ptr<float>(0);
    
    cv::Mat mat_out = cv::Mat::zeros(img_height_, img_width_, CV_32FC1);
    int *ptr_warp_img_index = str_warpPointcloud_->img_index.ptr<int>(0);
    float *ptr_mat_out = mat_out.ptr<float>(0);

    std::vector<float> I_vec1;
    I_vec1.reserve(n_row * n_col);
    bool isempty_flag = 0;

    for (int i = 0; i < n_row; ++i)
    {
        int i_ncols = i * n_col;
        for (int j = 0; j < n_col; ++j)
        {            
            if (*(ptr_mat_in + i_ncols + j) != 0 && *(ptr_img_rho + i_ncols + j) != 0)
            {
                velo_xyz_.push_back(pcl::PointXYZ(*(ptr_img_x + i_ncols + j), *(ptr_img_y + i_ncols + j), *(ptr_img_z + i_ncols + j)));

                I_vec1.push_back(*(ptr_mat_in + i_ncols + j));
                isempty_flag = 1;
            }
        } //end for i
    }     // end for j

    if (isempty_flag == 0)
    {
        return;
    }

    pcl::transformPointCloud(velo_xyz_, *pts_warpewd_, T01);

    CloudFrame->genRangeImages(*pts_warpewd_, str_warpPointcloud_, 0);
    // countZerofloat(str_warpPointcloud_->img_index);

    // int cnt_debug = 0;

    for (int i = 0; i < n_row; ++i)
    {
        int i_ncols = i * n_col;
        for (int j = 0; j < n_col; ++j)
        {            
            if (*(ptr_warp_img_index + i_ncols + j) != 0)
            {
                *(ptr_mat_out + i_ncols + j) = I_vec1[*(ptr_warp_img_index + i_ncols + j)];
                // std::cout<<cnt_debug << " "<< *(ptr_warp_img_index + i_ncols + j)<<std::endl;
                // cnt_debug += 1;
            } // end if
        } // end for j
    } //end for i
    // countZerofloat(mat_in);
    // if (cnt_data==3)
    // {
    //     exit(0);
    // }

    // ptr_mat_out = ptr_mat_in;
    mat_out.copyTo(mat_in);
}

void PclWarp::initializeStructAndPcl()
{
    {
        str_warpPointcloud_->rho.resize(0);
        str_warpPointcloud_->phi.resize(0);
        str_warpPointcloud_->theta.resize(0);
        str_warpPointcloud_->img_rho               = cv::Mat::zeros(img_height_, img_width_, CV_32FC1);
        str_warpPointcloud_->img_index             = cv::Mat::zeros(img_height_, img_width_, CV_32SC1);
        str_warpPointcloud_->img_x                 = cv::Mat::zeros(img_height_, img_width_, CV_32FC1);
        str_warpPointcloud_->img_y                 = cv::Mat::zeros(img_height_, img_width_, CV_32FC1);
        str_warpPointcloud_->img_z                 = cv::Mat::zeros(img_height_, img_width_, CV_32FC1);
        str_warpPointcloud_->img_restore_mask      = cv::Mat::zeros(img_height_, img_width_, CV_32SC1);
        str_warpPointcloud_->img_restore_warp_mask = cv::Mat::zeros(img_height_, img_width_, CV_32SC1);

        for (int i=0; i<img_height_*img_width_; ++i)
        {
            str_warpPointcloud_->pts_per_pixel_n[i] = 0;
        }

        for (int i=0; i<img_height_*img_width_; ++i)
        {
            if (str_warpPointcloud_->pts_per_pixel_index[i].size() != 0)
            {
                str_warpPointcloud_->pts_per_pixel_index[i].resize(0);
            }
        }

        for (int i=0; i<img_height_*img_width_; ++i)
        {
            if (str_warpPointcloud_->pts_per_pixel_rho[i].size() != 0)
            {
                str_warpPointcloud_->pts_per_pixel_rho[i].resize(0);
            }
        }

        for (int i=0; i<img_height_*img_width_; ++i)
        {
            if (str_warpPointcloud_->pts_per_pixel_index_valid[i].size() != 0)
            {
                str_warpPointcloud_->pts_per_pixel_index_valid[i].resize(0);
            }
        }
    }

    velo_xyz_.resize(0);
    pts_warpewd_->resize(0);
}