#include "pcl_warp.h"

PclWarp::PclWarp(const std::unique_ptr<UserParam>& user_param)
{
    img_height_ = user_param->image_param_.height_;
    img_width_  = user_param->image_param_.width_;

    velo_xyz_       = boost::make_shared<pcl::PointCloud<pcl::PointXYZ>>();
    pts_warpewd_    = boost::make_shared<pcl::PointCloud<pcl::PointXYZ>>();

};

PclWarp::~PclWarp()
{

};

void PclWarp::warpPointcloud(std::unique_ptr<CloudFrame>& CloudFrame_in, std::unique_ptr<CloudFrame>& CloudFrame_warpPointcloud, const Pose &T01, 
                            /*output*/ cv::Mat& mat_in, int cnt_data)
{
    int n_row = CloudFrame_in->str_rhopts_->img_rho.rows;
    int n_col = CloudFrame_in->str_rhopts_->img_rho.cols;
    float *ptr_mat_in = mat_in.ptr<float>(0);
    float *ptr_img_x    = CloudFrame_in->str_rhopts_->img_x.ptr<float>(0);
    float *ptr_img_y    = CloudFrame_in->str_rhopts_->img_y.ptr<float>(0);
    float *ptr_img_z    = CloudFrame_in->str_rhopts_->img_z.ptr<float>(0);
    float *ptr_img_rho  = CloudFrame_in->str_rhopts_->img_rho.ptr<float>(0);
    
    cv::Mat mat_out = cv::Mat::zeros(img_height_, img_width_, CV_32FC1);
    float *ptr_mat_out = mat_out.ptr<float>(0);

    int *ptr_warp_img_index = CloudFrame_warpPointcloud->str_rhopts_->img_index.ptr<int>(0);
    
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
                velo_xyz_->push_back(pcl::PointXYZ(*(ptr_img_x + i_ncols + j), *(ptr_img_y + i_ncols + j), *(ptr_img_z + i_ncols + j)));
                
                I_vec1.push_back(*(ptr_mat_in + i_ncols + j));
                isempty_flag = 1;
            }
        } //end for i
    }     // end for j

    if (isempty_flag == 0)
    {
        return;
    }
    pcl::transformPointCloud(*velo_xyz_, *pts_warpewd_, T01);

    CloudFrame_warpPointcloud->genRangeImages_noComp(pts_warpewd_, false);

    for (int i = 0; i < n_row; ++i)
    {
        int i_ncols = i * n_col;
        for (int j = 0; j < n_col; ++j)
        {            
            if (*(ptr_warp_img_index + i_ncols + j) != 0)
            {
                *(ptr_mat_out + i_ncols + j) = I_vec1[*(ptr_warp_img_index + i_ncols + j)];
            } // end if
        } // end for j
    } //end for i

    mat_out.copyTo(mat_in);

    // if (cnt_data == 2)
    // {
    //     int cnt = 0;
    //     for (int i = 0; i < n_row; ++i)
    //     {
    //         int i_ncols = i * n_col;
    //         for (int j = 0; j < n_col; ++j)
    //         {
    //             if (*(ptr_mat_out + i_ncols + j) != 0)
    //             {
    //                 cnt += 1;
    //             }
    //         }
    //     }
    //     std::cout << "# of non zero: " << cnt << std::endl;
    //     exit(0);
    // }
}

void PclWarp::initializeStructAndPcl(std::unique_ptr<CloudFrame>& CloudFrame_warpPointcloud)
{
    {
        CloudFrame_warpPointcloud->str_rhopts_->rho.resize(0);
        CloudFrame_warpPointcloud->str_rhopts_->phi.resize(0);
        CloudFrame_warpPointcloud->str_rhopts_->theta.resize(0);
        CloudFrame_warpPointcloud->str_rhopts_->img_rho               = cv::Mat::zeros(img_height_, img_width_, CV_32FC1);
        CloudFrame_warpPointcloud->str_rhopts_->img_index             = cv::Mat::zeros(img_height_, img_width_, CV_32SC1);
        CloudFrame_warpPointcloud->str_rhopts_->img_x                 = cv::Mat::zeros(img_height_, img_width_, CV_32FC1);
        CloudFrame_warpPointcloud->str_rhopts_->img_y                 = cv::Mat::zeros(img_height_, img_width_, CV_32FC1);
        CloudFrame_warpPointcloud->str_rhopts_->img_z                 = cv::Mat::zeros(img_height_, img_width_, CV_32FC1);
    }
    velo_xyz_->resize(0);
    pts_warpewd_->resize(0);
}

void PclWarp::reset()
{
    this->velo_xyz_->resize(0);
    this->pts_warpewd_->resize(0);
}