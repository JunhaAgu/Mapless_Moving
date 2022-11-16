#ifndef _DEFINES_H_
#define _DEFINES_H_

#include <iostream>
#include <ros/ros.h>
#include <vector>
#include <Eigen/Dense>

// ROS cv_bridge
#include <cv_bridge/cv_bridge.h>
#include <opencv2/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>

#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/common/transforms.h>

#include <pcl/register_point_struct.h>
#include <pcl/pcl_macros.h>
#include <pcl/io/pcd_io.h>

typedef std::vector<bool> Mask;
typedef Eigen::Matrix4d   Pose; 
typedef Eigen::Matrix3d   Rot;
typedef Eigen::Vector3f   Euler;

// distance from origin
#define NORM(x, y, z) (sqrt(((x) * (x) + (y) * (y) + (z) * (z))))

#define R2D 180.0f/M_PI
#define D2R M_PI/180.0f

// struct PointXYZT
// {
//   PCL_ADD_POINT4D;                  // preferred way of adding a XYZ+padding
//   double timestamp;
//   PCL_MAKE_ALIGNED_OPERATOR_NEW     // make sure our new allocators are aligned
// } EIGEN_ALIGN16;                    // enforce SSE padding for correct memory alignment

// POINT_CLOUD_REGISTER_POINT_STRUCT (PointXYZT,           // here we assume a XYZ + "test" (as fields)
//                                    (float, x, x)
//                                    (float, y, y)
//                                    (float, z, z)
//                                    (double, timestamp, timestamp)
// )

namespace slam {

    // A XYZT point compatible with pcl
    struct PointXYZT {

        inline PointXYZT(const PointXYZT &pt) : x(pt.x), y(pt.y), z(pt.z), timestamp(pt.timestamp) {
            data[3] = 1.0f;
        }

        inline PointXYZT &operator=(const slam::PointXYZT &pt) {
            x = pt.x;
            y = pt.y;
            z = pt.z;
            timestamp = pt.timestamp;
            return *this;
        }

        inline PointXYZT() : x(0.f), y(0.f), z(0.f), timestamp(0.) {
            data[3] = 1.0f;
        }

        PCL_ADD_POINT4D

        double timestamp;

        EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    } EIGEN_ALIGN16;

} // namespace slam

POINT_CLOUD_REGISTER_POINT_STRUCT (slam::PointXYZT,
                                    (float, x, x)
                                    (float, y, y)
                                    (float, z, z)
                                    (double, timestamp, timestamp))

typedef pcl::PointCloud<slam::PointXYZT> PointCloudwithTime;

struct StrRhoPts
{
    // pcl::PointCloud<pcl::PointXYZI>::Ptr pts;

    std::vector<float> rho;
    std::vector<float> phi;
    std::vector<float> theta;

    cv::Mat img_rho;
    cv::Mat img_index;

    cv::Mat img_x;
    cv::Mat img_y;
    cv::Mat img_z;

    // std::vector<int> pts_per_pixel_n;
    std::vector<std::vector<int>> pts_per_pixel_index;
    std::vector<std::vector<float>> pts_per_pixel_rho;
    std::vector<std::vector<int>> pts_per_pixel_index_valid;

    //interpRangeImage
    cv::Mat img_restore_mask;
    cv::Mat img_restore_warp_mask;

    void reset(){
        // pts        = nullptr;

        rho.resize(0);
        phi.resize(0);
        theta.resize(0);

        // img_rho = 
        // for(auto it : pts_per_pixel_n)           it = 0;
        for(auto it : pts_per_pixel_index)       it.resize(0);
        for(auto it : pts_per_pixel_rho)         it.resize(0);
        for(auto it : pts_per_pixel_index_valid) it.resize(0);
    };

    void state(){
        // std::cout << "(ptr) pts size: " << pts->size() << std::endl;
        std::cout << "(vector) rho size: " << rho.size() << std::endl;
        std::cout << "(vector) phi size: " << phi.size() << std::endl;
        std::cout << "(vector) theta size: " << theta.size() << std::endl;
        // std::cout << "(vector) pts_per_pixel_n size: " << pts_per_pixel_n.size() << std::endl;
        std::cout << "(vector(vector)) pts_per_pixel_index size: " << pts_per_pixel_index.size() << std::endl;
        std::cout << "(vector(vector)) pts_per_pixel_rho size: " << pts_per_pixel_rho.size() << std::endl;
        std::cout << "(vector) pts_per_pixel_index_valid size: " << pts_per_pixel_index_valid.size() << std::endl;
        std::cout << " ================================== "<< std::endl;
    }
};

#endif