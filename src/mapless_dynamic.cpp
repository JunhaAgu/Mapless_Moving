#include "mapless_dynamic.h"
#include <string>
#include <fstream> 


MaplessDynamic::MaplessDynamic(bool test_flag)
: test_flag_(test_flag)
 {
    // constructor
    ROS_INFO_STREAM("MaplessDynamic - constructed.");
    // IMPLEMENT YOUR CODE FROM THIS LINE.

    // do something...

    // END YOUR CODE

    this->getUserSettingParameters();
    accumulated_dRdt_ = new cv::Mat(img_height_, img_width_, CV_32FC1);
    accumulated_dRdt_score_ = new cv::Mat(img_height_, img_width_, CV_32FC1);

    if (test_flag_){
        this->loadTestData();
    }
};


MaplessDynamic::~MaplessDynamic() {
    // destructor
    ROS_INFO_STREAM("MaplessDynamic - deleted.");
    // IMPLEMENT YOUR CODE FROM THIS LINE.

    // do something...

    // END YOUR CODE  
};


void MaplessDynamic::TEST(){
    std::cout<< "Test" << std::endl;
    // The test function.
    // IMPLEMENT YOUR CODE FROM THIS LINE.

    // p0 = get!
    // p1 = get!
    // T01 = get!

    // this->algorithm(p0,p1,T01);
    
    // END YOUR CODE

    
};



void MaplessDynamic::loadTestData(){
    std::string data_num = "07";
    std::string dataset_dir = "/home/junhakim/KITTI_odometry/";
    float pose_arr[12];
    Pose T_tmp;
    Pose T_warp;
    T_warp << 0, -1, 0, 0,
        0, 0, -1, 0,
        1, 0, 0, 0,
        0, 0, 0, 1;
    
    int d_start;
    int d_final;
    if (data_num == "00"){
        d_start = 4390 + 00;
        d_final = 4530 + 1; // 1 ~ 141+2 75
    }
    else if (data_num == "01"){
        d_start = 150 + 00;
        d_final = 250 + 1; // 1 ~ 101+2
    }
    else if (data_num == "02"){
        d_start = 860 + 00;
        d_final = 950 + 1; // 1 ~ 91+2
    }
    else if (data_num == "05"){
        d_start = 2350 + 00;
        d_final = 2670 + 1; // 1 ~ 321+2
        //         d_start = 2350+269; d_final = 2670+1; // 1 ~ 321+2
    }
    else if (data_num == "07"){
        d_start = 630 + 00;
        d_final = 820 + 1; // 1 ~ 190+2
        //         d_start = 630+140; d_final = 820+1; // 1 ~ 190+2
    }
    d_start = d_start - 1;
    d_final = d_final - 1;

    // read Association --> n_data_
    std::string as_dir = dataset_dir + "data_odometry_poses/dataset/poses/" + data_num + ".txt";
    std::ifstream associationfile;
    associationfile.open(as_dir.c_str());
    std::string s;
    int cnt_line = 0;

    std::cout << "Test data directory: " << as_dir << std::endl;
    while (getline(associationfile, s, '\n'))
    {
        ++cnt_line;
    }
    associationfile.close();
    this->test_n_data_ = cnt_line;
    // std::cout << cnt_line << std::endl;

    test_data_buf_.reserve(test_n_data_);
    T_gt_.reserve(test_n_data_);
    all_pose.reserve(test_n_data_);
    for(int i=0; i<test_n_data_; ++i){
        all_pose[i].reserve(12);
        test_data_buf_.push_back(new S_TEST_DATA);

        test_data_buf_[i]->pcl_.reset(new pcl::PointCloud<pcl::PointXYZI>);

        T_gt_.push_back(new Pose);
    }
    

    // read Association --> save pcds
    associationfile.open(as_dir.c_str());
    cnt_line = 0;
    while (!associationfile.eof())
    {
        getline(associationfile, s);
        if (!s.empty())
        {
            std::stringstream ss;
            ss << s;
            for (int i = 0; i < 12; ++i)
            {
                ss >> pose_arr[i];
                all_pose[cnt_line].push_back(pose_arr[i]);
                // std::cout << pose_arr[i] << std::endl;
                
            }
            // std::cout << test_data_buf_[cnt_line]->T_gt << std::endl;
            ++cnt_line;
        }
    }
    int n_valid_data = d_final - d_start + 1;
    std::vector<int> valid_data;
    for (int i=0; i<n_valid_data; ++i)
    {
        valid_data.push_back(d_start+i);
    }
    

    for (int ii = 0; ii < n_valid_data; ++ii)
    {
        int i = valid_data[ii];
        T_tmp = Pose::Zero(4, 4);
        T_tmp(3, 3) = 1;
        T_tmp.block<1, 4>(0, 0) << all_pose[i][0], all_pose[i][1], all_pose[i][2], all_pose[i][3];
        T_tmp.block<1, 4>(1, 0) << all_pose[i][4], all_pose[i][5], all_pose[i][6], all_pose[i][7];
        T_tmp.block<1, 4>(2, 0) << all_pose[i][8], all_pose[i][9], all_pose[i][10], all_pose[i][11];

        if (test_data_type_ == "KITTI")
        {
            test_data_buf_[i]->T_gt = T_tmp * T_warp;
        }
        else if (test_data_type_ == "CARLA")
        {
            test_data_buf_[i]->T_gt = T_tmp;
        }
        // std::cout << test_data_buf_[ii]->T_gt << std::endl;
        // std::cout <<ii << std::endl;
    }

    for (int ii = 0; ii < n_valid_data; ++ii)
    {
        int i = valid_data[ii];
        if (test_data_type_ == "KITTI")
        {
            if (ii == 0)
                *(T_gt_[ii]) = (test_data_buf_[i]->T_gt).inverse() * test_data_buf_[i]->T_gt;
            else
                *(T_gt_[ii]) = (test_data_buf_[i - 1]->T_gt).inverse() * test_data_buf_[i]->T_gt;
            // std::cout << *(T_gt_[ii]) << std::endl;
        }
        else if (test_data_type_ == "CARLA")
        {
            if (ii == 0){}
            else
                *(T_gt_[ii-1]) = (test_data_buf_[i - 1]->T_gt).inverse() * test_data_buf_[i]->T_gt;
            // std::cout << *(T_gt_[ii]) << std::endl;
        }
    }

    // LiDAR data read
    std::string bin_path;
    bin_path = "/home/junhakim/KITTI_odometry/data_odometry_velodyne/dataset/sequences/07/velodyne/";

    read_filelists(bin_path, file_lists_, "bin");
    sort_filelists(file_lists_, "bin");

    #pragma omp parallel num_threads(8)
    #pragma omp parallel for
    for (int i = valid_data[0]; i < valid_data[valid_data.size()-1]+1; ++i)
    {   
        std::string bin_file = bin_path + file_lists_[i];
        // std::cout << i << std::endl;
        readKittiPclBinData(bin_file, i);
    }
}

void MaplessDynamic::solve(
    /* inputs */ 
    const sensor_msgs::PointCloud2& p0, const sensor_msgs::PointCloud2& p1, const Pose& T01, 
    /* outputs */ 
    Mask& mask1)
{
    // IMPLEMENT YOUR ALGORITHM FROM THIS LINE.

    // do something...

    // END YOUR ALGORITHM
};

void MaplessDynamic::getUserSettingParameters(){
    // IMPLEMENT YOUR CODE FROM THIS LINE.

    // do something...

    // END YOUR CODE  

    img_height_ = 64/1;
    img_width_ = 4500/5+1;
    object_threshold_ = 30;

    alpha_ = 0.3;
    beta_ = 0.4;

    //initialize
    score_cnt_ = 0;

    //for test
    test_data_type_ = "KITTI";      
};

void MaplessDynamic::calculateGTpose(int cnt_line)
{
    Pose T_tmp = Pose::Zero(4,4);
    std::vector<float> pose_tmp = all_pose[cnt_line];
    // T_tmp.block<1,4>(0,0) << pose_tmp[0], pose_tmp[1], pose_tmp[2], pose_tmp[3];
    // T_tmp.block<1,4>(1,0) << pose_tmp[4], pose_tmp[5], pose_tmp[6], pose_tmp[7];
    // T_tmp.block<1,4>(2,0) << pose_tmp[8], pose_tmp[9], pose_tmp[10], pose_tmp[11];
}

//////////// TO READ KITTI BIN FILES ////////////

void MaplessDynamic::read_filelists(const std::string& dir_path,std::vector<std::string>& out_filelsits,std::string type)
{
    struct dirent *ptr;
    DIR *dir;
    dir = opendir(dir_path.c_str());
    out_filelsits.clear();
    while ((ptr = readdir(dir)) != NULL){
        std::string tmp_file = ptr->d_name;
        if (tmp_file[0] == '.')continue;
        if (type.size() <= 0){
            out_filelsits.push_back(ptr->d_name);
        }else{
            if (tmp_file.size() < type.size())continue;
            std::string tmp_cut_type = tmp_file.substr(tmp_file.size() - type.size(),type.size());
            if (tmp_cut_type == type){
                out_filelsits.push_back(ptr->d_name);
            }
        }
    }
}

bool computePairNum(std::string pair1,std::string pair2)
{
    return pair1 < pair2;
}

void MaplessDynamic::sort_filelists(std::vector<std::string>& filists,std::string type)
{
    if (filists.empty())return;

    std::sort(filists.begin(),filists.end(),computePairNum);
}

void MaplessDynamic::readKittiPclBinData(std::string &in_file, int file_num)
{
    // load point cloud
    std::fstream input(in_file.c_str(), std::ios::in | std::ios::binary);
    if(!input.good()){
        std::cerr << "Could not read file: " << in_file << std::endl;
        exit(EXIT_FAILURE);
    }
    input.seekg(0, std::ios::beg);

    // pcl::PointCloud<pcl::PointXYZI>::Ptr points (new pcl::PointCloud<pcl::PointXYZI>);

    int i;
    for (i=0; input.good() && !input.eof(); ++i) {
        pcl::PointXYZI point;
        input.read((char *) &point.x, 3*sizeof(float));
        input.read((char *) &point.intensity, sizeof(float));
        test_data_buf_[file_num]->pcl_->push_back(point);
        // ->push_back(point);
    }

    std::cout<< "file_num: "<<file_num <<" "<<"num_pts: "<<test_data_buf_[file_num]->pcl_->size() <<std::endl;
    std::cout<< "file_num: "<<file_num <<" "<<"final pts: "<<test_data_buf_[file_num]->pcl_->at((test_data_buf_[file_num]->pcl_)->size()-2) <<std::endl;
    input.close();
}