# OMMOCAR: Online Mapless Moving Object detection for 3D LiDAR using oCclusion Accumulation in Range image
<p align = "center">
<img src= "https://github.com/JunhaAgu/Mapless_Moving_matlab/blob/main/imgs/thumbnail_white.png" alt="aligned four lidars via the AutoL2LCalib" width="474" height="465">
</p> 

<p align = "center">
<img src= "https://github.com/JunhaAgu/Mapless_Moving/blob/main/video/loam/loam_kitti_05.gif" alt="aligned four lidars via the AutoL2LCalib" width="175" height="197.5">
<img src= "https://github.com/JunhaAgu/Mapless_Moving/blob/main/video/loam/loam_carla_town01_002.gif" alt="aligned four lidars via the AutoL2LCalib" width="175" height="197.5">
<img src= "https://github.com/JunhaAgu/Mapless_Moving/blob/main/video/cticp/cticp_kitti_01.gif" alt="aligned four lidars via the AutoL2LCalib" width="175" height="197.5">
<img src= "https://github.com/JunhaAgu/Mapless_Moving/blob/main/video/cticp/cticp_carla_town03_002.gif" alt="aligned four lidars via the AutoL2LCalib" width="175" height="197.5">
</p>
<p align = "center">
Left two: <b>KITTI</b> <i>05</i> (left) and <b>CARLA</b> <i>town01_002</i> (right) with <b><a href="https://github.com/HKUST-Aerial-Robotics/A-LOAM">A-LOAM</a></b>
<p align = "center">
Right two: <b>KITTI</b> <i>01</i> (left) and <b>CARLA</b> <i>town03_002</i> (right) with <b><a href="https://github.com/jedeschaud/ct_icp">CT-ICP</a></b>
  
  <I>The subscript 002 of the town01 and town03 means 3D points have zero mean Gaussian noise whose std. is 0.02 m.</i>
</p>

## 1. Descriptions
**Note:** This software is based on our paper (IEEE Transactions on Instrumentation and Measurement):

The **OMMOCAR** is a program to detect moving objects for 3D LiDAR using occlusion accumulation in range image.

The source code is written in two languages: MATLAB and C++.
(This source code is C++ ROS ver.)

*The Matlab version of the code is uploaded in [OMMOCAR Matlab ver](https://github.com/JunhaAgu/Mapless_Moving_matlab).*

Three main features of the **OMMOCAR** are like;
- The proposed method **does not require prior information** about objects or maps, and outputs are at the **point level**.
- By **accumulating occlusion in the range image domain** and using bitwise operations, the computational speed of the proposed method is about 20 Hz or faster, suitable to run in real-time.
- Because the proposed method is **not coupled with a pose estimation module**, it can be used as the front-end of other 3D LiDAR odometry algorithms.

We demonstrate that the proposed method improves the existing 3D LiDAR odometry algorithms' performance with **KITTI odometry** and **synthetic datasets** (CARLA).

- Maintainers: Junha Kim (wnsgk02@snu.ac.kr) and Changhyeon Kim (rlackd93@snu.ac.kr)

### Datasets used in the paper
The datasets used in our submission are **KITTI odometry (with SemanticKITTI)** and **synthetic datasets** obtained from **CARLA**

- KITTI odometry: [KITTI_odometry](https://www.cvlibs.net/datasets/kitti/eval_odometry.php)
- SemanticKITTI: [SemanticKITTI](http://www.semantic-kitti.org/dataset.html#download)
- CARLA: [CARLA](https://carla.org/)

We evaluate with *00*, *01*, *02*, *05* and *07* of **KITTI odometry**, and *town01* and *town03* of **CARLA**. 

- **KITTI** *00*: 4390 - 4530
- **KITTI** *01*: 150  - 250
- **KITTI** *02*: 860  - 950
- **KITTI** *05*: 2350 - 2670
- **KITTI** *07*: 630  - 820
- **CARLA** *town01* &nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp; : 10 - 370
- **CARLA** *town01_001* : 10 - 370
- **CARLA** *town01_002* : 10 - 370
- **CARLA** *town03* &nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp; : 10 - 400
- **CARLA** *town03_001* : 10 - 400
- **CARLA** *town03_002* : 10 - 400
  
*The subscripts 001 and 002 of the town01 and town03 mean 3D points have zero mean Gaussian noise whose std. is 0.01, 0.02 m.*

Download datasets - [Download link](https://larr.snu.ac.kr/drive/d/s/uulKtWN4b41HXBNk92QigruwP2eBMqhY/4-Lw2fCmp5F_xCIgcX2TNC_qzBnMwVFd-HbYgiTNDsQo)

## 2) How to run OMMOCAR C++ ROS ver.?
### (a) Dependencies
Recommend: Ubuntu 20.04 Noetic

*We tested only in 20.04*

### (b) Build
Clone the repository and catkin build

```
    cd ~/catkin_ws/src
    git clone https://github.com/JunhaAgu/Mapless_Moving.git
    catkin build mapless_moving
    source ~/catkin_ws/devel/setup.bash
```

### (c) Run
#### (I) with pcd files and gt-pose
In run.launch file, modify **"dataset_name"** and **"data_number"**
, and set 
```
<arg name="rosbag_play"         default="false"/>
<arg name="T01_slam"            default="false"/>
```
and then,
```
  roslaunch mapless_moving run.launch
```

#### (II) with rosbag files and estimated pose from Odometry algorithm
(1) In run.launch file, set 
```
<arg name="rosbag_play"         default="true"/>
<arg name="T01_slam"            default="true"/>
```
(2) launch an odometry algorithm (A-LOAM and CT-ICP in this paper) 

(3) launch OMMOCAR
```
  roslaunch mapless_moving run.launch
```

(4) rosbag play
