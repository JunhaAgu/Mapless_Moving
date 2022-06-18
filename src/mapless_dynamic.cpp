#include "mapless_dynamic.h"

MaplessDynamic::MaplessDynamic() {
    // constructor
    ROS_INFO_STREAM("MaplessDynamic - constructed.");
    // IMPLEMENT YOUR CODE FROM THIS LINE.

    // do something...

    // END YOUR CODE
};


MaplessDynamic::~MaplessDynamic() {
    // destructor
    ROS_INFO_STREAM("MaplessDynamic - deleted.");
    // IMPLEMENT YOUR CODE FROM THIS LINE.

    // do something...

    // END YOUR CODE  
};


void MaplessDynamic::TEST(){
    // The test function.
    // IMPLEMENT YOUR CODE FROM THIS LINE.

    // p0 = get!
    // p1 = get!
    // T01 = get!

    // this->algorithm(p0,p1,T01);
    
    // END YOUR CODE  
};


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

};