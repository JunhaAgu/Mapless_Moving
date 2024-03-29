cmake_minimum_required(VERSION 3.5.1)
project(mapless_moving)
set(CMAKE_BUILD_TYPE Release) # Debug, Release, RelWithDebInfo
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++17 -pthread -pg -O4 -msse3 -flto -pg -march=native -mfpmath=sse") #O0,O2: optimize 
set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} ${CMAKE_CURRENT_SOURCE_DIR}) 
find_package(catkin REQUIRED cmake_modules)
find_package(Eigen3 REQUIRED)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  cv_bridge

  message_generation

  std_msgs
  sensor_msgs
  pcl_conversions
  pcl_ros
  message_filters
  image_transport
)

find_package(OpenCV REQUIRED)

catkin_package(
  DEPENDS EIGEN3 #Eigen -> EIGEN3 : resolved
  # CATKIN_DEPENDS message_runtime roscpp message_filters
)

set(SRC ${SRC} 
  src/ros_wrapper.cpp
  src/mapless_moving.cpp
  src/timer.cpp
  src/user_param.cpp
  src/cloud_frame.cpp
  src/segment_ground.cpp
  src/dR_calc.cpp
  src/pcl_warp.cpp
  src/object_ext.cpp
  src/image_fill.cpp
)

set(includeDirs ${includeDirs} ${CMAKE_CURRENT_BINARY_DIR} include)

include_directories( ${includeDirs} )
include_directories( ${catkin_INCLUDE_DIRS} )
include_directories( ${Eigen_INCLUDE_DIRS} )

add_executable(mapless_moving_node src/main.cpp ${SRC})
target_link_libraries (mapless_moving_node ${catkin_LIBRARIES} ${OpenCV_LIBRARIES} ${Eigen_LIBRARIES})

# install
install(TARGETS mapless_moving_node RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
install(DIRECTORY launch/
    DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
)