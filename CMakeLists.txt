cmake_minimum_required(VERSION 3.0.2)
project(mapless_dynamic)
set(CMAKE_CONFIGURATION_TYPES "Release" CACHE STRING "" FORCE)
add_compile_options(-std=c++17)

find_package(catkin REQUIRED cmake_modules)
find_package(Eigen3 REQUIRED)
find_package(OpenCV REQUIRED)

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

catkin_package(
  DEPENDS EIGEN3 #Eigen -> EIGEN3 : resolved
  # CATKIN_DEPENDS message_runtime roscpp message_filters
)

set(SRC ${SRC} 
  src/main.cpp
#   src/data.cpp
#   src/lidar_image.cpp
#   src/roi.cpp
#   src/function_transform.cpp
#   src/relative_pose.cpp
)

set(includeDirs ${includeDirs} ${CMAKE_CURRENT_BINARY_DIR})
set(includeDirs ${includeDirs} include)

include_directories( ${includeDirs} )
include_directories( ${catkin_INCLUDE_DIRS} )
include_directories( ${Eigen_INCLUDE_DIRS} )


add_executable (main_mapless_dynamic src/main.cpp ${SRC})
target_link_libraries (main_mapless_dynamic ${catkin_LIBRARIES} ${OpenCV_LIBRARIES} ${Eigen_LIBRARIES})