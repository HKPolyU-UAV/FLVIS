cmake_minimum_required(VERSION 2.8.3)
project(vo_nodelet)

set(CMAKE_CXX_FLAGS "-std=c++11 ${CMAKE_CXX_FLAGS} -O3 -Wall -pthread") # -Wextra -Werror
set(CMAKE_BUILD_TYPE "RELEASE")

find_package (OpenCV 3 REQUIRED)
find_package (Eigen3 REQUIRED)

find_package(catkin REQUIRED COMPONENTS
    nodelet
    roscpp
    rostime
    sensor_msgs
    std_msgs
    geometry_msgs
    pcl_conversions
    pcl_ros
    cv_bridge
    message_generation
    )


## Declare a catkin package
catkin_package(
    CATKIN_DEPENDS message_runtime
    )

include_directories(SYSTEM
    ${catkin_INCLUDE_DIRS}
    ${PCL_INCLUDE_DIRS}
    "${CMAKE_CURRENT_SOURCE_DIR}/src/"
    )

## Declare a C++ library
add_library(vo_nodelet
  src/vo_tracking.cpp
  src/vo_localmap.cpp
)

target_link_libraries(vo_nodelet ${catkin_LIBRARIES} ${OpenCV_LIBRARIES})

