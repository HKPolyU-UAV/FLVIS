cmake_minimum_required(VERSION 2.8.3)
project(vo_nodelet)

set(CMAKE_CXX_FLAGS "-std=c++11 ${CMAKE_CXX_FLAGS} -O3 -Wall -pthread") # -Wextra -Werror
set(CMAKE_BUILD_TYPE "RELEASE")

find_package (OpenCV 3 REQUIRED)
find_package (Eigen3 REQUIRED)
find_package (G2O REQUIRED )
find_package (CSparse REQUIRED )
find_package (Sophus REQUIRED )


find_package(catkin REQUIRED COMPONENTS
    nodelet
    roscpp
    rostime
    sensor_msgs
    std_msgs
    geometry_msgs
    cv_bridge
    message_generation
    message_filters
    )


## Declare a catkin package
catkin_package(
    CATKIN_DEPENDS message_runtime
    )

include_directories(SYSTEM
    ${catkin_INCLUDE_DIRS}
    ${OpenCV_INCLUDE_DIRS}
    ${G2O_INCLUDE_DIRS}
    ${CSPARSE_INCLUDE_DIR}
    "${CMAKE_CURRENT_SOURCE_DIR}/src/"
    "${CMAKE_CURRENT_SOURCE_DIR}/src/include/"
    "${CMAKE_CURRENT_SOURCE_DIR}/src/include/"
    ${Sophus_INCLUDE_DIRS}
    )

## Declare a C++ library
add_library(vo_nodelet
    src/vo_tracking.cpp
    src/vo_localmap.cpp
    src/featureDEM.cpp
    src/depthCamera.cpp
    src/landMark.cpp
    src/landMarkInFrame.cpp
    src/cameraFrame.cpp
    src/rvizFrame.cpp
    src/rvizPath.cpp
    )

target_link_libraries(vo_nodelet
    ${catkin_LIBRARIES}
    ${OpenCV_LIBRARIES}
    ${CSPARSE_LIBRARY}
    ${Sophus_LIBRARIES}
    g2o_core g2o_stuff g2o_types_sba g2o_csparse_extension
    )

add_executable(testSophus
    src/test.cpp)

target_link_libraries(testSophus
    ${catkin_LIBRARIES}
    ${OpenCV_LIBRARIES}
    ${CSPARSE_LIBRARY}
    ${Sophus_LIBRARIES}
    g2o_core g2o_stuff g2o_types_sba g2o_csparse_extension
    )


add_executable(testVis
    src/rvizFrame.cpp
    src/testVis.cpp)

target_link_libraries(testVis
    ${catkin_LIBRARIES}
    ${OpenCV_LIBRARIES}
    ${CSPARSE_LIBRARY}
    ${Sophus_LIBRARIES}
    )

