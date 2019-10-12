cmake_minimum_required(VERSION 2.8.3)
project(vo_nodelet)

add_definitions(-std=c++11)
set(CMAKE_CXX_FLAGS "-std=c++11)
#set(CMAKE_CXX_FLAGS "-std=c++11 ${CMAKE_CXX_FLAGS} -O3 -Wall -pthread") # -Wextra -Werror
set(CMAKE_BUILD_TYPE "RELEASE")


list( APPEND CMAKE_MODULE_PATH ${CMAKE_CURRENT_SOURCE_DIR}/3rdPartLib/g2o/cmake_modules)
set(G2O_ROOT /usr/local/include/g2o)
find_package(G2O REQUIRED) 

find_package (OpenCV 3 REQUIRED)
find_package (Eigen3 REQUIRED)
find_package (CSparse REQUIRED )
find_package (Sophus REQUIRED )
find_package (yaml-cpp REQUIRED )
find_package (DBoW3 REQUIRED)


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
    image_transport
    tf
    )

add_message_files(
  FILES
  KeyFrame.msg
)

generate_messages(
  DEPENDENCIES
  std_msgs
  sensor_msgs
  geometry_msgs
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
    ${Sophus_INCLUDE_DIRS}
    ${YAML_CPP_INCLUDE_DIR}
    ${DBoW3_INCLUDE_DIR}
    "${CMAKE_CURRENT_SOURCE_DIR}/src/"
    "${CMAKE_CURRENT_SOURCE_DIR}/src/include/"
    )

## Declare a C++ library
add_library(vo_nodelet
    src/vo_tracking.cpp
    src/vo_localmap.cpp
    src/vo_loopclosing.cpp
    src/feature_dem.cpp
    src/depth_camera.cpp
    src/unique_id.cpp
    src/landmark.cpp
    src/landmark_in_frame.cpp
    src/camera_frame.cpp
    src/rviz_frame.cpp
    src/rviz_path.cpp
    src/rviz_tf.cpp
    src/triangulation.cpp
    src/keyframe_msg.cpp
    )

target_link_libraries(vo_nodelet
    ${catkin_LIBRARIES}
    ${OpenCV_LIBRARIES}
    ${CSPARSE_LIBRARY}
    ${Sophus_LIBRARIES}
    ${YAML_CPP_LIBRARIES}
    ${DBoW3_LIBRARIES}
    g2o_core g2o_stuff g2o_types_sba g2o_csparse_extension
    )

#add_executable(testSophus
#    src/test/test_SE3.cpp)

#target_link_libraries(testSophus
#    ${catkin_LIBRARIES}
#    ${OpenCV_LIBRARIES}
#    ${CSPARSE_LIBRARY}
#    ${Sophus_LIBRARIES}
#    g2o_core g2o_stuff g2o_types_sba g2o_csparse_extension
#    )


#add_executable(testVis
#    src/test/rviz_Vis.cpp
#    src/rvizFrame.cpp)

#target_link_libraries(testVis
#    ${catkin_LIBRARIES}
#    ${OpenCV_LIBRARIES}
#    ${CSPARSE_LIBRARY}
#    ${Sophus_LIBRARIES}
#    )

#add_executable(test_YAML2Eigen
#    src/test/test_yaml2eigen.cpp
#)

#target_link_libraries(test_YAML2Eigen
#    ${catkin_LIBRARIES}
#    ${YAML_CPP_LIBRARIES}
#    )

#add_executable(undistort
#    src/test/undistort.cpp)

#target_link_libraries(undistort
#    ${catkin_LIBRARIES}
#    ${OpenCV_LIBRARIES}
#    )

#add_executable(test_triangulation
#    src/test/test_triangulation.cpp
#    src/triangulation.cpp)

#target_link_libraries(test_triangulation
#    ${catkin_LIBRARIES}
#    ${OpenCV_LIBRARIES}
#    ${Sophus_LIBRARIES}
#    )







