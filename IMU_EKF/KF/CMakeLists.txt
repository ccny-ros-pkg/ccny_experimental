cmake_minimum_required(VERSION 2.4.6)
include($ENV{ROS_ROOT}/core/rosbuild/rosbuild.cmake)

# Set the build type. Options are:
# Coverage : w/ debug symbols, w/o optimization, w/ code-coverage
# Debug : w/ debug symbols, w/o optimization
# Release : w/o debug symbols, w/ optimization
# RelWithDebInfo : w/ debug symbols, w/ optimization
# MinSizeRel : w/o debug symbols, w/ optimization, stripped binaries
#set(ROS_BUILD_TYPE RelWithDebInfo)


#include_directories({${KF_SOURCE_DIR}/EXTERNAL/eigen3)
find_package(Eigen REQUIRED)
include_directories(${Eigen_INCLUDE_DIRS})

rosbuild_init()

#set the default path for built executables to the "bin" directory
set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)
#set the default path for built libraries to the "lib" directory
set(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib)

#SET( EKF_EIGEN3_INCLUDE ${g2o_SOURCE_DIR}/EXTERNAL/eigen3 CACHE PATH "Directory of Eigen3")
#INCLUDE_DIRECTORIES(${g2o_SOURCE_DIR} ${G2O_EIGEN3_INCLUDE})

#uncomment if you have defined messages
rosbuild_genmsg()
#uncomment if you have defined services
#rosbuild_gensrv()

#common commands for building c++ executables and libraries
#rosbuild_add_library(${PROJECT_NAME} src/example.cpp)
#target_link_libraries(${PROJECT_NAME} another_library)
#rosbuild_add_boost_directories()
#rosbuild_link_boost(${PROJECT_NAME} thread)
#rosbuild_add_executable(example examples/example.cpp)
#target_link_libraries(example ${PROJECT_NAME})

#common commands for building c++ executables and libraries
rosbuild_add_library(KF src/kf.cpp)

# create niac_nodelet library
rosbuild_add_library (kf_nodelet src/kf_nodelet.cpp)
target_link_libraries (kf_nodelet KF)

# create niac_node library
rosbuild_add_executable(kf_node src/kf_node.cpp)
target_link_libraries (kf_node KF)
