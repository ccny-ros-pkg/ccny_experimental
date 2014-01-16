cmake_minimum_required(VERSION 2.4.6)
include($ENV{ROS_ROOT}/core/rosbuild/rosbuild.cmake)

# Set the build type. Options are:
# Coverage : w/ debug symbols, w/o optimization, w/ code-coverage
# Debug : w/ debug symbols, w/o optimization
# Release : w/o debug symbols, w/ optimization
# RelWithDebInfo : w/ debug symbols, w/ optimization
# MinSizeRel : w/o debug symbols, w/ optimization, stripped binaries
set(ROS_BUILD_TYPE RelWithDebInfo)

find_package(Eigen REQUIRED)
include_directories(${Eigen_INCLUDE_DIRS})

rosbuild_init()

#set the default path for built executables to the "bin" directory
set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)
#set the default path for built libraries to the "lib" directory
set(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib)

# create complementary_filter library
rosbuild_add_library(complementary_filter
                     src/complementary_filter.cpp 
                     src/complementary_filter_ros.cpp)

# create complementary_filter_node executable
rosbuild_add_executable(complementary_filter_node 
                        src/complementary_filter_node.cpp)
target_link_libraries(complementary_filter_node complementary_filter)
