cmake_minimum_required(VERSION 2.4.6)
include($ENV{ROS_ROOT}/core/rosbuild/rosbuild.cmake)
rosbuild_init()

# Unit tests
rosbuild_add_gtest(utest test/utest.cpp)
target_link_libraries(utest)
