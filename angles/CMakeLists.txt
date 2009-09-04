cmake_minimum_required(VERSION 2.4.6)
include($ENV{ROS_ROOT}/core/rosbuild/rosbuild.cmake)
rospack(angles)

# Unit tests
rospack_add_gtest(utest test/utest.cpp)
target_link_libraries(utest)
