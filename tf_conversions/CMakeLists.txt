cmake_minimum_required(VERSION 2.4.6)
include($ENV{ROS_ROOT}/core/rosbuild/rosbuild.cmake)
set(ROS_BUILD_TYPE Release)
rosbuild_init()

rosbuild_add_boost_directories()

rosbuild_genmsg()
rosbuild_gensrv()

rosbuild_add_library(tf_conversions
    src/tf_kdl.cpp
    src/tf_eigen.cpp
    )

rosbuild_add_gtest(test_eigen_tf test/test_eigen_tf.cpp)
rosbuild_add_gtest(test_kdl_tf test/test_kdl_tf.cpp)
target_link_libraries(test_eigen_tf tf_conversions gtest)
target_link_libraries(test_kdl_tf tf_conversions gtest)

rosbuild_add_pyunit(test/posemath.py)
