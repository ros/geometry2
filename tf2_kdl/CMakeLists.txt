
cmake_minimum_required(VERSION 2.8.3)
project(tf2_kdl)

find_package(catkin REQUIRED COMPONENTS geometry_msgs tf2_ros kdl)
catkin_package()

add_executable(test_kdl test/test_tf2_kdl.cpp)
catkin_add_gtest_build_flags(test_kdl)

catkin_add_rostest(${CMAKE_CURRENT_SOURCE_DIR}/test/test.launch)

