
cmake_minimum_required(VERSION 2.8.3)
project(tf2_geometry_msgs)

find_package(catkin REQUIRED COMPONENTS geometry_msgs tf2_ros kdl)
catkin_package()

add_executable(test_geometry_msgs test/test_tf2_geometry_msgs.cpp)
catkin_add_gtest_build_flags(test_geometry_msgs)

catkin_add_rostest(${CMAKE_CURRENT_SOURCE_DIR}/test/test.launch)


