cmake_minimum_required(VERSION 2.8.3)
project(test_tf2)

find_package(catkin REQUIRED COMPONENTS tf2_ros bullet geometry_msgs)

catkin_package()

add_executable(test_bullet test/test_tf2_bullet.cpp)
catkin_add_gtest_build_flags(test_bullet)

catkin_add_gtest(${CMAKE_CURRENT_SOURCE_DIR}/test/test.launch)


