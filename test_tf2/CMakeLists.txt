cmake_minimum_required(VERSION 2.8.3)
project(test_tf2)

find_package(catkin REQUIRED COMPONENTS rostest tf tf2 tf2_bullet tf2_ros tf2_geometry_msgs tf2_kdl tf2_msgs roscpp)
find_package(Boost REQUIRED COMPONENTS thread)

catkin_package()

catkin_add_gtest(buffer_core_test test/buffer_core_test.cpp)
catkin_add_gtest(test_message_filter test/test_message_filter.cpp)
catkin_add_gtest(test_convert test/test_convert.cppp)

add_executable(test_buffer_server test/test_buffer_server.cpp)
target_link_libraries(test_buffer_server ${Boost_LIBRARIES})

add_executable(test_buffer_client test/test_buffer_client.cpp)
#!!! rosbuild_add_gtest_build_flags(test_buffer_client)

catkin_add_gtest(test/buffer_client_tester.launch)

add_executable(test_static_publisher test/test_static_publisher.cpp)
#!!! rosbuild_add_gtest_build_flags(test_static_publisher)

catkin_add_gtest(test/static_publisher.launch)



