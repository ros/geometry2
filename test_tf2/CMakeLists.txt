cmake_minimum_required(VERSION 2.8.3)
project(test_tf2)

find_package(catkin REQUIRED COMPONENTS rostest tf tf2 tf2_bullet tf2_ros tf2_geometry_msgs tf2_kdl tf2_msgs roscpp)
find_package(Boost REQUIRED COMPONENTS thread)

include_directories(${Boost_INCLUDE_DIRS} ${catkin_INCLUDE_DIRS})

catkin_package()

catkin_add_gtest(buffer_core_test test/buffer_core_test.cpp)

catkin_add_gtest(test_message_filter test/test_message_filter.cpp)
target_link_libraries(test_message_filter ${Boost_LIBRARIES} ${catkin_LIBRARIES})

catkin_add_gtest(test_convert test/test_convert.cpp)
target_link_libraries(test_convert ${Boost_LIBRARIES} ${catkin_LIBRARIES})

catkin_add_gtest(test_buffer_server test/test_buffer_server.cpp)
target_link_libraries(test_buffer_server ${Boost_LIBRARIES} ${catkin_LIBRARIES})

catkin_add_gtest(test_buffer_client test/test_buffer_client.cpp)
target_link_libraries(test_buffer_client ${Boost_LIBRARIES} ${catkin_LIBRARIES})


add_rostest(test/buffer_client_tester.launch)

catkin_add_gtest(test_static_publisher test/test_static_publisher.cpp)
target_link_libraries(test_static_publisher ${Boost_LIBRARIES} ${catkin_LIBRARIES})

add_rostest(test/static_publisher.launch)


