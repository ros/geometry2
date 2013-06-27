cmake_minimum_required(VERSION 2.8.3)
project(test_tf2)


find_package(catkin REQUIRED COMPONENTS rosconsole roscpp rostest tf tf2 tf2_bullet tf2_ros tf2_geometry_msgs tf2_kdl tf2_msgs)
find_package(Boost REQUIRED COMPONENTS thread)

include_directories(${Boost_INCLUDE_DIRS} ${catkin_INCLUDE_DIRS})

catkin_package()

catkin_add_gtest(buffer_core_test test/buffer_core_test.cpp)
target_link_libraries(buffer_core_test ${Boost_LIBRARIES} ${catkin_LIBRARIES})

catkin_add_gtest(test_message_filter test/test_message_filter.cpp)
target_link_libraries(test_message_filter ${Boost_LIBRARIES} ${catkin_LIBRARIES})

catkin_add_gtest(test_convert test/test_convert.cpp)
target_link_libraries(test_convert ${Boost_LIBRARIES} ${catkin_LIBRARIES})

add_executable(test_buffer_server EXCLUDE_FROM_ALL test/test_buffer_server.cpp)
target_link_libraries(test_buffer_server ${Boost_LIBRARIES} ${catkin_LIBRARIES} ${GTEST_LIBRARIES})

add_executable(test_buffer_client EXCLUDE_FROM_ALL test/test_buffer_client.cpp)
target_link_libraries(test_buffer_client ${Boost_LIBRARIES} ${catkin_LIBRARIES} ${GTEST_LIBRARIES})


add_rostest(test/buffer_client_tester.launch)

add_executable(test_static_publisher EXCLUDE_FROM_ALL test/test_static_publisher.cpp)
target_link_libraries(test_static_publisher ${Boost_LIBRARIES} ${catkin_LIBRARIES} ${GTEST_LIBRARIES})

add_rostest(test/static_publisher.launch)


if(TARGET tests)
  add_dependencies(tests test_buffer_server test_buffer_client test_static_publisher)
endif()


# used as a test fixture
if(TARGET tf2_ros_static_transform_publisher)
  add_dependencies(tests tf2_ros_static_transform_publisher)
endif()