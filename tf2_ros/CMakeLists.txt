cmake_minimum_required(VERSION 2.8.3)
project(tf2_ros)

find_package(catkin REQUIRED COMPONENTS
  actionlib
  actionlib_msgs
  geometry_msgs
  message_filters
  roscpp
  rosgraph
  rospy
  tf2
  tf2_msgs
)
find_package(Boost REQUIRED COMPONENTS thread)

catkin_python_setup()

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES ${PROJECT_NAME}
  CATKIN_DEPENDS
  actionlib
  actionlib_msgs
  geometry_msgs
  message_filters
  roscpp
  rosgraph
  tf2
  tf2_msgs
)

include_directories(include ${catkin_INCLUDE_DIRS})

# tf2_ros library
add_library(${PROJECT_NAME}
  src/buffer.cpp
  src/transform_listener.cpp
  src/buffer_client.cpp
  src/buffer_server.cpp
  src/transform_broadcaster.cpp
  src/static_transform_broadcaster.cpp
)
add_dependencies(${PROJECT_NAME} tf2_msgs_gencpp)
target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES})

# buffer_server executable
add_executable(${PROJECT_NAME}_buffer_server src/buffer_server_main.cpp)
add_dependencies(${PROJECT_NAME}_buffer_server tf2_msgs_gencpp)
target_link_libraries(${PROJECT_NAME}_buffer_server
  ${PROJECT_NAME}
  ${catkin_LIBRARIES}
  ${Boost_LIBRARIES}
)
set_target_properties(${PROJECT_NAME}_buffer_server
  PROPERTIES OUTPUT_NAME buffer_server
)

# static_transform_publisher
add_executable(${PROJECT_NAME}_static_transform_publisher
  src/static_transform_broadcaster_program.cpp
)
add_dependencies(${PROJECT_NAME}_static_transform_publisher tf2_msgs_gencpp)
target_link_libraries(${PROJECT_NAME}_static_transform_publisher
  ${PROJECT_NAME}
  ${catkin_LIBRARIES}
)
set_target_properties(${PROJECT_NAME}_static_transform_publisher
  PROPERTIES OUTPUT_NAME static_transform_publisher
)

# tf2_ros_test_listener
catkin_add_gtest(${PROJECT_NAME}_test_listener test/listener_unittest.cpp)
add_dependencies(${PROJECT_NAME}_test_listener tf2_msgs_gencpp)
target_link_libraries(${PROJECT_NAME}_test_listener
  ${PROJECT_NAME}
  ${catkin_LIBRARIES}
)

# Install rules
install(TARGETS
  ${PROJECT_NAME} ${PROJECT_NAME}_buffer_server
  ${PROJECT_NAME}_static_transform_publisher
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)
