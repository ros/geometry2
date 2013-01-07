cmake_minimum_required(VERSION 2.8.3)
project(tf2_ros)

find_package(catkin REQUIRED COMPONENTS actionlib
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

catkin_package(
   INCLUDE_DIRS include
   LIBRARIES ${PROJECT_NAME}
   CATKIN_DEPENDS actionlib
                  actionlib_msgs
                  geometry_msgs
                  message_filters
                  roscpp
                  rosgraph
                  rospy
                  tf2
                  tf2_msgs)

include_directories(include ${catkin_INCLUDE_DIRS})

#common commands for building c++ executables and libraries
add_library(${PROJECT_NAME} src/buffer.cpp 
                            src/transform_listener.cpp 
                            src/buffer_client.cpp 
                            src/buffer_server.cpp 
                            src/transform_broadcaster.cpp 
                            src/static_transform_broadcaster.cpp)
add_dependencies(${PROJECT_NAME} tf2_msgs_gencpp)

add_executable(buffer_server src/buffer_server_main.cpp)
target_link_libraries(buffer_server ${PROJECT_NAME} ${catkin_LIBRARIES} ${Boost_LIBRARIES})
add_dependencies(buffer_server tf2_msgs_gencpp)

catkin_add_gtest(test_listener test/listener_unittest.cpp)
target_link_libraries(test_listener ${PROJECT_NAME} ${catkin_LIBRARIES})
add_dependencies(test_listener tf2_msgs_gencpp)
#!! ros_add_gtest_build_flags(test_listener)
#!! ros_add_rostest(test/transform_listener_unittest.launch)
##add_dependencies(tests test_listener)


add_executable(static_transform_publisher src/static_transform_broadcaster_program.cpp)
target_link_libraries(static_transform_publisher ${PROJECT_NAME} ${catkin_LIBRARIES} )
add_dependencies(static_transform_publisher tf2_msgs_gencpp)

install(TARGETS ${PROJECT_NAME} buffer_server static_transform_publisher
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)



