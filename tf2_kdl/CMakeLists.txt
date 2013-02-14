cmake_minimum_required(VERSION 2.8.3)
project(tf2_kdl)

#find_package(GTest)
find_package(catkin REQUIRED COMPONENTS rostest orocos_kdl tf2 tf2_ros tf2_msgs)

catkin_package(
  INCLUDE_DIRS include
)

#uncomment if you have defined messages
#rosbuild_genmsg()
#uncomment if you have defined services
#rosbuild_gensrv()

#common commands for building c++ executables and libraries
#rosbuild_add_library(${PROJECT_NAME} src/example.cpp)
#target_link_libraries(${PROJECT_NAME} another_library)
#rosbuild_add_boost_directories()
#rosbuild_link_boost(${PROJECT_NAME} thread)

include_directories(include ${catkin_INCLUDE_DIRS} ${GTEST_INCLUDE_DIRS})

add_executable(${PROJECT_NAME}_test_kdl test/test_tf2_kdl.cpp)
target_link_libraries(${PROJECT_NAME}_test_kdl ${catkin_LIBRARIES} ${GTEST_LIBRARIES})

add_rostest(${CMAKE_CURRENT_SOURCE_DIR}/test/test.launch)

#rosbuild_add_executable(test_kdl test/test_tf2_kdl.cpp)
#rosbuild_add_gtest_build_flags(test_kdl)
#rosbuild_add_rostest(${CMAKE_CURRENT_SOURCE_DIR}/test/test.launch)

