cmake_minimum_required(VERSION 2.4.6)
include($ENV{ROS_ROOT}/core/rosbuild/rosbuild.cmake)

# Set the build type.  Options are:
#  Coverage       : w/ debug symbols, w/o optimization, w/ code-coverage
#  Debug          : w/ debug symbols, w/o optimization
#  Release        : w/o debug symbols, w/ optimization
#  RelWithDebInfo : w/ debug symbols, w/ optimization
#  MinSizeRel     : w/o debug symbols, w/ optimization, stripped binaries
#set(ROS_BUILD_TYPE RelWithDebInfo)

rosbuild_init()

#set the default path for built executables to the "bin" directory
set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)
#set the default path for built libraries to the "lib" directory
set(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib)

#uncomment if you have defined messages
#rosbuild_genmsg()
#uncomment if you have defined services
#rosbuild_gensrv()

#common commands for building c++ executables and libraries
rosbuild_add_library(${PROJECT_NAME} src/buffer.cpp src/transform_listener.cpp src/buffer_client.cpp src/buffer_server.cpp src/transform_broadcaster.cpp src/static_transform_broadcaster.cpp)
#target_link_libraries(${PROJECT_NAME} another_library)
rosbuild_add_boost_directories()
rosbuild_add_executable(buffer_server src/buffer_server_main.cpp)
target_link_libraries(buffer_server ${PROJECT_NAME})
rosbuild_link_boost(buffer_server thread)

rosbuild_add_executable(test_listener test/listener_unittest.cpp)
target_link_libraries(test_listener ${PROJECT_NAME})
rosbuild_add_gtest_build_flags(test_listener)
rosbuild_add_rostest(test/transform_listener_unittest.launch)


rosbuild_add_executable(static_transform_publisher src/static_transform_broadcaster_program.cpp)
target_link_libraries(static_transform_publisher ${PROJECT_NAME})
