cmake_minimum_required(VERSION 2.4.6)
include($ENV{ROS_ROOT}/core/rosbuild/rosbuild.cmake)
set(ROS_BUILD_TYPE Debug)
rosbuild_init()

rosbuild_add_boost_directories()

rosbuild_genmsg()
rosbuild_gensrv()

rosbuild_add_library(tf src/tf.cpp src/transform_listener.cpp src/cache.cpp src/message_notifier.cpp src/transform_broadcaster.cpp)
rosbuild_link_boost(tf thread signals)

rosbuild_add_executable(bin/tf_echo src/tf_echo.cpp)
target_link_libraries(bin/tf_echo tf)

rosbuild_add_executable(bin/change_notifier src/change_notifier.cpp)
target_link_libraries(bin/change_notifier tf)

rosbuild_add_executable(bin/tf_monitor src/tf_monitor.cpp)
target_link_libraries(bin/tf_monitor tf)

rosbuild_add_gtest(test/tf_unittest test/tf_unittest.cpp)
target_link_libraries(test/tf_unittest tf)

rosbuild_add_executable(test/transform_listener_unittest test/transform_listener_unittest.cpp)
target_link_libraries(test/transform_listener_unittest tf)
rosbuild_add_gtest_build_flags(test/transform_listener_unittest)
rosbuild_add_rostest(test/transform_listener_unittest.launch)

if(${CMAKE_SYSTEM_NAME} STREQUAL Linux)
  rosbuild_add_gtest(test/wait_for_transform_death test/wait_for_transform_death.cpp)
  target_link_libraries(test/wait_for_transform_death tf)
endif(${CMAKE_SYSTEM_NAME} STREQUAL Linux)

rosbuild_add_gtest_future(test/tf_unittest_future test/tf_unittest_future.cpp)
target_link_libraries(test/tf_unittest_future tf)
rosbuild_add_gtest(test/cache_unittest test/cache_unittest.cpp)
target_link_libraries(test/cache_unittest tf)


### Benchmarking
rosbuild_add_gtest_future(test/tf_benchmark test/tf_benchmark.cpp)
target_link_libraries(test/tf_benchmark tf)

rosbuild_add_executable(test/btTest test/quaternion.cpp)
rosbuild_add_executable(test/testListener test/testListener.cpp)
target_link_libraries(test/testListener tf)
                                 
rosbuild_add_executable(test/testBroadcaster test/testBroadcaster.cpp)
target_link_libraries(test/testBroadcaster tf)
rosbuild_add_executable(bin/transform_sender src/transform_sender.cpp)
target_link_libraries(bin/transform_sender tf)

rosbuild_add_executable(test/test_notifier EXCLUDE_FROM_ALL test/test_notifier.cpp)
rosbuild_declare_test(test/test_notifier)
rosbuild_add_gtest_build_flags(test/test_notifier)
target_link_libraries(test/test_notifier tf ${Boost_LIBRARIES})
rosbuild_add_rostest_future(test/test_notifier.xml)

rosbuild_add_executable(test/test_message_filter EXCLUDE_FROM_ALL test/test_message_filter.cpp)
rosbuild_declare_test(test/test_message_filter)
rosbuild_add_gtest_build_flags(test/test_message_filter)
target_link_libraries(test/test_message_filter tf ${Boost_LIBRARIES})
rosbuild_add_rostest(test/test_message_filter.xml)

find_package(PythonLibs REQUIRED)
include_directories(${PYTHON_INCLUDE_PATH})

# # If on Darwin, create a symlink _foo.so -> _foo.dylib, because the
# # MacPorts version of Python won't find _foo.dylib for 'import _foo'
# include(CMakeDetermineSystem)
# if(CMAKE_SYSTEM_NAME MATCHES "Darwin")
#   add_custom_command(OUTPUT ${LIBRARY_OUTPUT_PATH}/_${PROJECT_NAME}_swig.so
#                      COMMAND cmake -E create_symlink  ${LIBRARY_OUTPUT_PATH}/_${PROJECT_NAME}_swig.dylib ${LIBRARY_OUTPUT_PATH}/_${PROJECT_NAME}_swig.so
#                      DEPENDS python_${PROJECT_NAME})
#   add_custom_target(symlink_darwin_lib ALL
#                     DEPENDS ${LIBRARY_OUTPUT_PATH}/_${PROJECT_NAME}_swig.so)
# endif(CMAKE_SYSTEM_NAME MATCHES "Darwin")

# Dynamic linking with tf worked OK, except for exception propagation, which failed in the unit test.
# so build with the objects directly instead.

link_libraries(${PYTHON_LIBRARIES})
rosbuild_add_library(pytf_py src/pytf.cpp src/tf.cpp src/transform_listener.cpp src/cache.cpp)
find_package(PythonLibs REQUIRED)
set_target_properties(pytf_py PROPERTIES OUTPUT_NAME tf PREFIX "_" SUFFIX ".so")
# target_link_libraries(pytf_py tf)
rosbuild_add_compile_flags(pytf_py -g -Wno-missing-field-initializers -msse3)
rosbuild_add_pyunit(test/testPython.py)
