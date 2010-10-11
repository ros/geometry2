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
rosbuild_add_boost_directories()

#set the default path for built executables to the "bin" directory
set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)
#set the default path for built libraries to the "lib" directory
set(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib)

#uncomment if you have defined messages
#rosbuild_genmsg()
#uncomment if you have defined services
#rosbuild_gensrv()

#common commands for building c++ executables and libraries
#rosbuild_add_library(${PROJECT_NAME} src/example.cpp)
#target_link_libraries(${PROJECT_NAME} another_library)
#rosbuild_add_boost_directories()
#rosbuild_link_boost(${PROJECT_NAME} thread)
#rosbuild_add_executable(example examples/example.cpp)
#target_link_libraries(example ${PROJECT_NAME})

include_directories (src/bt) 


#CPP Libraries
rosbuild_add_library(tf2 src/cache.cpp src/buffer_core.cpp src/static_cache.cpp)
rosbuild_link_boost(tf2 thread signals)

#BEGIN Python Libraries
find_package(PythonLibs REQUIRED)
include_directories(${PYTHON_INCLUDE_PATH})

# Check for SSE
rosbuild_check_for_sse()

# Dynamic linking with tf worked OK, except for exception propagation, which failed in the unit test.
# so build with the objects directly instead.

link_libraries(${PYTHON_LIBRARIES})
rosbuild_add_library(tf2_py src/tf2_py.cpp)
target_link_libraries(tf2_py tf2)
find_package(PythonLibs REQUIRED)
set_target_properties(tf2_py PROPERTIES OUTPUT_NAME tf2 PREFIX "_" SUFFIX ".so")
# target_link_libraries(tf2_py tf)
rosbuild_add_compile_flags(tf2_py -g -Wno-missing-field-initializers)
rosbuild_add_compile_flags(tf2_py ${SSE_FLAGS}) #conditionally adds sse flags if available
#rosbuild_add_pyunit(test/testPython.py)
#END Python Libraries

rosbuild_add_gtest(test_cache_unittest test/cache_unittest.cpp)
target_link_libraries(test_cache_unittest tf2)

rosbuild_add_gtest(test_static_cache_unittest test/static_cache_test.cpp)
target_link_libraries(test_static_cache_unittest tf2)

#rosbuild_add_gtest(test/tf2_core test/tf2_core_unittest.cpp)
#target_link_libraries(test/tf2_core tf2)

rosbuild_add_gtest(test_simple test/simple_tf2_core.cpp)
target_link_libraries(test_simple tf2)
