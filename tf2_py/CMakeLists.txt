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

find_package(PythonLibs REQUIRED)
include_directories(${PYTHON_INCLUDE_PATH})

# Check for SSE
rosbuild_check_for_sse()

# Dynamic linking with tf worked OK, except for exception propagation, which failed in the unit test.
# so build with the objects directly instead.

link_libraries(${PYTHON_LIBRARIES})
rosbuild_add_library(tf_py src/tf_py.cpp)
find_package(PythonLibs REQUIRED)
set_target_properties(tf_py PROPERTIES OUTPUT_NAME tf2 PREFIX "_" SUFFIX ".so")
# target_link_libraries(tf_py tf)
rosbuild_add_compile_flags(tf_py -g -Wno-missing-field-initializers)
rosbuild_add_compile_flags(tf_py ${SSE_FLAGS}) #conditionally adds sse flags if available
#rosbuild_add_pyunit(test/testPython.py)
