cmake_minimum_required(VERSION 2.8.3)
project(tf2)

find_package(catkin REQUIRED COMPONENTS geometry_msgs tf2_msgs rospy roscpp rosconsole)
find_package(Boost REQUIRED COMPONENTS thread signals)

catkin_python_setup()

catkin_package(
   INCLUDE_DIRS include
   LIBRARIES tf2 
   CATKIN_DEPENDS geometry_msgs tf2_msgs rospy roscpp)

include_directories(include ${catkin_INCLUDE_DIRS})
include_directories (src/bt) 

add_definitions(-DBT_USE_DOUBLE_PRECISION)
# export user definitions

#CPP Libraries
add_library(tf2 src/cache.cpp src/buffer_core.cpp src/static_cache.cpp)
target_link_libraries(tf2 ${Boost_LIBRARIES} ${catkin_LIBRARIES})
add_dependencies(tf2 tf2_msgs_gencpp)

#BEGIN Python Libraries
find_package(PythonLibs REQUIRED)
include_directories(${PYTHON_INCLUDE_PATH})

# Check for SSE
#!!! rosbuild_check_for_sse()

# Dynamic linking with tf worked OK, except for exception propagation, which failed in the unit test.
# so build with the objects directly instead.

link_libraries(${PYTHON_LIBRARIES})
add_library(tf2_py src/tf2_py.cpp)
target_link_libraries(tf2_py tf2 ${catkin_LIBRARIES})
add_dependencies(tf2_py tf2_msgs_gencpp)

set_target_properties(tf2_py PROPERTIES OUTPUT_NAME tf2 PREFIX "_" SUFFIX ".so")
set_target_properties(tf2_py PROPERTIES COMPILE_FLAGS "-g -Wno-missing-field-initializers")
set_target_properties(tf2_py PROPERTIES
  ARCHIVE_OUTPUT_DIRECTORY ${CATKIN_DEVEL_PREFIX}/${CATKIN_PACKAGE_PYTHON_DESTINATION}
  LIBRARY_OUTPUT_DIRECTORY ${CATKIN_DEVEL_PREFIX}/${CATKIN_PACKAGE_PYTHON_DESTINATION}
)
#!! rosbuild_add_compile_flags(tf2_py ${SSE_FLAGS}) #conditionally adds sse flags if available
#END Python Libraries

catkin_add_gtest(test_cache_unittest test/cache_unittest.cpp)
target_link_libraries(test_cache_unittest tf2)
add_dependencies(test_cache_unittest tf2_msgs_gencpp)

catkin_add_gtest(test_static_cache_unittest test/static_cache_test.cpp)
target_link_libraries(test_static_cache_unittest tf2)
add_dependencies(test_static_cache_unittest tf2_msgs_gencpp)

catkin_add_gtest(test_simple test/simple_tf2_core.cpp)
target_link_libraries(test_simple tf2)
add_dependencies(test_simple tf2_msgs_gencpp)

add_executable(speed_test EXCLUDE_FROM_ALL test/speed_test.cpp)
target_link_libraries(speed_test tf2)
add_dependencies(tests speed_test)
add_dependencies(tests tf2_msgs_gencpp)

install(TARGETS tf2
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)

install(FILES ${CATKIN_BUILD_PREFIX}/${CATKIN_PACKAGE_PYTHON_DESTINATION}/_tf2.so
  DESTINATION ${CATKIN_PACKAGE_PYTHON_DESTINATION}
)