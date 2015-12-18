cmake_minimum_required(VERSION 2.8.3)
project(tf2)

if(NOT WIN32)
  add_definitions(-std=c++11)
endif()

find_package(ament_cmake)
find_package(console_bridge REQUIRED)
find_package(geometry_msgs REQUIRED)

include_directories(include src/bt ${geometry_msgs_INCLUDE_DIRS} ${console_bridge_INCLUDE_DIRS})

# export user definitions

#CPP Libraries
add_library(tf2 src/cache.cpp src/buffer_core.cpp src/static_cache.cpp)
target_link_libraries(tf2 ${geometry_msgs_LIBRARIES} ${console_bridge_LIBRARIES})

install(TARGETS tf2
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin/${PROJECT_NAME}
)

install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION include/${PROJECT_NAME}
)

# Tests
if(AMENT_ENABLE_TESTING)
  # TODO(tfoote) enable linters
  # find_package(ament_lint_auto REQUIRED)
  # ament_lint_auto_find_test_dependencies()
  find_package(ament_cmake_gtest)
     
  ament_add_gtest(test_cache_unittest test/cache_unittest.cpp)
  if(TARGET test_cache_unittest)
    target_link_libraries(test_cache_unittest tf2 ${geometry_msgs_LIBRARIES} ${console_bridge_LIBRARIES})
  endif()
  
  ament_add_gtest(test_static_cache_unittest test/static_cache_test.cpp)
  if(TARGET test_static_cache_unittest)
    target_link_libraries(test_static_cache_unittest tf2 ${geometry_msgs_LIBRARIES} ${console_bridge_LIBRARIES})
  endif()
  
  ament_add_gtest(test_simple test/simple_tf2_core.cpp)
  if(TARGET test_simple)
    target_link_libraries(test_simple tf2  ${geometry_msgs_LIBRARIES} ${console_bridge_LIBRARIES})
  endif()

# TODO(tfoote) reimplement speed test without dependency on message datatypes.
# add_executable(speed_test EXCLUDE_FROM_ALL test/speed_test.cpp)
# target_link_libraries(speed_test tf2  ${geometry_msgs_LIBRARIES} ${console_bridge_LIBRARIES})
# add_dependencies(tests speed_test)

endif()

ament_export_dependencies(console_bridge)
ament_export_include_directories(include)
ament_export_libraries(tf2)
ament_package()
