cmake_minimum_required(VERSION 2.8.3)
project(tf2_bullet)

find_package(PkgConfig REQUIRED)

set(bullet_FOUND 0)
pkg_check_modules(bullet REQUIRED bullet)

find_package(catkin REQUIRED COMPONENTS geometry_msgs rostest tf2 tf2_ros)

include_directories(include ${bullet_INCLUDE_DIRS} ${catkin_INCLUDE_DIRS})

catkin_package(INCLUDE_DIRS include ${bullet_INCLUDE_DIRS} ${catkin_INCLUDE_DIRS})

add_executable(test_bullet EXCLUDE_FROM_ALL test/test_tf2_bullet.cpp)
target_link_libraries(test_bullet ${catkin_LIBRARIES} ${GTEST_LIBRARIES})

add_rostest(${CMAKE_CURRENT_SOURCE_DIR}/test/test.launch)


install(DIRECTORY include/${PACKAGE_NAME}/
        DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION})

if(TARGET tests)
  add_dependencies(tests test_bullet)
endif()
