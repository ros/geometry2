cmake_minimum_required(VERSION 2.8.3)
project(tf2_eigen)

find_package(catkin REQUIRED COMPONENTS
  cmake_modules
  geometry_msgs
  tf2
)

find_package(Eigen REQUIRED)
add_definitions(${EIGEN_DEFINITIONS})

include_directories(include)
include_directories(${EIGEN_INCLUDE_DIRS})
include_directories(${catkin_INCLUDE_DIRS})


catkin_package(
  INCLUDE_DIRS include
  DEPENDS eigen)

install(DIRECTORY include/${PROJECT_NAME}/
        DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION})


if(CATKIN_ENABLE_TESTING)

  catkin_add_gtest(tf2_eigen-test test/tf2_eigen-test.cpp)
  target_link_libraries(tf2_eigen-test ${catkin_LIBRARIES} ${GTEST_LIBRARIES})

endif()
