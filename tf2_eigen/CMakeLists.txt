cmake_minimum_required(VERSION 2.8.3)
project(tf2_eigen)

find_package(catkin REQUIRED COMPONENTS
  eigen
  geometry_msgs
  cmake_modules
  tf2
)

find_package(catkin REQUIRED COMPONENTS geometry_msgs tf2)

find_package(Eigen REQUIRED)
include_directories(${EIGEN_INCLUDE_DIRS})
add_definitions(${EIGEN_DEFINITIONS})

catkin_package(
  INCLUDE_DIRS include ${catkin_INCLUDE_DIRS}
  DEPENDS eigen)

install(DIRECTORY include/${PROJECT_NAME}/
        DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION})


if(CATKIN_ENABLE_TESTING)

  catkin_add_gtest(tf2_eigen-test test/tf2_eigen-test.cpp)
  target_link_libraries(tf2_eigen-test ${catkin_LIBRARIES} ${GTEST_LIBRARIES})

endif()