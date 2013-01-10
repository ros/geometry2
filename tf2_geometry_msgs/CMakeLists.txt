cmake_minimum_required(VERSION 2.8.3)
project(tf2_geometry_msgs)

find_package(catkin REQUIRED COMPONENTS geometry_msgs tf2_ros orocos_kdl tf2)
find_package(Boost COMPONENTS thread REQUIRED)

catkin_package(
   INCLUDE_DIRS include
   CATKIN_DEPENDS  geometry_msgs tf2_ros orocos_kdl tf2)


include_directories(include 
                    ${catkin_INCLUDE_DIRS}   
)

#!!add_executable(test_geometry_msgs test/test_tf2_geometry_msgs.cpp)
#!!catkin_add_gtest(${CMAKE_CURRENT_SOURCE_DIR}/test/test.launch)

#install(TARGETS test_geometry_msgs
#  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
#)

install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)

catkin_python_setup()
