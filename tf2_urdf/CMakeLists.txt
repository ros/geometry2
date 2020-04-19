cmake_minimum_required(VERSION 2.8.3)
project(tf2_urdf)

find_package(catkin REQUIRED COMPONENTS
    geometry_msgs
    tf2
    tf2_geometry_msgs
)

find_package(urdfdom_headers REQUIRED)

catkin_package(
    INCLUDE_DIRS
        include
    DEPENDS
        urdfdom_headers
)

include_directories(
    include
    ${catkin_INCLUDE_DIRS}
    ${urdfdom_headers_INCLUDE_DIRS}
)

install(
    DIRECTORY include/${PROJECT_NAME}/
    DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)

if(CATKIN_ENABLE_TESTING)

    catkin_add_gtest(tf2_urdf-test test/tf2_urdf-test.cpp)
    target_link_libraries(tf2_urdf-test ${catkin_LIBRARIES} ${GTEST_LIBRARIES})

endif()