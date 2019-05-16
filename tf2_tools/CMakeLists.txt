cmake_minimum_required(VERSION 2.8.3)
project(tf2_tools)

find_package(catkin REQUIRED COMPONENTS tf2
                                        tf2_msgs
                                        tf2_ros
                                        geometry_msgs
                                        roscpp
)

catkin_package(
   CATKIN_DEPENDS tf2
                  tf2_msgs
                  tf2_ros
                  geometry_msgs
                  roscpp
)

include_directories(${catkin_INCLUDE_DIRS})

add_executable(monitor src/monitor.cpp)
target_link_libraries(monitor ${catkin_LIBRARIES})

install(PROGRAMS scripts/view_frames.py scripts/echo.py
        DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(TARGETS monitor
    RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
