cmake_minimum_required(VERSION 2.8.3)
project(tf2_kdl)

find_package(orocos_kdl)
find_package(catkin REQUIRED COMPONENTS rostest tf2 tf2_ros tf2_msgs)

catkin_package(
  INCLUDE_DIRS include ${catkin_INCLUDE_DIRS}
)

catkin_python_setup()


include_directories(include ${catkin_INCLUDE_DIRS} ${GTEST_INCLUDE_DIRS})

catkin_add_gtest(test_kdl test/test_tf2_kdl.cpp)
find_package (Threads)
target_link_libraries(test_kdl ${catkin_LIBRARIES} ${GTEST_LIBRARIES} ${CMAKE_THREAD_LIBS_INIT})

add_rostest(${CMAKE_CURRENT_SOURCE_DIR}/test/test.launch)

install(DIRECTORY include/${PACKAGE_NAME}/
        DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION})

install(PROGRAMS scripts/test.py
         DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

