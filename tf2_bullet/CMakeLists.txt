cmake_minimum_required(VERSION 3.0.2)
project(tf2_bullet)

find_package(Bullet REQUIRED)

find_package(catkin REQUIRED COMPONENTS geometry_msgs tf2)

include_directories(include ${BULLET_INCLUDE_DIRS} ${catkin_INCLUDE_DIRS})

catkin_package(INCLUDE_DIRS include ${BULLET_INCLUDE_DIRS})

install(DIRECTORY include/${PROJECT_NAME}/
        DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION})


if(CATKIN_ENABLE_TESTING)

catkin_add_gtest(test_bullet test/test_tf2_bullet.cpp)
target_link_libraries(test_bullet ${catkin_LIBRARIES} ${GTEST_LIBRARIES})

endif()
