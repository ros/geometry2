cmake_minimum_required(VERSION 2.8.3)
project(tf2_msgs)

find_package(catkin REQUIRED COMPONENTS message_generation geometry_msgs actionlib_msgs)
find_package(Boost COMPONENTS thread REQUIRED)

add_message_files(DIRECTORY msg FILES TF2Error.msg  TFMessage.msg)

add_action_files(DIRECTORY action FILES LookupTransform.action)
generate_messages(
  DEPENDENCIES actionlib_msgs std_msgs geometry_msgs
)

install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)


