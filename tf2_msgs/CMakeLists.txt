cmake_minimum_required(VERSION 2.8.3)
project(tf2_msgs)

if(NOT WIN32)
  add_definitions(-std=c++11)
endif()

find_package(ament_cmake)
find_package(rosidl_default_generators REQUIRED)
find_package(builtin_interfaces REQUIRED)
#find_package(actionlib_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)

## TODO(tfoote) add actions back
#add_action_files(DIRECTORY action FILES LookupTransform.action)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/TF2Error.msg"
  "msg/TFMessage.msg"
  "srv/FrameGraph.srv"
  DEPENDENCIES builtin_interfaces std_msgs geometry_msgs
)
#  actionlib_msgs

# TODO(tfoote) Need amend_export_dependenceies?
ament_export_dependencies(rosidl_generator_cpp)
ament_package()
