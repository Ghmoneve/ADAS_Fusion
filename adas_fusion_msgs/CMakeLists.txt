cmake_minimum_required(VERSION 3.8)
project(adas_fusion_msgs)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/Detection2D.msg"
  "msg/Detection2DArray.msg"
  "msg/RadarObject.msg"
  "msg/RadarObjectArray.msg"
  "msg/TrackedObject.msg"
  "msg/TrackedObjectArray.msg"
  DEPENDENCIES std_msgs geometry_msgs
)

ament_export_dependencies(rosidl_default_runtime)
ament_package()
