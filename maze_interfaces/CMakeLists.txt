cmake_minimum_required(VERSION 3.8)
project(maze_interfaces)

# Message and service definitions shared by the maze robot's nodes.
# rosidl generates the Python and C++ classes for every file listed here.
find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_msgs REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/DeliveryObject.msg"
  "msg/DeliveryObjectArray.msg"
  "msg/ExitBin.msg"
  "msg/ExitBinArray.msg"
  "msg/PlanStep.msg"
  "msg/DeliveryPlan.msg"
  "srv/GetDeliveryPlan.srv"
  DEPENDENCIES geometry_msgs std_msgs
)

ament_export_dependencies(rosidl_default_runtime)
ament_package()
