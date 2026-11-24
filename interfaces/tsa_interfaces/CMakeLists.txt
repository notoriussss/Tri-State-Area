cmake_minimum_required(VERSION 3.8)
project(tsa_interfaces)

find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/CubeObservation.msg"
  "msg/RobotObservation.msg"
  "msg/WorldState.msg"
  "msg/RobotSensors.msg"
  "msg/Obstacle.msg"
  "msg/RobotObstacles.msg"
  "msg/VirtualSensors.msg"
  "msg/RobotStatus.msg"
  "msg/Pose2D.msg"
  "msg/RobotPlan.msg"
  "msg/FleetPlan.msg"
  "srv/GetWorld.srv"
  "srv/CorridorFree.srv"
  DEPENDENCIES builtin_interfaces std_msgs geometry_msgs
)

ament_export_dependencies(rosidl_default_runtime)
ament_package()
