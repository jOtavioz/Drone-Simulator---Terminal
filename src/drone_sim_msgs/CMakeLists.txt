cmake_minimum_required(VERSION 3.8)
project(drone_sim_msgs)

find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/DroneState.msg"
  "msg/DroneCmd.msg"
)

ament_package()
