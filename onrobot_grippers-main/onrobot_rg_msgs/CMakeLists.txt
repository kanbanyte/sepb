cmake_minimum_required(VERSION 3.5)
project(onrobot_rg_msgs)

find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(std_msgs REQUIRED)

set(msg_files
  "msg/OnRobotRGInput.msg"
  "msg/OnRobotRGOutput.msg"
  "srv/SetCommand.srv"
) 

rosidl_generate_interfaces(${PROJECT_NAME}
    ${msg_files}
    DEPENDENCIES std_msgs
)

ament_export_dependencies(rosidl_default_runtime)

ament_package()