cmake_minimum_required(VERSION 3.0.2)
project(onrobot_vg_description)
find_package(catkin REQUIRED)
catkin_package()

find_package(ament_cmake REQUIRED)

install(
  DIRECTORY launch meshes urdf
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
