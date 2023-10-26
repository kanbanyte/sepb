cmake_minimum_required(VERSION 3.8)
project(pick_place_interfaces)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
	add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(std_srvs REQUIRED)

set(srv_files
	"srv/PickPlaceService.srv"
)

set(action_files
	"action/PickPlaceAction.action"
)

rosidl_generate_interfaces(${PROJECT_NAME}
	${srv_files}
	${action_files}
	# DEPENDENCIES std_srvs
)

ament_export_dependencies(rosidl_default_runtime)

ament_package()
