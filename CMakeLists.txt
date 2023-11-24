cmake_minimum_required(VERSION 3.8)
project(tortoisebot_waypoints)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(tortoisebot_waypoints_interface REQUIRED)


# add source to libraries
include_directories(include)
set(HEADER_FILES include/tortoisebot_waypoints/tortoisebot_action_server.hpp)
set(SOURCES_FILES src/tortoisebot_action_server.cpp)
add_library(tortoisebot_waypoints_lib ${SOURCES_FILES} ${HEADER_FILES})

# export the targets
ament_target_dependencies(tortoisebot_waypoints_lib rclcpp geometry_msgs nav_msgs std_msgs tf2 rclcpp_action tortoisebot_waypoints_interface)
ament_export_targets(tortoisebot_waypoints_lib HAS_LIBRARY_TARGET)
# add source executables
add_executable(tortoisebot_action_server_node ${SOURCES_FILES})
ament_target_dependencies(tortoisebot_action_server_node rclcpp geometry_msgs nav_msgs std_msgs tf2 rclcpp_action tortoisebot_waypoints_interface)

install(TARGETS
	tortoisebot_action_server_node
	DESTINATION lib/${PROJECT_NAME}/
)

install(
	DIRECTORY include/
	DESTINATION include
)

install(
  TARGETS tortoisebot_waypoints_lib
  EXPORT tortoisebot_waypoints_lib
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
  RUNTIME DESTINATION bin
  INCLUDES DESTINATION include
)


if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # uncomment the line when a copyright and license is not present in all source files
  #set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # uncomment the line when this package is not in a git repo
  #set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
