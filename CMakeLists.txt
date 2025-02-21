cmake_minimum_required(VERSION 3.8)
project(sensor_compute)

# find required packages
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)

# ros idl custom messages
rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/SensorTimestamp.msg"
  DEPENDENCIES std_msgs
)

find_package(rosidl_default_runtime REQUIRED)
ament_export_dependencies(rosidl_default_runtime)
rosidl_get_typesupport_target(cpp_typesupport_target ${PROJECT_NAME} "rosidl_typesupport_cpp")

# include header files
include_directories(include)

# adding source files
add_executable(sensor_node src/sensor_node.cpp)
target_link_libraries(sensor_node ${cpp_typesupport_target})
ament_target_dependencies(sensor_node rclcpp std_msgs rosidl_default_runtime)

add_executable(central_node src/central_node.cpp)
target_link_libraries(central_node ${cpp_typesupport_target})
ament_target_dependencies(central_node rclcpp std_msgs rosidl_default_runtime)

# install target and launch
install(TARGETS sensor_node central_node DESTINATION lib/${PROJECT_NAME})
install(
  DIRECTORY launch/
  DESTINATION share/${PROJECT_NAME}/
)
ament_package()
