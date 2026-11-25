cmake_minimum_required(VERSION 3.8)
project(ntou_hardware_interface)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)

add_executable(ntou_hardware_interface_node src/ntou_hardware_interface.cpp)
target_compile_features(ntou_hardware_interface_node PUBLIC cxx_std_17)
ament_target_dependencies(ntou_hardware_interface_node
  rclcpp
  geometry_msgs
  sensor_msgs
  std_msgs
)

install(TARGETS ntou_hardware_interface_node
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
