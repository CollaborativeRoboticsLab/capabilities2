cmake_minimum_required(VERSION 3.8)
project(capabilities2_launch)

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(capabilities2_msgs REQUIRED)

include_directories(
  include
)

add_executable(${PROJECT_NAME} 
  src/launch_server.cpp
)

ament_target_dependencies(${PROJECT_NAME} 
  rclcpp 
  capabilities2_msgs
)

install(TARGETS
  ${PROJECT_NAME} 
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY include/
  DESTINATION include
)

ament_package()
