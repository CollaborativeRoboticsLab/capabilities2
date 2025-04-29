cmake_minimum_required(VERSION 3.8)
project(capabilities2_events)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()


find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(capabilities2_msgs REQUIRED)

install(DIRECTORY include/
  DESTINATION include
)

ament_package()
