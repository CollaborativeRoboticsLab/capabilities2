cmake_minimum_required(VERSION 3.8)
project(capabilities2_executor)

# Default to C++17
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(capabilities2_msgs REQUIRED)

find_package(PkgConfig)
pkg_check_modules(TINYXML2 tinyxml2 REQUIRED)

include_directories(
  include
  ${TINYXML2_INCLUDE_DIRS}
)

add_executable(${PROJECT_NAME}
  src/capabilities_executor.cpp
)

target_link_libraries(${PROJECT_NAME}
  ${TINYXML2_LIBRARIES}
)

ament_target_dependencies(${PROJECT_NAME}
  rclcpp
  rclcpp_action
  capabilities2_msgs
  TINYXML2
)

install(TARGETS ${PROJECT_NAME}
    DESTINATION lib/${PROJECT_NAME}
)

add_executable(${PROJECT_NAME}_file
  src/capabilities_file_parser.cpp
)

target_link_libraries(${PROJECT_NAME}_file
  ${TINYXML2_LIBRARIES}
)

ament_target_dependencies(${PROJECT_NAME}_file
  rclcpp
  rclcpp_action
  capabilities2_msgs
  TINYXML2
)

install(TARGETS ${PROJECT_NAME}_file
    DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY include/
  DESTINATION include
)

install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

install(DIRECTORY config
  DESTINATION share/${PROJECT_NAME}
)

install(DIRECTORY plans
  DESTINATION share/${PROJECT_NAME}
)


ament_package()
