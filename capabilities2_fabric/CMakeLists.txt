cmake_minimum_required(VERSION 3.8)
project(capabilities2_fabric)

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
find_package(bondcpp REQUIRED)
find_package(backward_ros REQUIRED)

# Locate the static version of tinyxml2
find_library(TINYXML2_LIB NAMES tinyxml2 PATHS /usr/local/lib NO_DEFAULT_PATH)

if(NOT TINYXML2_LIB)
  message(FATAL_ERROR "tinyxml2 library not found. Make sure it's installed as a static library.")
endif()

include_directories(include
  /usr/local/include
)

add_executable(${PROJECT_NAME}
  src/capabilities_fabric.cpp
)

target_link_libraries(${PROJECT_NAME}
  ${TINYXML2_LIB}
)

ament_target_dependencies(${PROJECT_NAME}
  rclcpp
  rclcpp_action
  bondcpp
  capabilities2_msgs
)

install(TARGETS ${PROJECT_NAME}
  DESTINATION lib/${PROJECT_NAME}
)

add_executable(fabric_client
  src/fabric_client.cpp
)

target_link_libraries(fabric_client
  ${TINYXML2_LIB}
)

ament_target_dependencies(fabric_client
  rclcpp
  rclcpp_action
  capabilities2_msgs
)

install(TARGETS fabric_client
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
