cmake_minimum_required(VERSION 3.10)
project(capabilities2_server)

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
find_package(bondcpp REQUIRED)
find_package(pluginlib REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(capabilities2_msgs REQUIRED)
find_package(capabilities2_runner REQUIRED)
# find_package(uuid REQUIRED)
# find_package(TinyXML2 REQUIRED)

find_package(PkgConfig)
pkg_check_modules(TINYXML2 tinyxml2 REQUIRED)
pkg_check_modules(UUID REQUIRED uuid)

# Find SQLite3
find_package(SQLite3)

# find yaml-cpp
find_package(yaml-cpp REQUIRED)

include_directories(
  include
  ${SQLITE3_INCLUDE_DIRS}
  ${YAML_CPP_INCLUDE_DIRS}
  ${TINYXML2_INCLUDE_DIRS}
  ${UUID_INCLUDE_DIRS}
)

add_library(${PROJECT_NAME} SHARED
  src/capabilities_server.cpp
)

target_link_libraries(${PROJECT_NAME}
  ${SQLITE3_LIBRARIES}
  ${YAML_CPP_LIBRARIES}
  ${TINYXML2_LIBRARIES}
  ${UUID_LIBRARIES}
)

ament_target_dependencies(${PROJECT_NAME}
  rclcpp
  rclcpp_action
  bondcpp
  pluginlib
  rclcpp_components
  capabilities2_msgs
  capabilities2_runner
  SQLite3
  yaml-cpp
  TINYXML2
  UUID
)

rclcpp_components_register_node(${PROJECT_NAME}
  PLUGIN "capabilities2_server::CapabilitiesServer"
  EXECUTABLE capabilities2_server_node
)

ament_export_targets(capabilities2_server_node)

install(TARGETS ${PROJECT_NAME}
  EXPORT capabilities2_server_node
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

# make test executable
add_executable(test_capabilities_server test/test.cpp)
target_link_libraries(test_capabilities_server
  ${PROJECT_NAME}
  ${SQLITE3_LIBRARIES}
  ${YAML_CPP_LIBRARIES}
  ${TINYXML2_LIBRARIES}
  ${UUID_LIBRARIES}
)
add_dependencies(test_capabilities_server ${PROJECT_NAME})

# install test executable
install(TARGETS test_capabilities_server
  RUNTIME DESTINATION bin
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

ament_package()
