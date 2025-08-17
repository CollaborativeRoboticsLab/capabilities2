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
find_package(rclcpp_components REQUIRED)
find_package(bondcpp REQUIRED)
find_package(pluginlib REQUIRED)
find_package(capabilities2_msgs REQUIRED)
find_package(capabilities2_runner REQUIRED)
find_package(capabilities2_utils REQUIRED)
find_package(event_logger REQUIRED)
find_package(event_logger_msgs REQUIRED)
find_package(tinyxml2_vendor REQUIRED)
find_package(TinyXML2 REQUIRED)  # provided by tinyxml2 upstream, or tinyxml2_vendor
find_package(backward_ros REQUIRED)

find_package(PkgConfig)
pkg_check_modules(UUID REQUIRED uuid)

# Find SQLite3
find_package(SQLite3)

# find yaml-cpp
find_package(yaml-cpp REQUIRED)

include_directories(include
  ${SQLITE3_INCLUDE_DIRS}
  ${YAML_CPP_INCLUDE_DIRS}
  ${UUID_INCLUDE_DIRS}
)

#######################################################################
# Component based implementation that compiles as a library
#######################################################################

add_library(${PROJECT_NAME}_comp SHARED
  src/capabilities_server_component.cpp
)

target_link_libraries(${PROJECT_NAME}_comp
  ${SQLITE3_LIBRARIES}
  ${YAML_CPP_LIBRARIES}
  ${UUID_LIBRARIES}
)

ament_target_dependencies(${PROJECT_NAME}_comp
  rclcpp
  rclcpp_action
  bondcpp
  pluginlib
  rclcpp_components
  capabilities2_msgs
  capabilities2_runner
  capabilities2_utils
  event_logger
  event_logger_msgs
  TinyXML2
  SQLite3
  yaml-cpp
  UUID
)

rclcpp_components_register_node(${PROJECT_NAME}_comp
  PLUGIN "capabilities2_server::CapabilitiesServer"
  EXECUTABLE capabilities2_server_component
)

ament_export_targets(capabilities2_server_component)

install(TARGETS ${PROJECT_NAME}_comp
  EXPORT capabilities2_server_component
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

############################################################################
# node implementation that compiles as a executable 
############################################################################

add_executable(${PROJECT_NAME}_node
  src/capabilities_server_node.cpp
)

target_link_libraries(${PROJECT_NAME}_node
  ${UUID_LIBRARIES}
  ${SQLITE3_LIBRARIES}
  ${YAML_CPP_LIBRARIES}
)

ament_target_dependencies(${PROJECT_NAME}_node
  rclcpp
  rclcpp_action
  bondcpp
  pluginlib
  capabilities2_msgs
  capabilities2_runner
  capabilities2_utils
  event_logger
  event_logger_msgs
  TinyXML2
  SQLite3
  yaml-cpp
  UUID
)

install(TARGETS ${PROJECT_NAME}_node
  DESTINATION lib/${PROJECT_NAME}
)

# make test executable

add_executable(test_capabilities_server test/test.cpp)

target_link_libraries(test_capabilities_server
  ${PROJECT_NAME}_comp
  ${SQLITE3_LIBRARIES}
  ${YAML_CPP_LIBRARIES}
  ${UUID_LIBRARIES}
)
add_dependencies(test_capabilities_server ${PROJECT_NAME}_comp)

############################################################################
# miscellaneous
############################################################################

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
