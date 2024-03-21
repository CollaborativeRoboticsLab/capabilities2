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
find_package(bond REQUIRED)
find_package(capabilities2_msgs REQUIRED)
# find_package(TinyXML2 REQUIRED)

find_package(PkgConfig)
pkg_check_modules(TINYXML2 tinyxml2 REQUIRED)

# Find SQLite3
find_package(SQLite3)

# find yaml-cpp
find_package(yaml-cpp REQUIRED)

include_directories(
  include
  ${SQLITE3_INCLUDE_DIRS}
  ${yaml-cpp_INCLUDE_DIRS}
  ${TINYXML2_INCLUDE_DIRS}
)

add_executable(capabilities_server src/capabilities2_server_node.cpp)
ament_target_dependencies(capabilities_server rclcpp bond capabilities2_msgs SQLite3 yaml-cpp TINYXML2)
# target_link_libraries(capabilities_server
#   ${SQLITE3_LIBRARIES}
#   ${yaml-cpp_LIBRARIES}
#   ${TINYXML2_LIBRARIES}
# )

install(TARGETS
  capabilities_server
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}
)

install(DIRECTORY
  config
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
