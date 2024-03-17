cmake_minimum_required(VERSION 3.10)
project(capabilities2)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(capabilities2_msgs REQUIRED)
find_package(bond REQUIRED)

# Find SQLite3
find_package(SQLite3)
if (SQLITE3_FOUND)
  include_directories(${SQLITE3_INCLUDE_DIRS})
  target_link_libraries(your_target_name ${SQLITE3_LIBRARIES})
else()
  message(FATAL_ERROR "SQLite3 not found")
endif()

# find yaml-cpp
find_package(yaml-cpp REQUIRED)
if (yaml-cpp_FOUND)
  include_directories(${yaml-cpp_INCLUDE_DIRS})
  target_link_libraries(your_target_name ${yaml-cpp_LIBRARIES})
else()
  message(FATAL_ERROR "yaml-cpp not found")
endif()

include_directories(include)

add_executable(capabilities_server src/capabilities2_server_node.cpp)
ament_target_dependencies(capabilities_server rclcpp bond capabilities2_msgs sqlite3 yaml-cpp)

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
