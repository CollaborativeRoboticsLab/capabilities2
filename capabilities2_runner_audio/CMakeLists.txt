cmake_minimum_required(VERSION 3.8)
project(capabilities2_runner_audio)

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
find_package(pluginlib REQUIRED)
find_package(hri_audio_msgs REQUIRED)
find_package(capabilities2_msgs REQUIRED)
find_package(capabilities2_runner REQUIRED)
find_package(PkgConfig)
pkg_check_modules(TINYXML2 tinyxml2 REQUIRED)

include_directories(include
${TINYXML2_INCLUDE_DIRS}
)

add_library(${PROJECT_NAME} SHARED
  src/audio_runners.cpp
)

target_link_libraries(${PROJECT_NAME}
  ${TINYXML2_LIBRARIES}
)

ament_target_dependencies(${PROJECT_NAME}
  rclcpp
  rclcpp_action
  pluginlib
  hri_audio_msgs
  capabilities2_msgs
  capabilities2_runner
  TINYXML2
)

pluginlib_export_plugin_description_file(${PROJECT_NAME} plugins.xml)

install(TARGETS ${PROJECT_NAME}
  EXPORT ${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(DIRECTORY include/
  DESTINATION include
)

ament_export_include_directories(include)
ament_export_libraries(${PROJECT_NAME})

ament_package()
