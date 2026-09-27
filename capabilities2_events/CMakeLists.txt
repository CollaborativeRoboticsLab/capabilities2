cmake_minimum_required(VERSION 3.16)
project(capabilities2_events)

# Default to C++17
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_gtest REQUIRED)
find_package(rclcpp REQUIRED)
find_package(capabilities2_msgs REQUIRED)

# uuid
find_package(PkgConfig)
pkg_check_modules(UUID REQUIRED uuid)

include_directories(include ${UUID_INCLUDE_DIRS})

# build library
add_library(${PROJECT_NAME} SHARED
  src/uuid_generator.cpp
)

target_link_libraries(${PROJECT_NAME} ${UUID_LIBRARIES})

ament_target_dependencies(${PROJECT_NAME}
  rclcpp
  capabilities2_msgs
  UUID
)

# install headers
install(DIRECTORY include/
  DESTINATION include
)

# install library
install(TARGETS ${PROJECT_NAME}
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

if(BUILD_TESTING)
  ament_add_gtest(published_event_test
    test/published_event_test.cpp
  )
  if(TARGET published_event_test)
    target_include_directories(published_event_test PUBLIC include)
    target_link_libraries(published_event_test ${PROJECT_NAME})
    ament_target_dependencies(published_event_test
      rclcpp
      capabilities2_msgs
    )
  endif()
endif()

ament_export_include_directories(include)
ament_export_dependencies(rclcpp capabilities2_msgs)
ament_export_libraries(${PROJECT_NAME})

ament_package()
