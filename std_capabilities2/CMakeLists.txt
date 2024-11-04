cmake_minimum_required(VERSION 3.8)
project(std_capabilities2)

find_package(ament_cmake REQUIRED)

# install interface files
install(DIRECTORY interfaces
  DESTINATION share/${PROJECT_NAME}
)

# install launch files
install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

# install providers files
install(DIRECTORY providers
  DESTINATION share/${PROJECT_NAME}
)

ament_export_include_directories(include)
ament_export_libraries(${PROJECT_NAME})

ament_package()
