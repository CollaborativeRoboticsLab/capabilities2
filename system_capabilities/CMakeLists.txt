cmake_minimum_required(VERSION 3.8)
project(system_capabilities)

find_package(ament_cmake REQUIRED)

# install interface files
install(DIRECTORY interfaces
  DESTINATION share/${PROJECT_NAME}
)

# install semantic interface files
install(DIRECTORY providers
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
