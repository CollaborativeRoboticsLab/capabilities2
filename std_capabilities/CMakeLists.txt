cmake_minimum_required(VERSION 3.10)
project(std_capabilities)
find_package(ament_cmake REQUIRED)

# install launch files
install(DIRECTORY launch
DESTINATION share/${PROJECT_NAME}
)

ament_package()
