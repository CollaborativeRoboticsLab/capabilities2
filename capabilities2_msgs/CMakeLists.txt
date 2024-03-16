cmake_minimum_required(VERSION 3.10)
project(capabilities2_msgs)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(std_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(action_msgs REQUIRED)

set(msg_files
  "msg/Capability.msg"
  "msg/CapabilityEvent.msg"
  "msg/CapabilitySpec.msg"
  "msg/Remapping.msg"
  "msg/RunningCapability.msg"
  "msg/NaturalCapability.msg"
  "msg/CapabilityCommand.msg"
  "msg/CapabilityResponse.msg"
)

set(srv_files
  "srv/EstablishBond.srv"
  "srv/GetCapabilitySpec.srv"
  "srv/FreeCapability.srv"
  "srv/GetCapabilitySpecs.srv"
  "srv/GetInterfaces.srv"
  "srv/GetNodeletManagerName.srv"
  "srv/GetProviders.srv"
  "srv/GetRemappings.srv"
  "srv/GetRunningCapabilities.srv"
  "srv/GetSemanticInterfaces.srv"
  "srv/RegisterCapability.srv"
  "srv/RegisterInterface.srv"
  "srv/RegisterProvider.srv"
  "srv/RegisterSemanticInterface.srv"
  "srv/StartCapability.srv"
  "srv/StopCapability.srv"
  "srv/UseCapability.srv"
)

set(action_files
  "action/Capability.action"
)

rosidl_generate_interfaces(${PROJECT_NAME}
  ${msg_files}
  ${srv_files}
  ${action_files}
  DEPENDENCIES builtin_interfaces std_msgs std_srvs action_msgs
)

ament_package()
