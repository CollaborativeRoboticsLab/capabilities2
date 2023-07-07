cmake_minimum_required(VERSION 3.10.2)
project(capabilities2_msgs)

find_package(catkin REQUIRED COMPONENTS
  std_msgs
  std_srvs
  actionlib_msgs
  message_generation
)

add_message_files(
  FILES
  Capability.msg
  CapabilityEvent.msg
  CapabilitySpec.msg
  Remapping.msg
  RunningCapability.msg
  NaturalCapability.msg
  CapabilityCommand.msg
  CapabilityResponse.msg
)

add_service_files(
  FILES
  EstablishBond.srv
  GetCapabilitySpec.srv
  FreeCapability.srv
  GetCapabilitySpecs.srv
  GetInterfaces.srv
  GetNodeletManagerName.srv
  GetProviders.srv
  GetRemappings.srv
  GetRunningCapabilities.srv
  GetSemanticInterfaces.srv
  RegisterCapability.srv
  RegisterInterface.srv
  RegisterProvider.srv
  RegisterSemanticInterface.srv
  StartCapability.srv
  StopCapability.srv
  UseCapability.srv
)

add_action_files(
  DIRECTORY action
  FILES Capability.action
)

generate_messages(
  DEPENDENCIES
  std_msgs
  std_srvs
  actionlib_msgs
)

catkin_package(CATKIN_DEPENDS message_runtime std_msgs std_srvs actionlib_msgs)
