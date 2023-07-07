cmake_minimum_required(VERSION 3.10.2)
project(capabilities2_msgs)

find_package(catkin REQUIRED COMPONENTS
  std_msgs
  actionlib_msgs
  message_generation
)

add_message_files(
  FILES
  Capability.msg
  CapabilityCommand.msg
  CapabilityResponse.msg
)

add_service_files(
  FILES
  Prompt.srv
)

add_action_files(
  DIRECTORY action
  FILES Capability.action
)

generate_messages(
  DEPENDENCIES
  std_msgs
  actionlib_msgs
)

catkin_package(CATKIN_DEPENDS message_runtime std_msgs actionlib_msgs)
