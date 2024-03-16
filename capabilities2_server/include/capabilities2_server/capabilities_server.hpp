#pragma once

#include <rclcpp/rclcpp.hpp>

namespace capabilities2_server
{
class CapabilitiesAPI
{
public:
  CapabilitiesAPI();
  ~CapabilitiesAPI();

  // control api
  void start_capability();
  void stop_capability();
  void free_capability();
  void use_capability();
  void establish_bond();

  // query api
  void get_interfaces();
  void get_sematic_interfaces();
  void get_providers();
  void get_capability_spec();
  void get_capability_specs();
  void get_remappings();

  // runner api
  void get_runner_status();
  void get_running_capabilities();

  // event api
  void on_capability_started();
  void on_capability_stopped();
  void on_capability_used();
  void on_capability_freed();
  void on_bond_established();
  void on_bond_broken();
  void on_capability_error();

private:
};

class CapabilitiesServer : public rclcpp::Node
{
public:
  CapabilitiesServer() : Node("capabilities_server")
  {
    // load capability models
  }

private:
  CapabilitiesAPI API;
};

}  // namespace capabilities2_server
