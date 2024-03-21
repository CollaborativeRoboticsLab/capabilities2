#pragma once

#include <memory>
#include <map>
#include <string>
#include <vector>

#include <capabilities2_server/capabilities_db.hpp>
#include <capabilities2_server/bond_cache.hpp>
#include <capabilities2_server/runner_cache.hpp>

#include <bond/msg/status.hpp>
#include <capabilities2_msgs/msg/remapping.hpp>
#include <capabilities2_msgs/msg/capability_spec.hpp>
#include <capabilities2_msgs/msg/capability_event.hpp>

namespace capabilities2_server
{

/**
 * @brief capabilities api
 * mapping message based logic of capabilities to database models
 * handles message conversion and db access
 * also tracks cache for bonds and runners
 *
 */
class CapabilitiesAPI
{
public:
  CapabilitiesAPI()
  {
  }

  // connect db
  void connect(const std::string& db_file)
  {
    cap_db_ = std::make_unique<CapabilitiesDB>(db_file);
  }

  // control api
  void start_capability(const std::string& capability, const std::string& provider)
  {
    // create a runner id
    std::string runner_id = "runner_" + capability + "_" + provider;

    // create a new runner
    runner_cache_.add_runner(runner_id, capability);

    // XXX TODO: start the runner
  }

  const bool stop_capability(const std::string& capability)
  {
    // get runners
    std::vector<std::string> runners = runner_cache_.get_runners(capability);

    // stop runners
    for (const auto& runner : runners)
    {
      // XXX TODO: stop the runner
      runner_cache_.remove_runner(runner);
    }
  }

  void free_capability(const std::string& capability, const std::string& bond_id);
  void use_capability(const std::string& capability, const std::string& provider, const std::string& bond_id);

  const std::string establish_bond(const std::string& capability, const std::string& bond_id);
  {
    bond_cache_.add_bond(capability, bond_id);
  }

  // capability api
  void add_capability();

  // query api
  std::vector<std::string> get_interfaces();
  std::vector<std::string> get_sematic_interfaces(const std::string& interface);
  std::vector<std::string> get_providers(const std::string& interface, const bool& include_semantic);
  const capabilities2_msgs::msg::CapabilitySpec get_capability_spec(const std::string& capability_spec);
  std::vector<capabilities2_msgs::msg::CapabilitySpec> get_capability_specs();
  std::vector<capabilities2_msgs::msg::Remapping> get_remappings(const std::string& capability_spec);

  // runner api
  std::vector<std::string> get_running_capabilities();

  // event api
  const capabilities2_msgs::msg::CapabilityEvent on_capability_started();
  const capabilities2_msgs::msg::CapabilityEvent on_capability_stopped();
  const capabilities2_msgs::msg::CapabilityEvent on_capability_used();
  const capabilities2_msgs::msg::CapabilityEvent on_capability_freed();
  const capabilities2_msgs::msg::CapabilityEvent on_bond_established();
  const capabilities2_msgs::msg::CapabilityEvent on_bond_broken();
  const capabilities2_msgs::msg::CapabilityEvent on_capability_error();

private:
  std::unique_ptr<CapabilitiesDB> cap_db_;
  BondCache bond_cache_;
  RunnerCache runner_cache_;
};
}  // namespace capabilities2_server
