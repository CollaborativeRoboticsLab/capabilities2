#pragma once

#include <capabilities2_runner/notrigger_runner.hpp>

namespace capabilities2_runner
{

/**
 * @brief launch runner base class
 *
 * Create a launch file runner to run a launch file based capability
 * uses process execution to run the launch file and kill the process on stop
 */
class LaunchRunner : public NoTriggerRunner
{
public:
  /**
   * @brief Constructor which needs to be empty due to plugin semantics
   */
  LaunchRunner() : NoTriggerRunner()
  {
  }

  /**
   * @brief Starter function for starting the launch runner
   *
   * @param node shared pointer to the capabilities node. Allows to use ros node related functionalities
   * @param run_config runner configuration loaded from the yaml file
   * @param bond_id bond identifier for the runner
   */
  virtual void start(rclcpp::Node::SharedPtr node, const runner_opts& run_config, const std::string& bond_id) override
  {
    init_base(node, run_config);

    package_name = run_config_.runner.substr(0, run_config_.runner.find("/"));
    launch_name = run_config_.runner.substr(run_config_.runner.find("/") + 1);

    // start launch process
    throw runner_exception("launch runner not implemented yet");

    // emit started event
    emit_started(bond_id, "", param_on_started());
  }

  /**
   * @brief stop function to cease functionality and shutdown
   *
   */
  virtual void stop(const std::string& bond_id, const std::string& instance_id = "") override
  {
    // if the node pointer is empty then throw an error
    // this means that the runner was not started and is being used out of order

    if (!node_)
      throw runner_exception("cannot stop runner that was not started");

    // stop the launch process
    throw runner_exception("launch runner not implemented yet");

    // emit stopped event
    emit_stopped(bond_id, instance_id, param_on_stopped());
  }

protected:
  std::string launch_name;
  std::string package_name;
};

}  // namespace capabilities2_runner
