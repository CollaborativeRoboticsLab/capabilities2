#pragma once

#include <map>
#include <string>
#include <vector>
#include <functional>
#include <tinyxml2.h>
#include <rclcpp/rclcpp.hpp>
#include <pluginlib/class_loader.hpp>
#include <capabilities2_server/models/run_config.hpp>
#include <capabilities2_runner/runner_base.hpp>

namespace capabilities2_server
{

/**
 * @brief runner cache class
 * responsible for caching the currently active runners of capabilities
 * this cache deals directly with the provider spec of the capability framework
 * to determine the type of runner to use
 *
 * There are two main types of runners:
 * 1. launch file runner
 * 2. action runner
 *
 */
class RunnerCache
{
public:
  RunnerCache() : runner_loader_("capabilities2_runner", "capabilities2_runner::RunnerBase")
  {
    // on_started = nullptr;
    // on_stopped = nullptr;
    // on_failure = nullptr;
    // on_success = nullptr;
  }

  /**
   * @brief connect with ROS node logging interface
   *
   * @param node_logging_interface_ptr pointer to the ROS node logging interface
   */
  void connect(rclcpp::node_interfaces::NodeLoggingInterface::SharedPtr node_logging_interface_ptr)
  {
    // set logger
    node_logging_interface_ptr_ = node_logging_interface_ptr;
  }

  /**
   * @brief Add a runner to the cache
   *
   * @param node   pointer to the origin node, generally the capabilities2_server
   * @param capability capability name to be loaded
   * @param run_config run_config of the runner to be loaded
   */
  void add_runner(rclcpp::Node::SharedPtr node, const std::string& capability,
                  const models::run_config_model_t& run_config)
  {
    // if the runner exists then throw an error
    if (running(capability))
    {
      // already running
      throw capabilities2_runner::runner_exception("capability is running already: " + capability);
      // return;
    }

    // check if run config is valid
    if (!run_config.is_valid())
    {
      throw capabilities2_runner::runner_exception("run config is not valid: " + YAML::Dump(run_config.to_yaml()));
    }

    // create the runner
    // add the runner to map
    // if the spec runner contains a path to a launch file then use the launch file runner
    if (run_config.runner.find(".launch") != std::string::npos || run_config.runner.find("/") != std::string::npos ||
        run_config.runner.find(".py") != std::string::npos)
    {
      runner_cache_[capability] = runner_loader_.createSharedInstance("capabilities2_runner::LaunchRunner");
    }
    else
    {
      // use different runner types based on cap and provider specs
      runner_cache_[capability] = runner_loader_.createSharedInstance(run_config.runner);
    }

    // start the runner
    runner_cache_[capability]->start(node, run_config.to_runner_opts());
  }

  /**
   * @brief Trigger a runner in the cache
   *
   * xml parameters are used
   *
   * @param capability capability name to be loaded
   * @param parameters parameters related to the runner in std::string form for compatibility accross various runners
   */
  void trigger_runner(const std::string& capability, const std::string& parameters)
  {
    // is the runner in the cache
    if (running(capability))
    {
      runner_cache_[capability]->trigger(parameters);
    }
    else
    {
      RCLCPP_ERROR(node_logging_interface_ptr_->get_logger(), "Runner not found for capability: %s",
                   capability.c_str());
      throw capabilities2_runner::runner_exception("capability runner not found: " + capability);
    }
  }

  /**
   * @brief Set triggers for `on_success`, `on_failure`, `on_start`, `on_stop` events
   *
   *
   * @param capability capability from where the events originate
   * @param on_started_capability capability triggered by on_start event
   * @param on_started_parameters parameters related to capability triggered by on_start event
   * @param on_stopped_capability capability triggered by on_stop event
   * @param on_stopped_parameters parameters related to capability triggered by on_stop event
   * @param on_success_capability capability triggered by on_success event
   * @param on_success_parameters parameters related to capability triggered by on_success event
   * @param on_failure_capability capability triggered by on_failure event
   * @param on_failure_parameters parameters related to capability triggered by on_failure event
   */
  void set_runner_triggers(const std::string& capability, const std::string& on_started_capability,
                           const std::string& on_started_parameters, const std::string& on_failure_capability,
                           const std::string& on_failure_parameters, const std::string& on_success_capability,
                           const std::string& on_success_parameters, const std::string& on_stopped_capability,
                           const std::string& on_stopped_parameters)
  {
    capabilities2_runner::event_opts event_options;

    event_options.on_started = on_started_capability;
    event_options.on_failure = on_failure_capability;
    event_options.on_success = on_success_capability;
    event_options.on_stopped = on_stopped_capability;

    event_options.on_started_param = on_started_parameters;
    event_options.on_failure_param = on_failure_parameters;
    event_options.on_success_param = on_success_parameters;
    event_options.on_stopped_param = on_stopped_parameters;

    int event_count = runner_cache_[capability]->attach_events(
        event_options, std::bind(&capabilities2_server::RunnerCache::trigger_runner, this, std::placeholders::_1,
                                 std::placeholders::_2));

    RCLCPP_INFO(node_logging_interface_ptr_->get_logger(),
                "Configured triggers for capability %s: \n\tStarted: %s \n\tFailure: %s \n\tSuccess: %s \n\tStopped: "
                "%s\n",
                capability.c_str(), on_started_capability.c_str(), on_failure_capability.c_str(),
                on_success_capability.c_str(), on_stopped_capability.c_str());
  }

  /**
   * @brief Remove a given runner
   *
   * @param capability capability to be removed
   */
  void remove_runner(const std::string& capability)
  {
    // find the runner in the cache
    if (!running(capability))
    {
      // not found so nothing to do
      throw capabilities2_runner::runner_exception("capability runner not found: " + capability);
      // return;
    }

    // safely stop the runner
    try
    {
      runner_cache_[capability]->stop();
    }
    catch (const capabilities2_runner::runner_exception& e)
    {
      // pass
      // RCLCPP_ERROR(node_logging_interface_ptr_->get_logger(), "%s", e.what());
    }

    // reset the runner pointer
    runner_cache_[capability].reset();

    // remove the runner from map
    runner_cache_.erase(capability);
  }

  /**
   * @brief Get a list of active runners in the cache
   *
   * @return const std::vector<std::string> of runners
   */
  const std::vector<std::string> get_running_capabilities()
  {
    std::vector<std::string> runners;
    for (const auto& runner : runner_cache_)
    {
      runners.push_back(runner.first);
    }
    return runners;
  }

  /**
   * @brief Get provider of capability
   *
   * @param capability capability of which the provider is requested
   * @return provider name
   */
  const std::string provider(const std::string& capability)
  {
    if (!running(capability))
      throw capabilities2_runner::runner_exception("capability runner not found: " + capability);

    return runner_cache_[capability]->get_provider();
  }

  /**
   * @brief get started_by of capability
   *
   * @param capability capability of which the started_by is requested
   * @return name of the started_by
   */
  const std::string started_by(const std::string& capability)
  {
    if (!running(capability))
      throw capabilities2_runner::runner_exception("capability runner not found: " + capability);

    return runner_cache_[capability]->get_started_by();
  }

  /**
   * @brief get pid of capability
   *
   * @param capability capability of which the pid is requested
   * @return value of pid
   */
  const std::string pid(const std::string& capability)
  {
    if (!running(capability))
      throw capabilities2_runner::runner_exception("capability runner not found: " + capability);

    return runner_cache_[capability]->get_pid();
  }

  /**
   * @brief Check if a capability has a runner
   *
   * @param capability
   * @return true
   * @return false
   */
  bool running(const std::string& capability)
  {
    return runner_cache_.find(capability) != runner_cache_.end();
  }

private:
  // map capability to running model
  // capability / provider specs -> runner
  std::map<std::string, std::shared_ptr<capabilities2_runner::RunnerBase>> runner_cache_;

  // runner plugin loader
  pluginlib::ClassLoader<capabilities2_runner::RunnerBase> runner_loader_;

  // logger
  rclcpp::node_interfaces::NodeLoggingInterface::SharedPtr node_logging_interface_ptr_;
};

}  // namespace capabilities2_server
