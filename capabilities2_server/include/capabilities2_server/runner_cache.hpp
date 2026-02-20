#pragma once

#include <map>
#include <string>
#include <vector>
#include <functional>
#include <yaml-cpp/yaml.h>
// #include <tinyxml2.h>
#include <rclcpp/rclcpp.hpp>
#include <pluginlib/class_loader.hpp>

#include <capabilities2_server/models/run_config.hpp>
#include <capabilities2_runner/runner_base.hpp>
#include <capabilities2_events/event_base.hpp>
#include <capabilities2_msgs/msg/capability_connection.hpp>

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
 * 2. <resource type> runner - e.g. action, service, topic, etc.
 *
 */
class RunnerCache
{
public:
  RunnerCache() : runner_loader_("capabilities2_runner", "capabilities2_runner::RunnerBase")
  {
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
    // if the runner exists then throw an error preserving uniqueness
    if (running(capability))
    {
      throw capabilities2_runner::runner_exception("capability is running already: " + capability);
    }

    // check if run config is valid
    if (!run_config.is_valid())
    {
      throw capabilities2_runner::runner_exception("run config is not valid: " + YAML::Dump(run_config.to_yaml()));
    }

    // create the runner, add the runner to map, and if the spec runner contains a path to a launch file then use the
    // launch file runner
    if (run_config.runner.find(".launch") != std::string::npos || run_config.runner.find("/") != std::string::npos ||
        run_config.runner.find(".py") != std::string::npos)
    {
      runner_cache_[capability] = runner_loader_.createSharedInstance("capabilities2_runner::LaunchRunner");
    }
    else
    {
      runner_cache_[capability] = runner_loader_.createSharedInstance(run_config.runner);
    }

    // start the runner
    runner_cache_[capability]->start(node, run_config.to_runner_opts(), "");
  }

  /**
   * @brief Trigger a runner in the cache
   *
   * xml parameters are used
   *
   * @param capability capability name to be loaded
   * @param parameters parameters related to the runner in std::string form for compatibility across various runners
   * @param bond_id unique identifier for the group on connections associated with this runner trigger
   * @param instance_id unique identifier for the instance of the capability
   */
  void trigger_runner(const std::string& bond_id, const std::string& capability, const std::string& instance_id,
                      capabilities2_events::EventParameters parameters)
  {
    // TODO: validate trigger id (DEPRECATED?)

    // is the runner in the cache
    if (running(capability))
    {
      runner_cache_[capability]->trigger(parameters, bond_id, instance_id);
    }
    else
    {
      throw capabilities2_runner::runner_exception("capability runner not found: " + capability);
    }
  }

  /**
   * @brief Add state change connection between runners via an event
   *
   * @param capability capability from where the event originates
   * @param connection_id unique id for the connection
   * @param connection connection options for the event
   * @param event_emitter event emitter to be used by this runner for emitting events on state changes to the target
   * capability
   */
  void add_connection(const std::string& capability, const std::string& connection_id,
                      const capabilities2_msgs::msg::CapabilityConnection& connection,
                      std::shared_ptr<capabilities2_events::EventBase> event_emitter = nullptr)
  {
    // find the runner in the cache and if not found then throw an error
    if (!running(capability))
    {
      throw capabilities2_runner::runner_exception("capability runner not found: " + capability);
    }

    // pass the event api to the runner
    // for emitting events on state changes
    runner_cache_[capability]->enable_events(event_emitter);

    try
    {
      // add connection to the runner
      // callback signature: (capability, parameters, bond_id, instance_id)
      runner_cache_[capability]->add_connection(connection_id, connection.type, connection.target,
                                                std::bind(&capabilities2_server::RunnerCache::trigger_runner, this,
                                                          std::placeholders::_1, std::placeholders::_2,
                                                          std::placeholders::_3, std::placeholders::_4));
    }
    catch (const capabilities2_events::event_exception& e)
    {
      throw capabilities2_runner::runner_exception(e.what());
    }
  }

  /**
   * @brief Remove a given runner
   *
   * @param capability capability to be removed
   * @param bond_id bond_id of the capability instance to be removed
   *
   * This will stop the runner and remove it from the cache. If the runner is not found then an error is thrown.
   */
  void remove_runner(const std::string& capability)
  {
    // find the runner in the cache and if not found then throw an error
    if (!running(capability))
    {
      throw capabilities2_runner::runner_exception("capability runner not found: " + capability);
    }

    // safely stop the runner
    try
    {
      runner_cache_[capability]->stop("");
    }
    catch (const capabilities2_runner::runner_exception& e)
    {
      // pass
      // RCLCPP_ERROR(node_logging_interface_ptr_->get_logger(), "%s", e.what());
    }

    // reset the runner pointer
    // runner_cache_[capability].reset();

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
};

}  // namespace capabilities2_server
