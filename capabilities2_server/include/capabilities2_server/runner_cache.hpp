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
    tinyxml2::XMLDocument doc;
    doc.Parse(parameters.c_str());
    tinyxml2::XMLElement* xml_parameters = doc.FirstChildElement();

    // is the runner in the cache
    if (running(capability))
    {
      runner_cache_[capability]->trigger(xml_parameters);
    }
    else
    {
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

    if (on_started_capability != "")
    {
      event_options.on_started = [this, &on_started_capability](tinyxml2::XMLElement* parameters) {
        runner_cache_[on_started_capability]->trigger(parameters);
      };

      tinyxml2::XMLDocument doc;
      doc.Parse(on_started_parameters.c_str());
      event_options.on_started_param = doc.FirstChildElement();
    }
    else
    {
      event_options.on_started = nullptr;
      event_options.on_started_param = nullptr;
    }

    if (on_failure_capability != "")
    {
      event_options.on_failure = [this, &on_failure_capability](tinyxml2::XMLElement* parameters) {
        runner_cache_[on_failure_capability]->trigger(parameters);
      };

      tinyxml2::XMLDocument doc;
      doc.Parse(on_failure_parameters.c_str());
      event_options.on_failure_param = doc.FirstChildElement();
    }
    else
    {
      event_options.on_failure = nullptr;
      event_options.on_failure_param = nullptr;
    }

    if (on_success_capability != "")
    {
      event_options.on_success = [this, &on_success_capability](tinyxml2::XMLElement* parameters) {
        runner_cache_[on_success_capability]->trigger(parameters);
      };

      tinyxml2::XMLDocument doc;
      doc.Parse(on_success_parameters.c_str());
      event_options.on_success_param = doc.FirstChildElement();
    }
    else
    {
      event_options.on_success = nullptr;
      event_options.on_success_param = nullptr;
    }

    if (on_stopped_capability != "")
    {
      event_options.on_stopped = [this, &on_stopped_capability](tinyxml2::XMLElement* parameters) {
        runner_cache_[on_stopped_capability]->trigger(parameters);
      };

      tinyxml2::XMLDocument doc;
      doc.Parse(on_stopped_parameters.c_str());
      event_options.on_stopped_param = doc.FirstChildElement();
    }
    else
    {
      event_options.on_stopped = nullptr;
      event_options.on_stopped_param = nullptr;
    }

    runner_cache_[capability]->attach_events(event_options);
    event_cache_[capability] = event_options;
  }

  /**
   * @brief Remove a given runner
   *
   * @param capability capability to be removed
   */
  void remove_runner(const std::string& capability)
  {
    // find the runner in the cache
    if (runner_cache_.find(capability) == runner_cache_.end())
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

  /**
   * @brief Callback function for 'on_started' event
   *
   * @param cb callback function pointer
   */
  // void set_on_started(std::function<void(const std::string&)> cb)
  // {
  //   on_started = cb;
  // }

  /**
   * @brief Callback function for 'on_stopped' event
   *
   * @param cb callback function pointer
   */
  // void set_on_stopped(std::function<void(const std::string&)> cb)
  // {
  //   on_stopped = cb;
  // }

  /**
   * @brief Callback function for 'on_failure' event
   *
   * @param cb callback function pointer
   */
  // void set_on_terminated(std::function<void(const std::string&)> cb)
  // {
  //   on_failure = cb;
  // }

private:
  // map capability to running model
  // capability / provider specs -> runner
  std::map<std::string, std::shared_ptr<capabilities2_runner::RunnerBase>> runner_cache_;

  // map events to capabilities
  std::map<std::string, capabilities2_runner::event_opts> event_cache_;

  // runner plugin loader
  pluginlib::ClassLoader<capabilities2_runner::RunnerBase> runner_loader_;

  // event callbacks
  // std::function<void(const std::string&)> on_started;
  // std::function<void(const std::string&)> on_stopped;
  // std::function<void(const std::string&)> on_failure;
  // std::function<void(const std::string&)> on_success;
};

}  // namespace capabilities2_server
