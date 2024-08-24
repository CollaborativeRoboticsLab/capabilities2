#pragma once

#include <map>
#include <string>
#include <vector>
#include <pluginlib/class_loader.hpp>
#include <capabilities2_server/models/run_config.hpp>
#include <capabilities2_runners_plugins/runner_base.hpp>

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
  RunnerCache() : runner_loader_("capabilities2_runners_plugins", "capabilities2_runner::RunnerBase")
  {
  }

  /**
   * @brief Add a runner to the cache
   *
   * @param capability
   * @param run_config
   */
  void add_runner(const std::string& capability, const models::run_config_model_t& run_config)
  {
    // if the runner exists then throw an error
    if (running(capability))
    {
      // already running
      throw std::runtime_error("capability is running already");
      // return;
    }

    // create the runner
    // add the runner to map
    // TODO: use different runner types based on cap and provider specs
    runner_cache_[capability] = runner_loader_.createSharedInstance("capabilities2_runner::LaunchRunner");

    // TODO: start the runner
    // runner_cache_[capability]->start(run_config.to_runner_opts());
  }

  void remove_runner(const std::string& capability)
  {
    // find the runner in the cache
    if (runner_cache_.find(capability) == runner_cache_.end())
    {
      // not found so nothing to do
      // throw std::runtime_error("capability runner not found");
      return;
    }

    // TODO: safely stop the runner
    std::shared_ptr<capabilities2_runner::RunnerBase> runner = runner_cache_[capability];
    // runner->stop();
    runner.reset();

    // remove the runner from map
    runner_cache_.erase(capability);
  }

  /**
   * @brief Get a list of active providers in the cache
   *
   * @param capability
   * @return const std::vector<std::string>
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

  // get provider of capability
  const std::string provider(const std::string& capability)
  {
    // return runner_cache_[capability]->get_provider();
    return "runner_cache_[capability]->get_provider()";
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
