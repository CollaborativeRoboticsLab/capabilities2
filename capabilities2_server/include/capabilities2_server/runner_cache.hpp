#pragma once

#include <map>
#include <string>
#include <vector>

namespace capabilities2_server
{

/**
 * @brief runner cache class
 * responsible for caching the currently active runners of capabilities
 *
 */
class RunnerCache
{
public:
  RunnerCache()
  {
  }

  void add_runner(const std::string& runner, const std::string& capability)
  {
    // if the runner exists then throw an error
    if (runner_cache_.find(runner) != runner_cache_.end())
    {
      throw std::runtime_error("Runner already exists");
    }

    runner_cache_[runner] = capability;
  }

  void remove_runner(const std::string& runner)
  {
    runner_cache_.erase(runner);
  }

  const std::string get_capability(const std::string& runner)
  {
    return runner_cache_[runner];
  }

  const std::vector<std::string> get_runners(const std::string& capability)
  {
    std::vector<std::string> runners;
    for (auto& [runner, cap] : runner_cache_)
    {
      if (cap == capability)
      {
        runners.push_back(runner);
      }
    }
    return runners;
  }

private:
  // map of runner to capability
  std::map<std::string, std::string> runner_cache_;
};

}  // namespace capabilities2_server
