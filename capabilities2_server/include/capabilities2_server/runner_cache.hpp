#pragma once

#include <map>
#include <string>
#include <vector>
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
		on_started = nullptr;
		on_stopped = nullptr;
		on_terminated = nullptr;
	}

	/**
	 * @brief Add a runner to the cache
	 *
	 * @param node   pointer to the origin node, generally the capabilities2_server
	 * @param capability capability name to be loaded
	 * @param run_config run_config of the runner to be loaded
	 * @param parameters parameters related to the runner in std::string form for compatibility accross various runners
	 */
	void add_runner(rclcpp::Node::SharedPtr node, const std::string& capability,
					const models::run_config_model_t& run_config, 
					std::shared_ptr<tinyxml2::XMLElement> parameters = nullptr)
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
		if (run_config.runner.find(".launch") != std::string::npos || 
			run_config.runner.find("/") != std::string::npos ||
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
		runner_cache_[capability]->start(node, run_config.to_runner_opts(), on_started, on_terminated, on_stopped);
	}

	/**
	 * @brief Trigger a runner in the cache
	 *
	 * @param node   pointer to the origin node, generally the capabilities2_server
	 * @param capability capability name to be loaded
	 * @param parameters parameters related to the runner in std::string form for compatibility accross various runners
	 */
	void trigger_runner(const std::string& capability, std::shared_ptr<tinyxml2::XMLElement> parameters = nullptr)
	{
		runner_cache_[capability]->trigger(parameters);
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
	void set_on_started(std::function<void(const std::string&)> cb)
	{
		on_started = cb;
	}

	/**
	 * @brief Callback function for 'on_stopped' event
	 *
	 * @param cb callback function pointer
	 */
	void set_on_stopped(std::function<void(const std::string&)> cb)
	{
		on_stopped = cb;
	}

	/**
	 * @brief Callback function for 'on_terminated' event
	 *
	 * @param cb callback function pointer
	 */
	void set_on_terminated(std::function<void(const std::string&)> cb)
	{
		on_terminated = cb;
	}

private:
  // map capability to running model
  // capability / provider specs -> runner
  std::map<std::string, std::shared_ptr<capabilities2_runner::RunnerBase>> runner_cache_;

  // runner plugin loader
  pluginlib::ClassLoader<capabilities2_runner::RunnerBase> runner_loader_;

  // event callbacks
  std::function<void(const std::string&)> on_started;
  std::function<void(const std::string&)> on_stopped;
  std::function<void(const std::string&)> on_terminated;
};

}  // namespace capabilities2_server
