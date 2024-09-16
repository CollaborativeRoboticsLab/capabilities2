#pragma once

#include <string>
#include <iostream>
#include <sstream>
#include <map>
#include <any>

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <action_msgs/srv/cancel_goal.hpp>

#include <capabilities2_runner/runner_base.hpp>
#include <capabilities2_runner/action_client_manager.hpp>

namespace capabilities2_runner
{

/**
 * @brief action runner base class
 *
 * Create an action client to run an action based capability
 */
class MultiActionRunner : public RunnerBase
{
public:
	/**
	 * @brief Constructor which needs to be empty due to plugin semantics
	 */
	MultiActionRunner() : RunnerBase()
	{
	}

	/**
	 * @brief Initializer function for initializing the action runner in place of constructor due to plugin semantics
	 *
	 * @param node shared pointer to the capabilities node. Allows to use ros node related functionalities
	 * @param run_config runner configuration loaded from the yaml file
	 * @param on_started function pointer to trigger at the start of the action client in the runner
	 * @param on_terminated function pointer to trigger at the termination of the action client in the runner
	 */
	virtual void init_runner(rclcpp::Node::SharedPtr node, const runner_opts& run_config,
							std::function<void(const std::string&)> on_started = nullptr,
							std::function<void(const std::string&)> on_terminated = nullptr,
							std::function<void(const std::string&)> on_stopped = nullptr)
	{
		// initialize the runner base by storing node pointer and run config
		init_base(node, run_config, on_started, on_terminated, on_stopped);

		// initialize the action client manager used for manageing the actions
		actionClientManager_ = std::make_shared<ActionClientManager>(node, run_config, on_started, on_terminated, on_stopped);
	}

	/**
	 * @brief Initializer function for initializing the action runner in place of constructor due to plugin semantics
	 *
	 * @param action_name action name used in the yaml file, used to load specific configuration from the run_config
	 */
	template <typename ActionT>
	void init_action(const std::string& action_name)
	{
		actionClientManager_->init_action<ActionT>(action_name);
	}

	/**
	 * @brief Deinitializer function for stopping an the action 
	 *
	 * @param action_name action name used in the yaml file, used to load specific configuration from the run_config
	 */
	template <typename ActionT>
	void deinit_action(const std::string& action_name)
	{
		actionClientManager_->deinit_action<ActionT>(action_name);
	}

	/**
	 * @brief Trigger function for calling an the action 
	 *
	 * @param action_name action name used in the yaml file, used to load specific configuration from the run_config
	 */
	template <typename ActionT>
	void trigger_action(const std::string& action_name, typename ActionT::Goal goal_msg)
	{
		actionClientManager_->trigger_action<ActionT>(action_name, goal_msg);
	}

protected:

	/** Action Client Manager for handling multiple actions*/
	std::shared_ptr<ActionClientManager> actionClientManager_;
	
};

}  // namespace capabilities2_runner
