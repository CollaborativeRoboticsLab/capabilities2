#pragma once

#include <string>
#include <iostream>
#include <sstream>
#include <vector>

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <action_msgs/srv/cancel_goal.hpp>

#include <capabilities2_runner/runner_base.hpp>

namespace capabilities2_runner
{

/**
 * @brief action runner base class
 *
 * Create an action client to run an action based capability
 */
template <typename ActionT>
class ActionRunner : public RunnerBase
{
public:

	/**
	 * @brief Constructor which needs to be empty due to plugin semantics
	 */
	ActionRunner() : RunnerBase()
	{}

	/**
	 * @brief Initializer function for initializing the action runner in place of constructor due to plugin semantics
	 * 
	 * @param node shared pointer to the capabilities node. Allows to use ros node related functionalities
   	 * @param run_config runner configuration loaded from the yaml file
	 * @param action_name action name used in the yaml file, used to load specific configuration from the run_config
	 * @param on_started function pointer to trigger at the start of the action client in the runner
	 * @param on_terminated function pointer to trigger at the termination of the action client in the runner
	 */
	virtual void init_action(rclcpp::Node::SharedPtr node, const runner_opts& run_config, const std::string& action_name, 
					 std::function<void(const std::string&)> on_started = nullptr,
                     std::function<void(const std::string&)> on_terminated = nullptr)
	{
		// initialize the runner base by storing node pointer and run config
		init_base(node, run_config);

		// initialize the used for message timestamps
		clock_ = node_->get_clock();

		// create an action client
		action_client_ = rclcpp_action::create_client<ActionT>(node_, action_name);

		// wait for action server
		RCLCPP_INFO(node_->get_logger(), "%s waiting for action: %s", run_config_.interface.c_str(), action_name.c_str());

		if (!action_client_->wait_for_action_server(std::chrono::seconds(3)))
		{
			RCLCPP_ERROR(node_->get_logger(), "%s failed to connect to action server", run_config_.interface.c_str());
			throw runner_exception("failed to connect to action server");
		}

		// goal response callback
		send_goal_options_.goal_response_callback = 
		[this, on_started, on_terminated](const typename rclcpp_action::ClientGoalHandle<ActionT>::SharedPtr& goal_handle) 
		{
			// store goal handle to be used with stop funtion
			goal_handle_ = goal_handle;
		};
        
		// result callback
		send_goal_options_.result_callback =
		[this, on_started, on_terminated](const typename rclcpp_action::ClientGoalHandle<ActionT>::WrappedResult& wrapped_result) 
		{
			if (wrapped_result.code == rclcpp_action::ResultCode::SUCCEEDED)
			{
				// publish event
				if (on_started)
				{
					on_started(run_config_.interface);
				}
			} else
			{
				// send terminated event
				if (on_terminated)
				{
					on_terminated(run_config_.interface);
				}
			}
		};
	}

	/**
	 * @brief stop function to cease functionality and shutdown
	 * 
	 * @param on_stopped function pointer to trigger at the termination of the action client by the server
	 */
	virtual void stop(std::function<void(const std::string&)> on_stopped = nullptr) override
	{
		// if the node pointer is empty then throw an error
		// this means that the runner was not started and is being used out of order

		if (!node_)  throw runner_exception("cannot stop runner that was not started");

		// throw an error if the action client is null
		// this can happen if the runner is not able to find the action resource

		if (!action_client_) throw runner_exception("cannot stop runner action that was not started");

		// stop runner using action client
		if (goal_handle_)
		{
			try
			{
				auto cancel_future = action_client_->async_cancel_goal(
					goal_handle_, 
					[this, on_stopped](action_msgs::srv::CancelGoal_Response::SharedPtr response) 
						{
							if (response->return_code != action_msgs::srv::CancelGoal_Response::ERROR_NONE)
							{
								// throw runner_exception("failed to cancel runner");
							}

							// publish event
							if (on_stopped)
							{
								on_stopped(run_config_.interface);
							}
						}
				);

				// wait for action to be stopped. hold the thread for 2 seconds to help keep callbacks in scope
				rclcpp::spin_until_future_complete(node_, cancel_future, std::chrono::seconds(2));
			}
			catch (const rclcpp_action::exceptions::UnknownGoalHandleError& e)
			{
				throw runner_exception(e.what());
			}
		}
	}

protected:

	/**< action client */
	std::shared_ptr<rclcpp_action::Client<ActionT>> action_client_;

	/**< rclcpp clock to get time */
	rclcpp::Clock::SharedPtr clock_;

	/**< Send Goal Option struct to link result_callback, feedback_callback and goal_response_callback with action client */
	typename rclcpp_action::Client<ActionT>::SendGoalOptions send_goal_options_;

	/**< goal handle parameter to capture goal response from goal_response_callback */
	typename rclcpp_action::ClientGoalHandle<ActionT>::SharedPtr goal_handle_;
};

}  // namespace capabilities2_runner
