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
  {
  }

  /**
   * @brief Initializer function for initializing the action runner in place of constructor due to plugin semantics
   *
   * @param node shared pointer to the capabilities node. Allows to use ros node related functionalities
   * @param run_config runner configuration loaded from the yaml file
   * @param action_name action name used in the yaml file, used to load specific configuration from the run_config
   * @param on_started function pointer to trigger at the start of the action client in the runner
   * @param on_terminated function pointer to trigger at the termination of the action client in the runner
   * @param on_stopped function pointer to trigger at the termination of the action client by the server
   */
  virtual void init_action(rclcpp::Node::SharedPtr node, const runner_opts& run_config, const std::string& action_name,
                           std::function<void(const std::string&)> on_started = nullptr,
                           std::function<void(const std::string&)> on_terminated = nullptr,
                           std::function<void(const std::string&)> on_stopped = nullptr)
  {
    // initialize the runner base by storing node pointer and run config
    init_base(node, run_config, on_started, on_terminated, on_stopped);

    // create an action client
    action_client_ = rclcpp_action::create_client<ActionT>(node_, action_name);

    // wait for action server
    RCLCPP_INFO(node_->get_logger(), "%s waiting for action: %s", run_config_.interface.c_str(), action_name.c_str());

    if (!action_client_->wait_for_action_server(std::chrono::seconds(3)))
    {
      RCLCPP_ERROR(node_->get_logger(), "%s failed to connect to action: %s", run_config_.interface.c_str(),
                   action_name.c_str());
      throw runner_exception("failed to connect to action server");
    }

    // send goal options
    // goal response callback
    send_goal_options_.goal_response_callback =
        [this](const typename rclcpp_action::ClientGoalHandle<ActionT>::SharedPtr& goal_handle) {
          // publish event
          if (goal_handle)
            if (on_started_)
              on_started_(run_config_.interface);

          // store goal handle to be used with stop funtion
          goal_handle_ = goal_handle;
        };

    // result callback
    send_goal_options_.result_callback =
        [this](const typename rclcpp_action::ClientGoalHandle<ActionT>::WrappedResult& wrapped_result) {
          if (wrapped_result.code == rclcpp_action::ResultCode::SUCCEEDED)
          {
            // Do something
          }
          else
          {
            // send terminated event
            if (on_terminated_)
            {
              on_terminated_(run_config_.interface);
            }
          }
        };
  }

  /**
   * @brief stop function to cease functionality and shutdown
   *
   */
  virtual void stop() override
  {
    // if the node pointer is empty then throw an error
    // this means that the runner was not started and is being used out of order

    if (!node_)
      throw runner_exception("cannot stop runner that was not started");

    // throw an error if the action client is null
    // this can happen if the runner is not able to find the action resource

    if (!action_client_)
      throw runner_exception("cannot stop runner action that was not started");

    // stop runner using action client
    if (goal_handle_)
    {
      try
      {
        auto cancel_future = action_client_->async_cancel_goal(
            goal_handle_, [this](action_msgs::srv::CancelGoal_Response::SharedPtr response) {
              if (response->return_code != action_msgs::srv::CancelGoal_Response::ERROR_NONE)
              {
                // throw runner_exception("failed to cancel runner");
              }

              // publish event
              if (on_stopped_)
              {
                on_stopped_(run_config_.interface);
              }
            });

        // wait for action to be stopped. hold the thread for 2 seconds to help keep callbacks in scope
        rclcpp::spin_until_future_complete(node_, cancel_future, std::chrono::seconds(2));
      }
      catch (const rclcpp_action::exceptions::UnknownGoalHandleError& e)
      {
        RCLCPP_ERROR(node_->get_logger(), "failed to cancel goal: %s", e.what());
        throw runner_exception(e.what());
      }
    }
  }

  /**
   * @brief the trigger function on the action runner is used to trigger an action.
   * this method provides a mechanism for injecting parameters or a goal into the action
   * and then trigger the action
   *
   * @param parameters
   * @return std::optional<std::function<void(std::shared_ptr<tinyxml2::XMLElement>)>>
   */
  virtual std::optional<std::function<void(std::shared_ptr<tinyxml2::XMLElement>)>>
  trigger(std::shared_ptr<tinyxml2::XMLElement> parameters = nullptr) override
  {
    // the action is being triggered out of order
    if (!goal_handle_)
      throw runner_exception("cannot trigger action that was not started");

    // if parameters are not provided then cannot proceed
    if (!parameters)
      throw runner_exception("cannot trigger action without parameters");

    // generate a goal from parameters if provided
    typename ActionT::Goal goal_msg = generate_goal(parameters);

    // trigger the action client with goal
    auto goal_handle_future = action_client_->async_send_goal(goal_msg, send_goal_options_);

    // spin until send future completes
    if (rclcpp::spin_until_future_complete(node_, goal_handle_future) != rclcpp::FutureReturnCode::SUCCESS)
    {
      RCLCPP_ERROR(node_->get_logger(), "send goal call failed");
      return std::nullopt;
    }

    // get result future
    typename rclcpp_action::ClientGoalHandle<ActionT>::SharedPtr goal_handle;
    auto result_future = action_client_->async_get_result(goal_handle);

    // create a function to call for the result
    // the future will be returned to the caller
    // and the caller can provide a conversion function
    // to handle the result
    std::function<void(std::shared_ptr<tinyxml2::XMLElement>)> result_callback =
        [this, result_future](std::shared_ptr<tinyxml2::XMLElement> result) {
          // wait for result
          if (rclcpp::spin_until_future_complete(node_, result_future) != rclcpp::FutureReturnCode::SUCCESS)
          {
            RCLCPP_ERROR(node_->get_logger(), "get result call failed");
            return;
          }

          // get wrapped result
          typename rclcpp_action::ClientGoalHandle<ActionT>::WrappedResult wrapped_result = result_future.get();

          // convert the result
          if (wrapped_result.code == rclcpp_action::ResultCode::SUCCEEDED)
          {
            // publish event
            if (on_result_)
            {
              on_result_(run_config_.interface);
            }

            result = generate_result(wrapped_result.result);
            return;
          }
        };

    return result_callback;
  }

protected:
  /**
   * @brief Generate a goal from parameters
   *
   * This function is used in conjunction with the trigger function to inject type erased parameters
   * into the typed action
   *
   * A pattern needs to be implemented in the derived class
   *
   * @param parameters
   * @return ActionT::Goal the generated goal
   */
  virtual typename ActionT::Goal generate_goal(std::shared_ptr<tinyxml2::XMLElement> parameters) = 0;

  /**
   * @brief generate a typed erased goal result
   *
   * this method is used in a callback passed to the trigger caller to get type erased result
   * from the action the result can be passed by the caller or ignored
   *
   * The pattern needs to be implemented in the derived class
   *
   * @param wrapped_result
   * @return std::shared_ptr<tinyxml2::XMLElement>
   */
  virtual std::shared_ptr<tinyxml2::XMLElement> generate_result(const typename ActionT::Result::SharedPtr& result) = 0;

  /**< action client */
  typename rclcpp_action::Client<ActionT>::SharedPtr action_client_;

  /** Send Goal Option struct to link result_callback, feedback_callback and goal_response_callback with action client
   */
  typename rclcpp_action::Client<ActionT>::SendGoalOptions send_goal_options_;

  /** goal handle parameter to capture goal response from goal_response_callback */
  typename rclcpp_action::ClientGoalHandle<ActionT>::SharedPtr goal_handle_;
};

}  // namespace capabilities2_runner
