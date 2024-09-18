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
   * @param on_started function pointer to trigger at the start of the action client in the runner
   * @param on_terminated function pointer to trigger at the termination of the action client in the runner
   * @param on_stopped function pointer to trigger at the termination of the action client by the server
   */
  virtual void init_runner(rclcpp::Node::SharedPtr node, const runner_opts& run_config,
                           std::function<void(const std::string&)> on_started = nullptr,
                           std::function<void(const std::string&)> on_terminated = nullptr,
                           std::function<void(const std::string&)> on_stopped = nullptr)
  {
    // initialize the runner base by storing node pointer and run config
    init_base(node, run_config, on_started, on_terminated, on_stopped);
  }

  /**
   * @brief Initializer function for initializing the action runner in place of constructor due to plugin semantics
   *
   * @param action_name action name used in the yaml file, used to load specific configuration from the run_config
   */
  virtual void init_action(const std::string& action_name)
  {
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

    // // send goal options
    // // goal response callback
    // send_goal_options_.goal_response_callback =
    //     [this](const typename rclcpp_action::ClientGoalHandle<ActionT>::SharedPtr& goal_handle) {
    //       // publish event
    //       if (goal_handle)
    //         if (on_started_)
    //           on_started_(run_config_.interface);

    //       // store goal handle to be used with stop funtion
    //       goal_handle_ = goal_handle;
    //     };

    // // result callback
    // send_goal_options_.result_callback =
    //     [this](const typename rclcpp_action::ClientGoalHandle<ActionT>::WrappedResult& wrapped_result) {
    //       if (wrapped_result.code == rclcpp_action::ResultCode::SUCCEEDED)
    //       {
    //         // Do something
    //       }
    //       else
    //       {
    //         // send terminated event
    //         if (on_terminated_)
    //         {
    //           on_terminated_(run_config_.interface);
    //         }
    //       }
    //     };
  }

  /**
   * @brief deinitializer function to cease functionality and shutdown
   *
   */
  virtual void deinit_action()
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
   * @brief Trigger function for calling and triggering an action. Non blocking implementation so result will not
   * be returned.
   *
   * @param goal_msg goal message to be sent to the action server
   *
   * @returns True for success of launching an action. False for failure to launching the action.
   */
  bool trigger_action(typename ActionT::Goal& goal_msg)
  {
    // result callback
    send_goal_options_.result_callback =
        [this](const typename rclcpp_action::ClientGoalHandle<ActionT>::WrappedResult& wrapped_result) {
          if (wrapped_result.code != rclcpp_action::ResultCode::SUCCEEDED)
          {
            // send terminated event
            if (on_terminated_)
            {
              on_terminated_(run_config_.interface);
            }
          }
        };

    auto goal_handle_future = action_client_->async_send_goal(goal_msg, send_goal_options_);

    if (rclcpp::spin_until_future_complete(node_, goal_handle_future) != rclcpp::FutureReturnCode::SUCCESS)
    {
      RCLCPP_ERROR(node_->get_logger(), "send goal call failed :(");
      return false;
    }

    typename rclcpp_action::ClientGoalHandle<ActionT>::SharedPtr goal_handle = goal_handle_future.get();

    if (!goal_handle)
    {
      RCLCPP_ERROR(node_->get_logger(), "Goal was rejected by server");
      return false;
    }
    else
    {
      // publish event
      if (on_started_)
        on_started_(run_config_.interface);

      // store goal handle to be used with stop funtion

      goal_handle_ = goal_handle;
      return true;
    }
  }

  /**
   * @brief Trigger function for calling and triggering an action. Blocking implementation so result will be returned.
   *
   * @param goal_msg goal message to be sent to the action server
   * @param result_msg result message returned by the action server upon completion
   *
   * @returns True for success of completing an action. False for failure to complete the action.
   */
  bool trigger_action_wait(typename ActionT::Goal& goal_msg, typename ActionT::Result::SharedPtr result_msg)
  {
    auto goal_handle_future = action_client_->async_send_goal(goal_msg, send_goal_options_);

    if (rclcpp::spin_until_future_complete(node_, goal_handle_future) != rclcpp::FutureReturnCode::SUCCESS)
    {
      RCLCPP_ERROR(node_->get_logger(), "send goal call failed :(");
      return false;
    }

    typename rclcpp_action::ClientGoalHandle<ActionT>::SharedPtr goal_handle = goal_handle_future.get();

    if (!goal_handle)
    {
      RCLCPP_ERROR(node_->get_logger(), "Goal was rejected by server");
      return false;
    }
    else
    {
      // publish event
      if (on_started_)
        on_started_(run_config_.interface);

      // store goal handle to be used with stop funtion
      goal_handle_ = goal_handle;
    }

    // Wait for the server to be done with the goal
    auto result_future = action_client_->async_get_result(goal_handle);

    RCLCPP_INFO(node_->get_logger(), "Waiting for result");

    if (rclcpp::spin_until_future_complete(node_, result_future) != rclcpp::FutureReturnCode::SUCCESS)
    {
      RCLCPP_ERROR(node_->get_logger(), "get result call failed :(");
      return false;
    }

    typename rclcpp_action::ClientGoalHandle<ActionT>::WrappedResult wrapped_result = result_future.get();

    if (wrapped_result.code == rclcpp_action::ResultCode::SUCCEEDED)
    {
      result_msg = wrapped_result.result;
      return true;
    }
    else
    {
      // send terminated event
      if (on_terminated_)
        on_terminated_(run_config_.interface);
      return false;
    }
  }

protected:
  /**
   * @brief Get the name of an action resource by given action type
   *
   * This helps to navigate remappings from the runner config
   *
   * @param action_type
   * @return std::string
   */
  std::string get_action_name_by_type(const std::string& action_type)
  {
    for (const auto& resource : run_config_.resources)
    {
      if (resource.resource_type == "action")
      {
        if (resource.msg_type == action_type)
        {
          return resource.name;
        }
      }
    }

    throw runner_exception("no action resource found: " + action_type);
  }

  /**
   * @brief get first action resource name
   *
   * This can be used to get the name of the first action resource in the runner config
   *
   * @return std::string
   */
  std::string get_first_action_name()
  {
    for (const auto& resource : run_config_.resources)
    {
      if (resource.resource_type == "action")
      {
        return resource.name;
      }
    }

    throw runner_exception("no action resources found for interface: " + run_config_.interface);
  }

  /**< action client */
  std::shared_ptr<rclcpp_action::Client<ActionT>> action_client_;

  /**< Send Goal Option struct to link result_callback, feedback_callback and goal_response_callback with action client
   */
  typename rclcpp_action::Client<ActionT>::SendGoalOptions send_goal_options_;

  /**< goal handle parameter to capture goal response from goal_response_callback */
  typename rclcpp_action::ClientGoalHandle<ActionT>::SharedPtr goal_handle_;
};

}  // namespace capabilities2_runner
