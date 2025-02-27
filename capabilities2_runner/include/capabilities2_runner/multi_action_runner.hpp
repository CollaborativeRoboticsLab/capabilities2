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

namespace capabilities2_runner
{

/**
 * @brief templated struct to handle Action clients and their respective goal_handles
 *
 * @tparam ActionT action type
 */
template <typename ActionT>
struct ActionClientBundle
{
  std::shared_ptr<rclcpp_action::Client<ActionT>> action_client;
  typename rclcpp_action::ClientGoalHandle<ActionT>::SharedPtr goal_handle;
  typename rclcpp_action::Client<ActionT>::SendGoalOptions send_goal_options;
};

/**
 * @brief Multi action runner base class
 *
 * Create action clients to run multiple action based capabilities
 * this class provides helpers to set up sequential and concurrent actions
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
   * @brief create action clients
   *
   * Create a new action client and store it in the map
   *
   * @param action_name action name used in the yaml file, used to load specific configuration from the run_config
   */
  template <typename ActionT>
  void init_action(const std::string& action_name)
  {
    typename rclcpp_action::ClientGoalHandle<ActionT>::SharedPtr goal_handle_;
    typename rclcpp_action::Client<ActionT>::SendGoalOptions send_goal_options_;

    auto client_ = rclcpp_action::create_client<ActionT>(node_, action_name);

    // wait for action server
    RCLCPP_INFO(node_->get_logger(), "%s waiting for action: %s", run_config_.interface.c_str(), action_name.c_str());

    if (!client_->wait_for_action_server(std::chrono::seconds(3)))
    {
      RCLCPP_ERROR(node_->get_logger(), "%s failed to connect to action: %s", run_config_.interface.c_str(),
                   action_name.c_str());
      throw runner_exception("failed to connect to action server");
    }

    ActionClientBundle<ActionT> bundle{ client_, goal_handle_, send_goal_options_ };

    action_clients_map_[action_name] = std::make_any<ActionClientBundle<ActionT>>(bundle);
  }

  /**
   * @brief Deinitializer function for stopping an the action
   *
   * Stop the named action and delete the action client
   *
   * @param action_name action name used in the yaml file, used to load specific configuration from the run_config
   */
  template <typename ActionT>
  void deinit_action(const std::string& action_name)
  {
    auto bundle = std::any_cast<ActionClientBundle<ActionT>>(action_clients_map_[action_name]);

    // if the node pointer is empty then throw an error
    // this means that the runner was not started and is being used out of order

    if (!node_)
      throw runner_exception("cannot stop runner that was not started");

    // throw an error if the action client is null
    // this can happen if the runner is not able to find the action resource
    if (!bundle.action_client)
      throw runner_exception("cannot stop runner action that was not started");

    // stop runner using action client
    if (bundle.goal_handle)
    {
      try
      {
        auto cancel_future = bundle.action_client->async_cancel_goal(
            bundle.goal_handle, [this](action_msgs::srv::CancelGoal_Response::SharedPtr response) {
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

    // delete action client
    bundle.action_client.reset();
    action_clients_map_.erase(action_name);
  }

  /**
   * @brief Trigger function for calling and triggering an action. Non blocking implementation so result will not
   * be returned.  Use when action triggering is required and result message of the action is not required.
   *
   * @param action_name action name used in the yaml file, used to load specific configuration from the run_config
   * @param goal_msg goal message to be sent to the action server
   *
   * @returns True for success of launching an action. False for failure to launching the action.
   */
  template <typename ActionT>
  bool trigger_action(const std::string& action_name, typename ActionT::Goal& goal_msg)
  {
    auto bundle = std::any_cast<ActionClientBundle<ActionT>>(action_clients_map_[action_name]);

    // result callback
    bundle.send_goal_options.result_callback =
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

    auto goal_handle_future = bundle.action_client->async_send_goal(goal_msg, bundle.send_goal_options);

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
      bundle.goal_handle = goal_handle;
      return true;
    }

    action_clients_map_[action_name] = std::make_any<ActionClientBundle<ActionT>>(bundle);
  }

  /**
   * @brief Trigger function for calling and triggering an action. Blocking implementation so result will be returned.
   * Use when result message of the action is required.
   *
   * @param action_name action name used in the yaml file, used to load specific configuration from the run_config
   * @param goal_msg goal message to be sent to the action server
   * @param result_msg result message returned by the action server upon completion
   *
   * @returns True for success of completing an action. False for failure to complete the action.
   */
  template <typename ActionT>
  bool trigger_action_wait(const std::string& action_name, typename ActionT::Goal& goal_msg,
                           typename ActionT::Result::SharedPtr result_msg)
  {
    auto bundle = std::any_cast<ActionClientBundle<ActionT>>(action_clients_map_[action_name]);

    auto goal_handle_future = bundle.action_client->async_send_goal(goal_msg, bundle.send_goal_options);

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
      bundle.goal_handle = goal_handle;
    }

    // Wait for the server to be done with the goal
    auto result_future = bundle.action_client->async_get_result(goal_handle);

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
   * Dictionary to hold action client bundle. The key is a string, and the value is a
   * polymorphic bundle.
   * */
  std::map<std::string, std::any> action_clients_map_;
};

}  // namespace capabilities2_runner
