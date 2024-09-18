#pragma once

#include <string>
#include <iostream>
#include <sstream>
#include <map>
#include <any>

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <action_msgs/srv/cancel_goal.hpp>

namespace capabilities2_runner
{

template <typename ActionF>
struct ActionClientBundle
{
  std::shared_ptr<rclcpp_action::Client<ActionF>> action_client;
  typename rclcpp_action::Client<ActionF>::SendGoalOptions send_goal_options;
  typename rclcpp_action::ClientGoalHandle<ActionF>::SharedPtr goal_handle;
};

/**
 * @brief action manager that control multiple action clients
 */
class ActionClientManager
{
public:
  /**
   * @brief Constructor of the action client manager
   *
   * @param node shared pointer to the capabilities node. Allows to use ros node related functionalities
   * @param run_config runner configuration loaded from the yaml file
   * @param on_started function pointer to trigger at the start of the action client in the runner
   * @param on_terminated function pointer to trigger at the termination of the action client in the runner
   * @param on_stopped function pointer to trigger at the termination of the action client by the server
   */
  ActionClientManager(rclcpp::Node::SharedPtr node, const runner_opts& run_config,
                      std::function<void(const std::string&)> on_started = nullptr,
                      std::function<void(const std::string&)> on_terminated = nullptr,
                      std::function<void(const std::string&)> on_stopped = nullptr)
  {
    node_ = node;
    run_config_ = run_config;
    on_started_ = on_started;
    on_terminated_ = on_terminated;
    on_stopped_ = on_stopped;
  }

  /**
   * @brief Initializer function for initializing the action runner in place of constructor due to plugin semantics
   *
   * @param action_name action name used in the yaml file, used to load specific configuration from the run_config
   */
  template <typename ActionT>
  void init_action(const std::string& action_name)
  {
    typename rclcpp_action::Client<ActionT>::SendGoalOptions goal_options_;
    typename rclcpp_action::ClientGoalHandle<ActionT>::SharedPtr goal_handle_;

    auto client_ = rclcpp_action::create_client<ActionT>(node_, action_name);

    // wait for action server
    RCLCPP_INFO(node_->get_logger(), "%s waiting for action: %s", run_config_.interface.c_str(), action_name.c_str());

    if (!client_->wait_for_action_server(std::chrono::seconds(3)))
    {
      RCLCPP_ERROR(node_->get_logger(), "%s failed to connect to action: %s", run_config_.interface.c_str(),
                   action_name.c_str());
      throw runner_exception("failed to connect to action server");
    }

    // send goal options
    // goal response callback
    goal_options_.goal_response_callback =
        [this, &goal_handle_](const typename rclcpp_action::ClientGoalHandle<ActionT>::SharedPtr& goal_handle) {
          // store goal handle to be used with stop funtion
          goal_handle_ = goal_handle;
        };

    // result callback
    goal_options_.result_callback =
        [this](const typename rclcpp_action::ClientGoalHandle<ActionT>::WrappedResult& wrapped_result) {
          if (wrapped_result.code == rclcpp_action::ResultCode::SUCCEEDED)
          {
            // publish event
            if (on_started_)
              on_started_(run_config_.interface);
          }
          else
          {
            // send terminated event
            if (on_terminated_)
              on_terminated_(run_config_.interface);
          }
        };

    ActionClientBundle<ActionT> bundle{ client_, goal_options_, goal_handle_ };

    action_clients_map_[action_name] = std::make_any<ActionClientBundle<ActionT>>(bundle);
  }

  /**
   * @brief Deinitializer function for stopping an the action
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
  }

  /**
   * @brief Trigger function for calling an the action
   *
   * @param action_name action name used in the yaml file, used to load specific configuration from the run_config
   */
  template <typename ActionT>
  void trigger_action(const std::string& action_name, typename ActionT::Goal goal_msg)
  {
    auto bundle = std::any_cast<ActionClientBundle<ActionT>>(action_clients_map_[action_name]);

    // launch runner using action client
    bundle.action_client->async_send_goal(goal_msg, bundle.send_goal_options);
  }

protected:
  /**
   * Dictionary to hold action client bundle. The key is a string, and the value is a polymorphic bundle.
   * */
  std::map<std::string, std::any> action_clients_map_;

  /**
   * shared pointer to the capabilities node. Allows to use ros node related functionalities
   */
  rclcpp::Node::SharedPtr node_;

  /**
   * runner configuration
   */
  runner_opts run_config_;

  /**
   * pointer to function to execute on starting the runner
   */
  std::function<void(const std::string&)> on_started_;

  /**
   * pointer to function to execute on terminating the runner
   */
  std::function<void(const std::string&)> on_terminated_;

  /**
   * pointer to function to execute on stopping the runner
   */
  std::function<void(const std::string&)> on_stopped_;
};

}  // namespace capabilities2_runner
