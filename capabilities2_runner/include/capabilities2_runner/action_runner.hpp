#pragma once

#include <string>
#include <iostream>
#include <sstream>
#include <vector>
#include <map>
#include <memory>
#include <mutex>
#include <condition_variable>

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <action_msgs/srv/cancel_goal.hpp>

#include <capabilities2_runner/threadtrigger_runner.hpp>

namespace capabilities2_runner
{

/**
 * @brief action runner base class
 *
 * Create an action client to run an action based capability
 */
template <typename ActionT>
class ActionRunner : public ThreadTriggerRunner
{
public:
  /**
   * @brief Constructor which needs to be empty due to plugin semantics
   */
  ActionRunner() : ThreadTriggerRunner()
  {
  }

  /**
   * @brief Initializer function for initializing the action runner in place of constructor due to plugin semantics
   *
   * @param node shared pointer to the capabilities node. Allows to use ros node related functionalities
   * @param run_config runner configuration loaded from the yaml file
   * @param action_name action name used in the yaml file, used to load specific configuration from the run_config
   */
  virtual void init_action(rclcpp::Node::SharedPtr node, const runner_opts& run_config, const std::string& action_name)
  {
    // initialize the runner base by storing node pointer and run config
    init_base(node, run_config);

    // create an action client
    action_client_ = rclcpp_action::create_client<ActionT>(node_, action_name);

    // wait for action server
    RCLCPP_INFO(node_->get_logger(), "waiting for action: %s", action_name.c_str());

    if (!action_client_->wait_for_action_server(std::chrono::seconds(1000)))
    {
      RCLCPP_ERROR(node_->get_logger(), "failed to connect to action: %s", action_name.c_str());
      throw runner_exception("failed to connect to action server");
    }

    RCLCPP_INFO(node_->get_logger(), "connected with action: %s", action_name.c_str());
  }

  /**
   * @brief stop function to cease functionality and shutdown
   *
   */
  virtual void stop(const std::string& bond_id, const std::string& instance_id = "") override
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
            goal_handle_, [this, &bond_id, &instance_id](action_msgs::srv::CancelGoal_Response::SharedPtr response) {
              if (response->return_code != action_msgs::srv::CancelGoal_Response::ERROR_NONE)
              {
                // throw runner_exception("failed to cancel runner");
                RCLCPP_ERROR(node_->get_logger(), "Runner cancellation failed.");
              }

              // emit stopped event
              emit_stopped(bond_id, instance_id, param_on_stopped());
            });

        // wait for action to be stopped. hold the thread for 2 seconds to help keep callbacks in scope
        // BUG: the line below does not work in jazzy build, so a workaround is used
        auto timeout = std::chrono::steady_clock::now() + std::chrono::seconds(2);
        while (std::chrono::steady_clock::now() < timeout)
        {
          // Check if the cancel operation is complete
          if (cancel_future.wait_for(std::chrono::milliseconds(100)) == std::future_status::ready)
            break;
        }
      }
      catch (const rclcpp_action::exceptions::UnknownGoalHandleError& e)
      {
        RCLCPP_ERROR(node_->get_logger(), "failed to cancel goal: %s", e.what());
        throw runner_exception(e.what());
      }
    }

    RCLCPP_INFO(node_->get_logger(), "runner cleaned. stopping..");
  }

protected:
  /**
   * @brief Trigger process to be executed.
   *
   * This method utilizes paramters set via the trigger() function
   *
   * @param parameters pointer to capabilities2_events::EventParameters that contains parameters
   */
  virtual void execution(capabilities2_events::EventParameters parameters, const std::string& thread_id) override
  {
    // split thread_id to get bond_id and instance_id (format: "bond_id/instance_id")
    std::string bond_id = ThreadTriggerRunner::bond_from_thread_id(thread_id);
    std::string instance_id = ThreadTriggerRunner::instance_from_thread_id(thread_id);

    // generate a goal from parameters provided
    goal_msg_ = generate_goal(parameters);
    RCLCPP_INFO(node_->get_logger(), "goal generated for instance %s", instance_id.c_str());

    std::mutex block_mutex;
    std::condition_variable cv;
    bool completed = false;
    std::unique_lock<std::mutex> lock(block_mutex);

    // trigger the action client with goal
    send_goal_options_.goal_response_callback =
        [this, &instance_id](const typename rclcpp_action::ClientGoalHandle<ActionT>::SharedPtr& goal_handle) {
          if (goal_handle)
          {
            RCLCPP_INFO(node_->get_logger(), "goal accepted. Waiting for result for instance %s", instance_id.c_str());
          }
          else
          {
            RCLCPP_ERROR(node_->get_logger(), "goal rejected for instance %s", instance_id.c_str());
          }

          // store goal handle to be used with stop funtion
          goal_handle_ = goal_handle;
        };

    send_goal_options_.feedback_callback = [this, &instance_id](
                                               typename rclcpp_action::ClientGoalHandle<ActionT>::SharedPtr goal_handle,
                                               const typename ActionT::Feedback::ConstSharedPtr feedback_msg) {
      std::string feedback = generate_feedback(feedback_msg);

      if (feedback != "")
      {
        RCLCPP_INFO(node_->get_logger(), "received feedback:  %s for instance %s", feedback.c_str(), instance_id.c_str());
      }
    };

    send_goal_options_.result_callback =
        [this, &instance_id, &completed, &cv,
         &bond_id, &instance_id](const typename rclcpp_action::ClientGoalHandle<ActionT>::WrappedResult& wrapped_result) {
          RCLCPP_INFO(node_->get_logger(), "received result for instance %s", instance_id.c_str());
          if (wrapped_result.code == rclcpp_action::ResultCode::SUCCEEDED)
          {
            RCLCPP_INFO(node_->get_logger(), "action succeeded for instance %s", instance_id.c_str());
            // emit success event
            emit_succeeded(bond_id, instance_id, param_on_success());
          }
          else
          {
            RCLCPP_ERROR(node_->get_logger(), "action failed for instance %s", instance_id.c_str());

            // emit failed event
            emit_failed(bond_id, instance_id, param_on_failure());
          }

          result_ = wrapped_result.result;
          completed = true;
          cv.notify_all();
        };

    goal_handle_future_ = action_client_->async_send_goal(goal_msg_, send_goal_options_);
    RCLCPP_INFO(node_->get_logger(), "goal sent. Waiting for acceptance for instance %s", instance_id.c_str());

    // Conditional wait
    cv.wait(lock, [&completed] { return completed; });
    RCLCPP_INFO(node_->get_logger(), "action complete. Thread closing for instance %s", instance_id.c_str());
  }

  /**
   * @brief Generate a goal from parameters
   *
   * This function is used in conjunction with the trigger function to inject type erased parameters
   * into the typed action
   *
   * A pattern needs to be implemented in the derived class
   *
   * @param parameters capability options that contain parameters for the instance
   * @return ActionT::Goal the generated goal
   */
  virtual typename ActionT::Goal generate_goal(capabilities2_events::EventParameters& parameters) = 0;

  /**
   * @brief Generate a std::string from feedback message
   *
   * This function is used to convert feedback messages into generic strings
   *
   * A pattern needs to be implemented in the derived class. If the feedback string
   * is empty, nothing will be printed on the screen
   *
   * @param msg the feedback message received from the action server
   * @return ActionT::Feedback the received feedback
   */
  virtual std::string generate_feedback(const typename ActionT::Feedback::ConstSharedPtr msg) = 0;

protected:
  /**< action client */
  typename rclcpp_action::Client<ActionT>::SharedPtr action_client_;

  /** Send Goal Option struct to link result_callback, feedback_callback and goal_response_callback with action client
   */
  typename rclcpp_action::Client<ActionT>::SendGoalOptions send_goal_options_;

  /** goal handle parameter to capture goal response from goal_response_callback */
  typename rclcpp_action::ClientGoalHandle<ActionT>::SharedPtr goal_handle_;

  /** Wrapped Result */
  typename rclcpp_action::ClientGoalHandle<ActionT>::WrappedResult wrapped_result_;

  /** Result */
  typename ActionT::Result::SharedPtr result_;

  /** Goal message */
  typename ActionT::Goal goal_msg_;

  /** Goal Handle Future message */
  std::shared_future<typename rclcpp_action::ClientGoalHandle<ActionT>::SharedPtr> goal_handle_future_;

  /** Result Future*/
  std::shared_future<typename rclcpp_action::ClientGoalHandle<ActionT>::WrappedResult> result_future_;
};

}  // namespace capabilities2_runner
