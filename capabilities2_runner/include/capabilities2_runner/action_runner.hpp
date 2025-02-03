#pragma once

#include <string>
#include <chrono>
#include <iostream>
#include <sstream>
#include <vector>
#include <tinyxml2.h>

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
   */
  virtual void init_action(rclcpp::Node::SharedPtr node, const runner_opts& run_config, const std::string& action_name,
                           std::function<void(Event&)> runner_publish_func)
  {
    // initialize the runner base by storing node pointer and run config
    init_base(node, run_config, runner_publish_func);

    // create an action client
    action_client_ = rclcpp_action::create_client<ActionT>(node_, action_name);

    // wait for action server
    info_("waiting for action: " + action_name);

    if (!action_client_->wait_for_action_server(std::chrono::seconds(1000)))
    {
      error_("failed to connect to action: " + action_name);
      throw runner_exception("failed to connect to action server");
    }

    info_("connected with action: " + action_name);
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
                error_("Runner cancellation failed.");
              }

              // Trigger on_stopped event if defined
              if (events[execute_id].on_stopped != "")
              {
                info_("on_stopped", -1, events[execute_id].on_stopped, EventType::STOPPED);
                triggerFunction_(events[execute_id].on_stopped, update_on_stopped(events[execute_id].on_stopped_param));
              }
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
        error_("failed to cancel goal: " + std::string(e.what()));
        throw runner_exception(e.what());
      }
    }
  }

  /**
   * @brief Trigger process to be executed.
   *
   * This method utilizes paramters set via the trigger() function
   *
   * @param parameters pointer to tinyxml2::XMLElement that contains parameters
   */
  virtual void execution(int id) override
  {
    execute_id += 1;

    // if parameters are not provided then cannot proceed
    if (!parameters_[id])
      throw runner_exception("cannot trigger action without parameters");

    // generate a goal from parameters if provided
    goal_msg_ = generate_goal(parameters_[id], id);

    info_("goal generated", id);

    std::unique_lock<std::mutex> lock(send_goal_mutex);
    action_complete = false;

    // trigger the action client with goal
    send_goal_options_.goal_response_callback =
        [this, id](const typename rclcpp_action::ClientGoalHandle<ActionT>::SharedPtr& goal_handle) {
          if (goal_handle)
          {
            info_("goal accepted. Waiting for result", id);

            // trigger the events related to on_started state
            if (events[execute_id].on_started != "")
            {
              info_("on_started", id, events[execute_id].on_started, EventType::STARTED);
              triggerFunction_(events[execute_id].on_started, update_on_started(events[execute_id].on_started_param));
            }
          }
          else
          {
            error_("goal rejected", id);
          }

          // store goal handle to be used with stop funtion
          goal_handle_ = goal_handle;
        };

    send_goal_options_.feedback_callback =
        [this, id](typename rclcpp_action::ClientGoalHandle<ActionT>::SharedPtr goal_handle,
                   const typename ActionT::Feedback::ConstSharedPtr feedback_msg) {
          std::string feedback = generate_feedback(feedback_msg);

          if (feedback != "")
          {
            info_("received feedback: " + feedback, id);
          }
        };

    send_goal_options_.result_callback =
        [this, id](const typename rclcpp_action::ClientGoalHandle<ActionT>::WrappedResult& wrapped_result) {
          info_("received result", id);
          if (wrapped_result.code == rclcpp_action::ResultCode::SUCCEEDED)
          {
            info_("action succeeded.", id);

            // trigger the events related to on_success state
            if (events[execute_id].on_success != "")
            {
              info_("on_success", id, events[execute_id].on_success, EventType::SUCCEEDED);
              triggerFunction_(events[execute_id].on_success, update_on_success(events[execute_id].on_success_param));
            }
          }
          else
          {
            error_("action failed", id);

            // trigger the events related to on_failure state
            if (events[execute_id].on_failure != "")
            {
              info_("on_failure", id, events[execute_id].on_failure, EventType::FAILED);
              triggerFunction_(events[execute_id].on_failure, update_on_failure(events[execute_id].on_failure_param));
            }
          }

          result_ = wrapped_result.result;
          action_complete = true;
          send_goal_cv.notify_all();
        };

    goal_handle_future_ = action_client_->async_send_goal(goal_msg_, send_goal_options_);
    info_("goal sent. Waiting for acceptance.", id);

    // Conditional wait
    send_goal_cv.wait(lock, [this] { return action_complete; });
    info_("action complete. Thread closing.", id);
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
  virtual typename ActionT::Goal generate_goal(tinyxml2::XMLElement* parameters, int id) = 0;

  /**
   * @brief Generate a std::string from feedback message
   *
   * This function is used to convert feedback messages into generic strings
   *
   * A pattern needs to be implemented in the derived class. If the feedback string
   * is empty, nothing will be printed on the screen
   *
   * @param parameters
   * @return ActionT::Feedback the received feedback
   */
  virtual std::string generate_feedback(const typename ActionT::Feedback::ConstSharedPtr msg) = 0;

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

  // Synchronization tools
  std::mutex mutex_;
  std::condition_variable cv_;
  bool action_completed_;
};

}  // namespace capabilities2_runner
