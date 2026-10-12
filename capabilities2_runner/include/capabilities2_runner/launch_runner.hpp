#pragma once

#include <chrono>
#include <cctype>
#include <future>
#include <mutex>
#include <string>
#include <thread>

#include <action_msgs/srv/cancel_goal.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <rclcpp/executors/single_threaded_executor.hpp>

#include <capabilities2_msgs/action/launch.hpp>
#include <capabilities2_runner/notrigger_runner.hpp>

namespace capabilities2_runner
{

/**
 * @brief launch runner base class
 *
 * Create a launch file runner to run a launch file based capability
 * uses process execution to run the launch file and kill the process on stop
 */
class LaunchRunner : public NoTriggerRunner
{
public:
  using LaunchAction = capabilities2_msgs::action::Launch;
  using GoalHandleLaunch = rclcpp_action::ClientGoalHandle<LaunchAction>;

  /**
   * @brief Constructor which needs to be empty due to plugin semantics
   */
  LaunchRunner() : NoTriggerRunner()
  {
  }

  /**
   * @brief Starter function for starting the launch runner
   *
   * @param node shared pointer to the capabilities node. Allows to use ros node related functionalities
   * @param run_config runner configuration loaded from the yaml file
   * @param bond_id bond identifier for the runner
   */
  virtual void start(rclcpp::Node::SharedPtr node, const runner_opts& run_config, const std::string& bond_id) override
  {
    init_base(node, run_config);

    {
      std::lock_guard<std::mutex> lock(state_mutex_);
      launch_reference_ = run_config_.runner;
      stop_requested_ = false;
      stop_event_emitted_ = false;
      failed_event_emitted_ = false;
      goal_handle_.reset();
      launch_id_.clear();
      resolved_launch_file_path_.clear();
      final_state_.clear();
      result_message_.clear();
    }

    const std::string action_name = resolve_launch_action_name();
    ensure_client_node();
    action_client_ = rclcpp_action::create_client<LaunchAction>(client_node_, action_name);

    RCLCPP_INFO(node_->get_logger(), "waiting for launch action: %s", action_name.c_str());
    if (!action_client_->wait_for_action_server(default_timeout_))
    {
      throw runner_exception("failed to connect to launch action server: " + action_name);
    }

    LaunchAction::Goal goal_msg;
    goal_msg.reference_name = run_config_.provider.empty() ? run_config_.interface : run_config_.provider;
    goal_msg.launch_file_path = launch_reference_;

    send_goal_options_ = typename rclcpp_action::Client<LaunchAction>::SendGoalOptions();
    send_goal_options_.feedback_callback =
        [this](GoalHandleLaunch::SharedPtr,
               const LaunchAction::Feedback::ConstSharedPtr feedback) {
          handle_feedback(feedback);
        };
    send_goal_options_.result_callback =
        [this, bond_id](const GoalHandleLaunch::WrappedResult& wrapped_result) {
          handle_result(wrapped_result, bond_id);
        };

    goal_handle_future_ = action_client_->async_send_goal(goal_msg, send_goal_options_);
    if (goal_handle_future_.wait_for(default_timeout_) != std::future_status::ready)
    {
      throw runner_exception("timed out waiting for launch goal acceptance: " + launch_reference_);
    }

    auto goal_handle = goal_handle_future_.get();
    if (!goal_handle)
    {
      throw runner_exception("launch goal rejected: " + launch_reference_);
    }

    {
      std::lock_guard<std::mutex> lock(state_mutex_);
      goal_handle_ = goal_handle;
    }

    RCLCPP_INFO(node_->get_logger(), "launch runner started: %s", launch_reference_.c_str());

    // emit started event
    emit_started(bond_id, "", param_on_started());
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

    if (!action_client_)
      throw runner_exception("cannot stop launch runner that was not started");

    GoalHandleLaunch::SharedPtr goal_handle;
    {
      std::lock_guard<std::mutex> lock(state_mutex_);
      stop_requested_ = true;
      goal_handle = goal_handle_;
    }

    if (goal_handle)
    {
      try
      {
        auto cancel_future = action_client_->async_cancel_goal(goal_handle);
        if (cancel_future.wait_for(default_timeout_) != std::future_status::ready)
        {
          throw runner_exception("timed out waiting for launch cancellation: " + launch_reference_);
        }

        auto response = cancel_future.get();
        if (response == nullptr)
        {
          throw runner_exception("launch cancellation returned no response: " + launch_reference_);
        }

        if (response->return_code != action_msgs::srv::CancelGoal_Response::ERROR_NONE &&
            response->return_code != action_msgs::srv::CancelGoal_Response::ERROR_GOAL_TERMINATED)
        {
          throw runner_exception("launch cancellation failed with code " + std::to_string(response->return_code) +
                                 " for: " + launch_reference_);
        }
      }
      catch (const rclcpp_action::exceptions::UnknownGoalHandleError& e)
      {
        RCLCPP_WARN(node_->get_logger(), "launch goal already finished while stopping %s: %s",
                    launch_reference_.c_str(), e.what());
      }
    }

    cleanup_client_node();

    emit_stopped_once(bond_id, instance_id);
  }

protected:
  void handle_feedback(const LaunchAction::Feedback::ConstSharedPtr& feedback)
  {
    if (!feedback)
    {
      return;
    }

    {
      std::lock_guard<std::mutex> lock(state_mutex_);
      if (!feedback->launch_id.empty())
      {
        launch_id_ = feedback->launch_id;
      }
      if (!feedback->resolved_launch_file_path.empty())
      {
        resolved_launch_file_path_ = feedback->resolved_launch_file_path;
      }
    }

    if (!feedback->log_text.empty())
    {
      if (feedback->log_level == "fatal" || feedback->log_level == "error")
      {
        RCLCPP_ERROR(node_->get_logger(), "launch feedback [%s]: %s", feedback->log_source.c_str(),
                     feedback->log_text.c_str());
      }
      else if (feedback->log_level == "warn")
      {
        RCLCPP_WARN(node_->get_logger(), "launch feedback [%s]: %s", feedback->log_source.c_str(),
                    feedback->log_text.c_str());
      }
      else
      {
        RCLCPP_INFO(node_->get_logger(), "launch feedback [%s]: %s", feedback->log_source.c_str(),
                    feedback->log_text.c_str());
      }
    }
  }

  void handle_result(const GoalHandleLaunch::WrappedResult& wrapped_result, const std::string& bond_id)
  {
    bool should_emit_failure = false;
    bool should_emit_stopped = false;
    std::string final_state;
    std::string result_message;

    {
      std::lock_guard<std::mutex> lock(state_mutex_);
      goal_handle_.reset();

      if (wrapped_result.result)
      {
        launch_id_ = wrapped_result.result->launch_id;
        resolved_launch_file_path_ = wrapped_result.result->resolved_launch_file_path;
        final_state_ = wrapped_result.result->final_state;
        result_message_ = wrapped_result.result->message;
      }
      else
      {
        final_state_.clear();
        result_message_.clear();
      }

      final_state = final_state_;
      result_message = result_message_;

      if (wrapped_result.code == rclcpp_action::ResultCode::ABORTED ||
          wrapped_result.code == rclcpp_action::ResultCode::UNKNOWN ||
          final_state == "failed" ||
          final_state == "rejected")
      {
        should_emit_failure = !failed_event_emitted_;
        failed_event_emitted_ = true;
      }
      else if (!stop_event_emitted_)
      {
        should_emit_stopped = true;
        stop_event_emitted_ = true;
      }
    }

    if (should_emit_failure)
    {
      RCLCPP_ERROR(node_->get_logger(), "launch runner failed for %s: %s",
                   launch_reference_.c_str(), result_message.c_str());
      emit_failed(bond_id, "", param_on_failure());
      return;
    }

    if (should_emit_stopped)
    {
      RCLCPP_INFO(node_->get_logger(), "launch runner completed for %s with state %s",
                  launch_reference_.c_str(), final_state.c_str());
      emit_stopped(bond_id, "", param_on_stopped());
    }
  }

  void emit_stopped_once(const std::string& bond_id, const std::string& instance_id)
  {
    bool should_emit = false;
    {
      std::lock_guard<std::mutex> lock(state_mutex_);
      should_emit = !stop_event_emitted_;
      stop_event_emitted_ = true;
    }

    if (should_emit)
    {
      emit_stopped(bond_id, instance_id, param_on_stopped());
    }
  }

  std::string resolve_launch_action_name()
  {
    constexpr const char* parameter_name = "launch_action_name";
    constexpr const char* default_action_name = "/capabilities2_launch/launch";

    if (!node_->has_parameter(parameter_name))
    {
      node_->declare_parameter<std::string>(parameter_name, default_action_name);
    }

    return node_->get_parameter(parameter_name).as_string();
  }

  void ensure_client_node()
  {
    if (client_node_)
    {
      return;
    }

    rclcpp::NodeOptions options;
    options.context(node_->get_node_base_interface()->get_context());
    options.start_parameter_services(false);
    options.start_parameter_event_publisher(false);
    options.use_global_arguments(false);

    client_node_ = std::make_shared<rclcpp::Node>(make_client_node_name(), options);
    client_executor_ = std::make_shared<rclcpp::executors::SingleThreadedExecutor>();
    client_executor_->add_node(client_node_);
    client_executor_thread_ = std::thread([this]() { client_executor_->spin(); });
  }

  void cleanup_client_node()
  {
    if (client_executor_)
    {
      client_executor_->cancel();
    }

    if (client_executor_thread_.joinable())
    {
      client_executor_thread_.join();
    }

    if (client_executor_ && client_node_)
    {
      client_executor_->remove_node(client_node_);
    }

    action_client_.reset();
    client_executor_.reset();
    client_node_.reset();
  }

  std::string make_client_node_name() const
  {
    std::string base = "capabilities2_launch_runner_" + run_config_.provider;
    for (char& character : base)
    {
      if (!std::isalnum(static_cast<unsigned char>(character)))
      {
        character = '_';
      }
    }

    const std::string suffix = std::to_string(reinterpret_cast<std::uintptr_t>(this));
    if (base.size() > 40)
    {
      base.resize(40);
    }

    return base + "_" + suffix;
  }

protected:
  static constexpr auto default_timeout_ = std::chrono::seconds(10);

  typename rclcpp_action::Client<LaunchAction>::SharedPtr action_client_;
  rclcpp::Node::SharedPtr client_node_;
  rclcpp::executors::SingleThreadedExecutor::SharedPtr client_executor_;
  std::thread client_executor_thread_;
  typename rclcpp_action::Client<LaunchAction>::SendGoalOptions send_goal_options_;
  typename GoalHandleLaunch::SharedPtr goal_handle_;
  std::shared_future<typename GoalHandleLaunch::SharedPtr> goal_handle_future_;
  std::mutex state_mutex_;
  std::string launch_reference_;
  std::string launch_id_;
  std::string resolved_launch_file_path_;
  std::string final_state_;
  std::string result_message_;
  bool stop_requested_{ false };
  bool stop_event_emitted_{ false };
  bool failed_event_emitted_{ false };
};

}  // namespace capabilities2_runner
