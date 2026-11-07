#pragma once

#include <atomic>
#include <chrono>
#include <cctype>
#include <cmath>
#include <condition_variable>
#include <future>
#include <mutex>
#include <string>
#include <thread>

#include <action_msgs/srv/cancel_goal.hpp>
#include <lifecycle_msgs/msg/state.hpp>
#include <lifecycle_msgs/srv/get_state.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <rclcpp_action/create_generic_client.hpp>
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

  ~LaunchRunner() override
  {
    cleanup_client_node();
  }

  LaunchAction::Goal make_goal_message() const
  {
    LaunchAction::Goal goal_msg;
    goal_msg.reference_name = run_config_.provider.empty() ? run_config_.interface : run_config_.provider;
    goal_msg.launch_file_path = launch_reference_;

    for (const auto& launch_parameter : run_config_.launch_parameters)
    {
      capabilities2_msgs::msg::LaunchParameter parameter_msg;
      parameter_msg.key = launch_parameter.key;
      parameter_msg.value = launch_parameter.value;
      goal_msg.launch_parameters.push_back(parameter_msg);
    }

    return goal_msg;
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
      launched_event_emitted_ = false;
      stop_event_emitted_ = false;
      failed_event_emitted_ = false;
      goal_handle_.reset();
      launch_id_.clear();
      resolved_launch_file_path_.clear();
      final_state_.clear();
      result_message_.clear();
      launch_finished_ = false;
      result_code_ = rclcpp_action::ResultCode::UNKNOWN;
    }

    if (!run_config_.launch_readiness.lifecycle_nodes.empty() &&
      (!std::isfinite(run_config_.launch_readiness.lifecycle_timeout_sec) ||
       run_config_.launch_readiness.lifecycle_timeout_sec <= 0.0))
    {
      throw runner_exception("lifecycle readiness timeout must be positive and finite");
    }

    const std::string action_name = resolve_launch_action_name();
    ensure_client_node();
    bool goal_response_pending = false;
    try
    {
      const auto& readiness = run_config_.launch_readiness;
      validate_endpoint(readiness.action, "action");
      validate_endpoint(readiness.service, "service");
      validate_endpoint(readiness.topic, "topic");
      auto topic_received = std::make_shared<std::atomic<bool>>(false);
      rclcpp::GenericSubscription::SharedPtr topic_subscription;
      if (!readiness.topic.name.empty())
      {
        topic_subscription = client_node_->create_generic_subscription(
            resolve_readiness_name(readiness.topic.name), readiness.topic.type, rclcpp::SensorDataQoS(),
            [topic_received](std::shared_ptr<rclcpp::SerializedMessage>) { topic_received->store(true); });
      }
      action_client_ = rclcpp_action::create_client<LaunchAction>(client_node_, action_name);

      RCLCPP_INFO(node_->get_logger(), "waiting for launch action: %s", action_name.c_str());
      if (!action_client_->wait_for_action_server(default_timeout_))
      {
        throw runner_exception("failed to connect to launch action server: " + action_name);
      }

      LaunchAction::Goal goal_msg = make_goal_message();

      send_goal_options_ = typename rclcpp_action::Client<LaunchAction>::SendGoalOptions();
      send_goal_options_.feedback_callback = [this, bond_id](GoalHandleLaunch::SharedPtr,
                                                             const LaunchAction::Feedback::ConstSharedPtr feedback) {
        handle_feedback(feedback, bond_id);
      };
      send_goal_options_.result_callback = [this, bond_id](const GoalHandleLaunch::WrappedResult& wrapped_result) {
        handle_result(wrapped_result, bond_id);
      };

      goal_handle_future_ = action_client_->async_send_goal(goal_msg, send_goal_options_);
      if (goal_handle_future_.wait_for(default_timeout_) != std::future_status::ready)
      {
        goal_response_pending = true;
        throw runner_exception("timed out waiting for launch goal acceptance: " + launch_reference_);
      }

      auto goal_handle = goal_handle_future_.get();
      if (!goal_handle)
      {
        throw runner_exception("launch goal rejected: " + launch_reference_);
      }

      {
        std::lock_guard<std::mutex> lock(state_mutex_);
        if (!launch_finished_)
        {
          goal_handle_ = goal_handle;
        }
      }

      wait_for_lifecycle_nodes();
      wait_for_endpoints(topic_received);

      RCLCPP_INFO(node_->get_logger(), "launch runner started: %s", launch_reference_.c_str());

      // emit started event
      emit_started(bond_id, "", param_on_started());
    }
    catch (const std::exception& error)
    {
      GoalHandleLaunch::SharedPtr goal_handle;
      bool emit_failure;
      bool cleanup_unconfirmed = goal_response_pending;
      {
        std::lock_guard<std::mutex> lock(state_mutex_);
        goal_handle = goal_handle_;
        stop_requested_ = true;
        stop_event_emitted_ = true;
        emit_failure = !failed_event_emitted_;
        failed_event_emitted_ = true;
        if (launch_finished_ &&
            !((result_code_ == rclcpp_action::ResultCode::CANCELED && final_state_ == "canceled") ||
              (result_code_ == rclcpp_action::ResultCode::SUCCEEDED && final_state_ == "stopped") ||
              (result_code_ == rclcpp_action::ResultCode::ABORTED && final_state_ == "failed")))
        {
          cleanup_unconfirmed = true;
        }
      }
      if (goal_handle && action_client_)
      {
        try
        {
          stop(bond_id);
        }
        catch (const std::exception& cleanup_error)
        {
          cleanup_unconfirmed = true;
          RCLCPP_WARN(node_->get_logger(), "failed to cancel launch %s: %s", launch_reference_.c_str(),
                      cleanup_error.what());
        }
      }
      if (emit_failure)
      {
        emit_failed(bond_id, "", param_on_failure());
      }
      if (cleanup_unconfirmed)
      {
        throw runner_unconfirmed_exception(error.what());
      }
      cleanup_client_node();
      throw runner_exception(error.what());
    }
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

    std::exception_ptr stop_error;
    try
    {
      if (!goal_handle && goal_handle_future_.valid())
      {
        if (goal_handle_future_.wait_for(default_timeout_) != std::future_status::ready)
        {
          throw runner_exception("timed out waiting for launch goal acceptance: " + launch_reference_);
        }
        goal_handle = goal_handle_future_.get();
        if (!goal_handle)
        {
          cleanup_client_node();
          emit_stopped_once(bond_id, instance_id);
          return;
        }
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

      std::unique_lock<std::mutex> lock(state_mutex_);
      if (!state_changed_.wait_for(lock, shutdown_timeout_, [this]() { return launch_finished_; }))
      {
        throw runner_exception("timed out waiting for launch termination: " + launch_reference_);
      }
      if (!((result_code_ == rclcpp_action::ResultCode::CANCELED && final_state_ == "canceled") ||
        (result_code_ == rclcpp_action::ResultCode::SUCCEEDED && final_state_ == "stopped") ||
        (result_code_ == rclcpp_action::ResultCode::ABORTED && final_state_ == "failed")))
      {
        throw runner_exception("launch termination was not confirmed for " + launch_reference_ + ": " +
                               result_message_);
      }
    }
    catch (...)
    {
      stop_error = std::current_exception();
    }

    if (stop_error)
    {
      std::rethrow_exception(stop_error);
    }
    cleanup_client_node();
    emit_stopped_once(bond_id, instance_id);
  }

protected:
  void validate_endpoint(const readiness_endpoint& endpoint, const std::string& kind) const
  {
    if (endpoint.name.empty() && endpoint.type.empty())
      return;
    if (endpoint.name.empty() || endpoint.type.empty())
      throw runner_exception(kind + " readiness requires both name and type");
    if (!std::isfinite(endpoint.timeout) || endpoint.timeout <= 0.0)
      throw runner_exception(kind + " readiness timeout must be positive and finite");
  }

  std::string resolve_readiness_name(const std::string& configured_name) const
  {
    if (configured_name.front() == '/')
      return configured_name;
    std::string name_space = run_config_.global_namespace;
    for (const auto& parameter : run_config_.launch_parameters)
    {
      if (parameter.key == "namespace")
      {
        name_space = parameter.value;
        break;
      }
    }
    if (!name_space.empty() && name_space.front() != '/')
      name_space.insert(name_space.begin(), '/');
    while (!name_space.empty() && name_space.back() == '/')
      name_space.pop_back();
    return name_space + "/" + configured_name;
  }

  void check_launch_running()
  {
    std::lock_guard<std::mutex> lock(state_mutex_);
    if (launch_finished_)
      throw runner_exception("launch ended before readiness: " + result_message_);
    if (!rclcpp::ok(node_->get_node_base_interface()->get_context()))
      throw runner_exception("ROS shutdown before launch readiness");
  }

  template <typename Predicate>
  void wait_for_endpoint(const readiness_endpoint& endpoint, const std::string& kind, Predicate ready)
  {
    if (endpoint.name.empty())
      return;
    const auto deadline = std::chrono::steady_clock::now() + std::chrono::duration<double>(endpoint.timeout);
    while (std::chrono::steady_clock::now() < deadline)
    {
      check_launch_running();
      if (ready())
      {
        check_launch_running();
        return;
      }
    }
    throw runner_exception("timed out waiting for " + kind + " readiness: " + endpoint.name);
  }

  void wait_for_endpoints(const std::shared_ptr<std::atomic<bool>>& topic_received)
  {
    const auto& readiness = run_config_.launch_readiness;
    if (!readiness.action.name.empty())
    {
      auto client = rclcpp_action::create_generic_client(
          client_node_, resolve_readiness_name(readiness.action.name), readiness.action.type);
      wait_for_endpoint(readiness.action, "action", [&client]() {
        return client->wait_for_action_server(std::chrono::milliseconds(100));
      });
    }
    if (!readiness.service.name.empty())
    {
      auto client = client_node_->create_generic_client(
          resolve_readiness_name(readiness.service.name), readiness.service.type);
      wait_for_endpoint(readiness.service, "service", [&client]() {
        return client->wait_for_service(std::chrono::milliseconds(100));
      });
    }
    if (!readiness.topic.name.empty())
    {
      wait_for_endpoint(readiness.topic, "topic", [&topic_received]() {
        if (topic_received->load())
          return true;
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
        return topic_received->load();
      });
    }
    check_launch_running();
  }

  void wait_for_lifecycle_nodes()
  {
    if (run_config_.launch_readiness.lifecycle_nodes.empty())
    {
      return;
    }

    using GetState = lifecycle_msgs::srv::GetState;
    std::vector<std::pair<std::string, rclcpp::Client<GetState>::SharedPtr>> clients;
    for (const auto& configured_name : run_config_.launch_readiness.lifecycle_nodes)
    {
      if (configured_name.empty())
      {
        throw runner_exception("lifecycle node name must not be empty");
      }
      const std::string name = resolve_readiness_name(configured_name);
      RCLCPP_INFO(node_->get_logger(), "waiting for lifecycle node to become active: %s", name.c_str());
      clients.emplace_back(name, client_node_->create_client<GetState>(name + "/get_state"));
    }

    const auto deadline = std::chrono::steady_clock::now() +
                std::chrono::duration<double>(run_config_.launch_readiness.lifecycle_timeout_sec);
    std::string waiting_for;
    while (rclcpp::ok(node_->get_node_base_interface()->get_context()) && std::chrono::steady_clock::now() < deadline)
    {
      bool all_active = true;
      for (const auto& entry : clients)
      {
        {
          std::lock_guard<std::mutex> lock(state_mutex_);
          if (launch_finished_)
          {
            throw runner_exception("launch ended before lifecycle readiness: " + result_message_);
          }
        }
        waiting_for = entry.first;
        if (!entry.second->wait_for_service(std::chrono::milliseconds(100)))
        {
          all_active = false;
          break;
        }
        auto future = entry.second->async_send_request(std::make_shared<GetState::Request>());
        bool response_ready = false;
        while (std::chrono::steady_clock::now() < deadline &&
               rclcpp::ok(node_->get_node_base_interface()->get_context()))
        {
          if (future.wait_for(std::chrono::milliseconds(100)) == std::future_status::ready)
          {
            response_ready = true;
            break;
          }
          std::lock_guard<std::mutex> lock(state_mutex_);
          if (launch_finished_)
          {
            entry.second->remove_pending_request(future);
            throw runner_exception("launch ended before lifecycle readiness: " + result_message_);
          }
        }
        if (!response_ready)
        {
          entry.second->remove_pending_request(future);
          all_active = false;
          break;
        }
        if (future.get()->current_state.id != lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE)
        {
          all_active = false;
          break;
        }
      }
      if (all_active)
      {
        std::lock_guard<std::mutex> lock(state_mutex_);
        if (launch_finished_)
        {
          throw runner_exception("launch ended before lifecycle readiness: " + result_message_);
        }
        return;
      }
      std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }
    throw runner_exception("timed out waiting for active lifecycle node: " + waiting_for);
  }

  void handle_feedback(const LaunchAction::Feedback::ConstSharedPtr& feedback, const std::string& bond_id)
  {
    if (!feedback)
    {
      return;
    }

    bool should_emit_launched = false;

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

      const int64_t launched_pid = feedback->resource.pid != 0 ?
                                       feedback->resource.pid :
                                       (feedback->main_pids.empty() ? 0 : feedback->main_pids.front());
      if (launched_pid > 0 && !launched_event_emitted_)
      {
        launched_event_emitted_ = true;
        should_emit_launched = true;
      }
    }

    if (should_emit_launched)
    {
      RCLCPP_INFO(node_->get_logger(), "emitting LAUNCHED event for %s", launch_reference_.c_str());
      emit_event(bond_id, "", capabilities2_msgs::msg::CapabilityEventCode::LAUNCHED);
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
      launch_finished_ = true;
      result_code_ = wrapped_result.code;
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
          wrapped_result.code == rclcpp_action::ResultCode::UNKNOWN || final_state == "failed" ||
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

    state_changed_.notify_all();

    if (should_emit_failure)
    {
      RCLCPP_ERROR(node_->get_logger(), "launch runner failed for %s: %s", launch_reference_.c_str(),
                   result_message.c_str());
      emit_failed(bond_id, "", param_on_failure());
      return;
    }

    if (should_emit_stopped)
    {
      RCLCPP_INFO(node_->get_logger(), "launch runner completed for %s with state %s", launch_reference_.c_str(),
                  final_state.c_str());
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
    constexpr const char* default_action_name = "/capabilities2_launch_proxy/launch";

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
    std::string base = "capabilities2_launch_proxy_runner_" + run_config_.provider;
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
  static constexpr auto shutdown_timeout_ = std::chrono::seconds(60);

  typename rclcpp_action::Client<LaunchAction>::SharedPtr action_client_;
  rclcpp::Node::SharedPtr client_node_;
  rclcpp::executors::SingleThreadedExecutor::SharedPtr client_executor_;
  std::thread client_executor_thread_;
  typename rclcpp_action::Client<LaunchAction>::SendGoalOptions send_goal_options_;
  typename GoalHandleLaunch::SharedPtr goal_handle_;
  std::shared_future<typename GoalHandleLaunch::SharedPtr> goal_handle_future_;
  std::mutex state_mutex_;
  std::condition_variable state_changed_;
  std::string launch_reference_;
  std::string launch_id_;
  std::string resolved_launch_file_path_;
  std::string final_state_;
  std::string result_message_;
  bool stop_requested_{ false };
  bool launched_event_emitted_{ false };
  bool stop_event_emitted_{ false };
  bool failed_event_emitted_{ false };
  bool launch_finished_{ false };
  rclcpp_action::ResultCode result_code_{ rclcpp_action::ResultCode::UNKNOWN };
};

}  // namespace capabilities2_runner
