#pragma once

#include <memory>

#include <capabilities2_runner/threadtrigger_runner.hpp>

namespace capabilities2_runner
{

/**
 * @brief Topic runner base class
 *
 * Create an topic subsriber for data grabbing capability
 */
template <typename TopicT>
class TopicRunner : public ThreadTriggerRunner
{
public:
  /**
   * @brief Constructor which needs to be empty due to plugin semantics
   */
  TopicRunner() : ThreadTriggerRunner()
  {
  }

  /**
   * @brief Initializer function for initializing the topic runner in place of constructor due to plugin semantics
   *
   * @param node shared pointer to the capabilities node. Allows to use ros node related functionalities
   * @param run_config runner configuration loaded from the yaml file
   * @param topic_name topic name used in the yaml file, used to load specific configuration from the run_config
   */
  virtual void init_subscriber(rclcpp::Node::SharedPtr node, const runner_opts& run_config,
                               const std::string& topic_name)
  {
    // initialize the runner base by storing node pointer and run config
    init_base(node, run_config);

    // create an service client
    subscription_ = node_->create_subscription<TopicT>(
        topic_name, 10, [this](const typename TopicT::SharedPtr msg) { this->callback(msg); });
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

    // throw an error if the subscription is null
    // this can happen if the runner is not able to find the topic resource

    if (!subscription_)
      throw runner_exception("cannot stop runner subscriber that was not started");

    // emit stop event
    emit_stopped(bond_id, instance_id, param_on_stopped());

    RCLCPP_INFO(node_->get_logger(), "runner cleaned. stopping..");
  }

protected:
  /**
   * @brief Trigger process to be executed.
   *
   * This method utilizes paramters set via the trigger() function
   *
   * @param parameters pointer to tinyxml2::XMLElement that contains parameters
   * @param thread_id unique identifier for the execution thread
   */
  virtual void execution(capabilities2_events::EventParameters parameters, const std::string& thread_id) override
  {
    // split thread_id to get bond_id and instance_id (format: "bond_id/instance_id")
    std::string bond_id = ThreadTriggerRunner::bond_from_thread_id(thread_id);
    std::string instance_id = ThreadTriggerRunner::instance_from_thread_id(thread_id);

    // emit started event
    emit_started(bond_id, instance_id, param_on_started());

    // block until a message is received in the callback and stored in latest_message_
    std::unique_lock<std::mutex> lock(block_mutex_);
    completed_ = false;

    RCLCPP_INFO(node_->get_logger(), "Waiting for Message.");

    // Conditional wait
    cv_.wait(lock, [this] { return completed_; });
    RCLCPP_INFO(node_->get_logger(), "Message Received.");

    if (latest_message_)
    {
      // emit success event
      emit_succeeded(bond_id, instance_id, param_on_success());
    }
    else
    {
      RCLCPP_ERROR(node_->get_logger(), "Message receiving failed.");
      // emit failed event
      emit_failed(bond_id, instance_id, param_on_failure());
    }

    RCLCPP_INFO(node_->get_logger(), "Thread closing.");
  }

protected:
  /**
   * @brief Callback to be executed when receiving a message
   *
   * This function is used grab the messages from the callback queue into a class
   * parameter so that it can be used later on dering trigger
   *
   * @param msg message parameter
   */
  void callback(const typename TopicT::SharedPtr& msg)
  {
    latest_message_ = msg;

    completed_ = true;
    cv_.notify_all();
  }

  /**
   * @brief parameter to be used in the on_started event emission
   */
  typename rclcpp::Subscription<TopicT>::SharedPtr subscription_;

  /**
   * @brief parameter to store the latest message received in the callback for use in the trigger execution
   */
  mutable typename TopicT::SharedPtr latest_message_;

  /**
   * @brief condition variable and mutex for synchronizing the callback and trigger execution
   */
  std::condition_variable cv_;
  std::mutex block_mutex_;
  bool completed_;
};

}  // namespace capabilities2_runner
