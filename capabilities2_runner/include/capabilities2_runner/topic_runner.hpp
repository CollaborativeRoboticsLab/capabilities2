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
  virtual void stop(const std::string& bond_id) override
  {
    // if the node pointer is empty then throw an error
    // this means that the runner was not started and is being used out of order

    if (!node_)
      throw runner_exception("cannot stop runner that was not started");

    // throw an error if the service client is null
    // this can happen if the runner is not able to find the action resource

    if (!subscription_)
      throw runner_exception("cannot stop runner subscriber that was not started");

    // emit stop event
    emit_stopped(bond_id, update_on_stopped(events[runner_id].on_stopped.parameters));

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
  virtual void execution(const std::string& parameters, const std::string& thread_id) override
  {
    // split thread_id to get bond_id and trigger_id (format: "bond_id/trigger_id")
    std::string bond_id = ThreadTriggerRunner::bond_from_thread_id(thread_id);
    std::string trigger_id = ThreadTriggerRunner::trigger_from_thread_id(thread_id);

    // if parameters are not provided then cannot proceed
    if (!parameters_[trigger_id])
      throw runner_exception("cannot grab data without parameters");

    // emit started event
    emit_started(bond_id, update_on_started(events[trigger_id].on_started.parameters));
    std::unique_lock<std::mutex> lock(mutex_);
    completed_ = false;

    RCLCPP_INFO(node_->get_logger(), "Waiting for Message.");

    // Conditional wait
    cv_.wait(lock, [this] { return completed_; });
    RCLCPP_INFO(node_->get_logger(), "Message Received.");

    if (latest_message_)
    {
      // emit success event
      emit_succeeded(bond_id, update_on_success(events[trigger_id].on_success.parameters));
    }
    else
    {
      RCLCPP_ERROR(node_->get_logger(), "Message receiving failed.");
      // emit failed event
      emit_failed(bond_id, update_on_failure(events[trigger_id].on_failure.parameters));
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

  typename rclcpp::Subscription<TopicT>::SharedPtr subscription_;

  mutable typename TopicT::SharedPtr latest_message_;
};

}  // namespace capabilities2_runner
