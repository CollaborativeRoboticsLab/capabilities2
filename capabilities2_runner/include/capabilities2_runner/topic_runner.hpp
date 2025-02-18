#pragma once
#include <tinyxml2.h>
#include <memory>

#include "rclcpp/rclcpp.hpp"

#include <capabilities2_runner/runner_base.hpp>

namespace capabilities2_runner
{

/**
 * @brief Topic runner base class
 *
 * Create an topic subsriber for data grabbing capability
 */
template <typename TopicT>
class TopicRunner : public RunnerBase
{
public:
  /**
   * @brief Constructor which needs to be empty due to plugin semantics
   */
  TopicRunner() : RunnerBase()
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
                               const std::string& topic_name, std::function<void(Event&)> print)
  {
    // initialize the runner base by storing node pointer and run config
    init_base(node, run_config, print);

    // create an service client
    subscription_ = node_->create_subscription<TopicT>(
        topic_name, 10, [this](const typename TopicT::SharedPtr msg) { this->callback(msg); });
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
      throw runner_exception("cannot grab data without parameters");

    // trigger the events related to on_started state
    if (events[execute_id].on_started != "")
    {
      info_("on_started", id, events[execute_id].on_started, EventType::STARTED);
      triggerFunction_(events[execute_id].on_started, update_on_started(events[execute_id].on_started_param));
    }

    std::unique_lock<std::mutex> lock(mutex_);
    completed_ = false;

    info_("Waiting for Message.", id);

    // Conditional wait
    cv_.wait(lock, [this] { return completed_; });
    info_("Message Received.", id);

    if (latest_message_)
    {
      // trigger the events related to on_success state
      if (events[execute_id].on_success != "")
      {
        info_("on_success", id, events[execute_id].on_success, EventType::SUCCEEDED);
        triggerFunction_(events[execute_id].on_success, update_on_success(events[execute_id].on_success_param));
      }
    }
    else
    {
      error_("Message receving failed.");

      // trigger the events related to on_failure state
      if (events[execute_id].on_failure != "")
      {
        info_("on_failure", id, events[execute_id].on_failure, EventType::FAILED);
        triggerFunction_(events[execute_id].on_failure, update_on_failure(events[execute_id].on_failure_param));
      }
    }

    info_("Thread closing.", id);
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

    // throw an error if the service client is null
    // this can happen if the runner is not able to find the action resource

    if (!subscription_)
      throw runner_exception("cannot stop runner subscriber that was not started");

    // Trigger on_stopped event if defined
    if (events[execute_id].on_stopped != "")
    {
      info_("on_stopped", -1, events[execute_id].on_stopped, EventType::STOPPED);
      triggerFunction_(events[execute_id].on_stopped, update_on_stopped(events[execute_id].on_stopped_param));
    }
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