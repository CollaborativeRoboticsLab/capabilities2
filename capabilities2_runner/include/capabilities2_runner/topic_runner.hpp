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
                               const std::string& topic_name)
  {
    // initialize the runner base by storing node pointer and run config
    init_base(node, run_config);

    // create an service client
    subscription_ = node_->create_subscription<TopicT>(
        topic_name, 1, [this](const typename TopicT::SharedPtr msg) { this->callback(msg); });
  }

  /**
   * @brief Trigger process to be executed.
   *
   * This method utilizes paramters set via the trigger() function
   *
   * @param parameters pointer to tinyxml2::XMLElement that contains parameters
   */
  virtual void triggerExecution() override
  {
    execute_id += 1;

    // if parameters are not provided then cannot proceed
    if (!parameters_)
      throw runner_exception("cannot grab data without parameters");

    // trigger the events related to on_started state
    if (events[execute_id].on_started != "")
    {
      triggerFunction_(events[execute_id].on_started, update_on_started(events[execute_id].on_started_param));
    }

    if (latest_message_)
    {
      // trigger the events related to on_success state
      if (events[execute_id].on_success != "")
      {
        triggerFunction_(events[execute_id].on_success, update_on_success(events[execute_id].on_success_param));
      }
    }
    else
    {
      RCLCPP_ERROR(node_->get_logger(), "get result call failed");

      // trigger the events related to on_failure state
      if (events[execute_id].on_failure != "")
      {
        triggerFunction_(events[execute_id].on_failure, update_on_failure(events[execute_id].on_failure_param));
      }
    }
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
  void callback(const typename TopicT::SharedPtr& msg) const
  {
    latest_message_ = msg;
  }

  typename rclcpp::Subscription<TopicT>::SharedPtr subscription_;

  mutable typename TopicT::SharedPtr latest_message_{ nullptr };
};
}  // namespace capabilities2_runner