#pragma once
#include <tinyxml2.h>

#include "rclcpp/rclcpp.hpp"

#include <capabilities2_runner/runner_base.hpp>

namespace capabilities2_runner
{

/**
 * @brief service runner base class
 *
 * Create an server client to run an service based capability
 */
template <typename ServiceT>
class ServiceRunner : public RunnerBase
{
public:
  /**
   * @brief Constructor which needs to be empty due to plugin semantics
   */
  ServiceRunner() : RunnerBase()
  {
  }

  /**
   * @brief Initializer function for initializing the service runner in place of constructor due to plugin semantics
   *
   * @param node shared pointer to the capabilities node. Allows to use ros node related functionalities
   * @param run_config runner configuration loaded from the yaml file
   * @param service_name action name used in the yaml file, used to load specific configuration from the run_config
   */
  virtual void init_service(rclcpp::Node::SharedPtr node, const runner_opts& run_config,
                            const std::string& service_name, std::function<void(Event&)> print)
  {
    // initialize the runner base by storing node pointer and run config
    init_base(node, run_config, print);

    // create an service client
    service_client_ = node_->create_client<ServiceT>(service_name);

    // wait for action server
    info_("waiting for service: " + service_name);

    if (!service_client_->wait_for_service(std::chrono::seconds(3)))
    {
      error_("failed to connect to service: " + service_name);
      throw runner_exception("failed to connect to server");
    }

    info_("connected with service: " + service_name);
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
      throw runner_exception("cannot trigger service without parameters");

    // generate a goal from parameters if provided
    auto request_msg = std::make_shared<typename ServiceT::Request>(generate_request(parameters_[id], id));

    info_("request generated", id);

    std::unique_lock<std::mutex> lock(mutex_);
    completed_ = false;

    auto result_future = service_client_->async_send_request(
        request_msg, [this, id](typename rclcpp::Client<ServiceT>::SharedFuture future) {
          if (!future.valid())
          {
            error_("get result call failed");

            // trigger the events related to on_failure state
            if (events[execute_id].on_failure != "")
            {
              info_("on_failure", id, events[execute_id].on_failure, EventType::FAILED);
              triggerFunction_(events[execute_id].on_failure, update_on_failure(events[execute_id].on_failure_param));
            }
          }
          else
          {
            info_("get result call succeeded", id);

            response_ = future.get();

            // trigger the events related to on_success state
            if (events[execute_id].on_success != "")
            {
              info_("on_success", id, events[execute_id].on_success, EventType::SUCCEEDED);
              triggerFunction_(events[execute_id].on_success, update_on_success(events[execute_id].on_success_param));
            }
          }

          completed_ = true;
          cv_.notify_all();
        });

    // trigger the events related to on_started state
    if (events[execute_id].on_started != "")
    {
      info_("on_started", id, events[execute_id].on_started, EventType::STARTED);
      triggerFunction_(events[execute_id].on_started, update_on_started(events[execute_id].on_started_param));
    }

    // Conditional wait
    cv_.wait(lock, [this] { return completed_; });
    info_("Service request complete. Thread closing.", id);
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

    if (!service_client_)
      throw runner_exception("cannot stop runner action that was not started");

    // Trigger on_stopped event if defined
    if (events[execute_id].on_stopped != "")
    {
      info_("on_stopped", -1, events[execute_id].on_stopped, EventType::STOPPED);
      triggerFunction_(events[execute_id].on_stopped, update_on_stopped(events[execute_id].on_stopped_param));
    }
  }

protected:
  /**
   * @brief Generate a request from parameters
   *
   * This function is used in conjunction with the trigger function to inject type erased parameters
   * into the typed service
   *
   * A pattern needs to be implemented in the derived class
   *
   * @param parameters
   * @return ServiceT::Request the generated request
   */
  virtual typename ServiceT::Request generate_request(tinyxml2::XMLElement* parameters, int id) = 0;

  typename rclcpp::Client<ServiceT>::SharedPtr service_client_;
  typename ServiceT::Response::SharedPtr response_;
};
}  // namespace capabilities2_runner