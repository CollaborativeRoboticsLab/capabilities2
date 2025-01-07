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
                            const std::string& service_name)
  {
    // initialize the runner base by storing node pointer and run config
    init_base(node, run_config);

    // create an service client
    service_client_ = node_->create_client<ServiceT>(service_name);

    // wait for action server
    RCLCPP_INFO(node_->get_logger(), "%s waiting for service: %s", run_config_.interface.c_str(), service_name.c_str());

    if (!service_client_->wait_for_service(std::chrono::seconds(3)))
    {
      RCLCPP_ERROR(node_->get_logger(), "%s failed to connect to service: %s", run_config_.interface.c_str(),
                   service_name.c_str());
      throw runner_exception("failed to connect to server");
    }
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
      throw runner_exception("cannot trigger service without parameters");

    // generate a goal from parameters if provided
    auto request_msg = std::make_shared<typename ServiceT::Request>(generate_request(parameters_));

    auto result_future = service_client_->async_send_request(
        request_msg, [this](typename rclcpp::Client<ServiceT>::SharedFuture future) {
          if (!future.valid())
          {
            RCLCPP_ERROR(node_->get_logger(), "get result call failed");

            // trigger the events related to on_failure state
            if (events[execute_id].on_failure != "")
            {
              triggerFunction_(events[execute_id].on_failure, update_on_failure(events[execute_id].on_failure_param));
            }
          }
          else
          {
            RCLCPP_INFO(node_->get_logger(), "get result call succeeded");

            response_ = future.get();

            // trigger the events related to on_success state
            if (events[execute_id].on_success != "")
            {
              triggerFunction_(events[execute_id].on_success, update_on_success(events[execute_id].on_success_param));
            }
          }
        });

    // trigger the events related to on_started state
    if (events[execute_id].on_started != "")
    {
      triggerFunction_(events[execute_id].on_started, update_on_started(events[execute_id].on_started_param));
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

    if (!service_client_)
      throw runner_exception("cannot stop runner action that was not started");

    // Trigger on_stopped event if defined
    if (events[execute_id].on_stopped != "")
    {
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
  virtual typename ServiceT::Request generate_request(tinyxml2::XMLElement* parameters) = 0;

  typename rclcpp::Client<ServiceT>::SharedPtr service_client_;
  typename ServiceT::Response::SharedPtr response_;
};
}  // namespace capabilities2_runner