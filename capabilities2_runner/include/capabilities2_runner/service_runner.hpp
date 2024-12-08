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
   * @brief the trigger function on the service runner is used to trigger an service.
   * this method provides a mechanism for injecting parameters or a goal into a service
   * and then trigger the service
   *
   * @param parameters
   * @return std::optional<std::function<void(tinyxml2::XMLElement*)>>
   */
  virtual std::optional<std::function<void(tinyxml2::XMLElement*)>>
  trigger(tinyxml2::XMLElement* parameters = nullptr) override
  {
    execute_id += 1;

    // if parameters are not provided then cannot proceed
    if (!parameters)
      throw runner_exception("cannot trigger service without parameters");

    // generate a goal from parameters if provided
    auto request_msg = std::make_shared<typename ServiceT::Request>(generate_request(parameters));

    auto result_future = service_client_->async_send_request(
        request_msg, [this](typename rclcpp::Client<ServiceT>::SharedFuture future) {
          if (!future.valid())
          {
            RCLCPP_ERROR(node_->get_logger(), "get result call failed");
            events[execute_id].on_failure(update_on_failure(events[execute_id].on_failure_param));
          }
          else
          {
            RCLCPP_INFO(node_->get_logger(), "get result call succeeded");
            events[execute_id].on_success(update_on_success(events[execute_id].on_success_param));
          }
        });

    // Trigger started event if defined
    events[execute_id].on_started(update_on_started(events[execute_id].on_started_param));

    // Define a callback function to handle the result once it's ready
    std::function<void(tinyxml2::XMLElement*)> result_callback =
        [this, &result_future](tinyxml2::XMLElement* result) mutable {
          auto response = result_future.get();
          result = generate_response(response);

          // Ensure the future is ready before accessing the result
          if (result_future.wait_for(std::chrono::seconds(0)) == std::future_status::ready)
          {
            auto response = result_future.get();
            result = generate_response(response);
          }
          else
          {
            RCLCPP_WARN(node_->get_logger(), "Result is not ready yet.");
          }
        };

    return result_callback;
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

    // publish event
    events[execute_id].on_stopped(update_on_stopped(events[execute_id].on_stopped_param));
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

  /**
   * @brief generate a typed erased response
   *
   * This method is used in a callback passed to the trigger caller to get type erased result
   * from the service the reponse can be passed by the caller or ignored
   *
   * The pattern needs to be implemented in the derived class
   *
   * @param wrapped_result
   * @return tinyxml2::XMLElement*
   */
  virtual tinyxml2::XMLElement* generate_response(const typename ServiceT::Response::SharedPtr& result) const = 0;

  typename rclcpp::Client<ServiceT>::SharedPtr service_client_;
};
}  // namespace capabilities2_runner