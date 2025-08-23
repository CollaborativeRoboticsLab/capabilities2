#pragma once

#include "rclcpp/rclcpp.hpp"

#include <tinyxml2.h>
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
    // if parameters are not provided then cannot proceed
    if (!parameters_[id])
      throw runner_exception("cannot trigger service without parameters");

    // generate a goal from parameters if provided
    auto request_msg = std::make_shared<typename ServiceT::Request>(generate_request(parameters_[id], id));

    info_("request generated for event :", id);

    std::unique_lock<std::mutex> lock(mutex_);
    completed_ = false;

    auto result_future = service_client_->async_send_request(
        request_msg, [this, id](typename rclcpp::Client<ServiceT>::SharedFuture future) {
          if (!future.valid())
          {
            error_("get result call failed");

            // trigger the events related to on_failure state
            if (events[id].on_failure.interface != "")
            {
              event_(EventType::FAILED, id, events[id].on_failure.interface, events[id].on_failure.provider);
              triggerFunction_(events[id].on_failure.interface, update_on_failure(events[id].on_failure.parameters));
            }
          }
          else
          {
            info_("get result call succeeded", id);

            response_ = future.get();
            process_response(response_, id);

            // trigger the events related to on_success state
            if (events[id].on_success.interface != "")
            {
              event_(EventType::SUCCEEDED, id, events[id].on_success.interface, events[id].on_success.provider);
              triggerFunction_(events[id].on_success.interface, update_on_success(events[id].on_success.parameters));
            }
          }

          completed_ = true;
          cv_.notify_all();
        });

    // trigger the events related to on_started state
    if (events[id].on_started.interface != "")
    {
      event_(EventType::STARTED, id, events[id].on_started.interface, events[id].on_started.provider);
      triggerFunction_(events[id].on_started.interface, update_on_started(events[id].on_started.parameters));
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
    if (events[runner_id].on_stopped.interface != "")
    {
      event_(EventType::STOPPED, -1, events[runner_id].on_stopped.interface, events[runner_id].on_stopped.provider);
      triggerFunction_(events[runner_id].on_stopped.interface,
                       update_on_stopped(events[runner_id].on_stopped.parameters));
    }

    info_("removing event options");

    // remove all event options for this runner instance
    const auto n = events.size();
    events.clear();
    info_("removed event options for " + std::to_string(n) + " runner ids");

    info_("runner cleaned. stopping..");
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

  /**
   * @brief Process the reponse and print data as required
   *
   * @param response service reponse
   * @param id thread id
   */
  virtual void process_response(typename ServiceT::Response::SharedPtr response, int id)
  {
  }

  typename rclcpp::Client<ServiceT>::SharedPtr service_client_;
  typename ServiceT::Response::SharedPtr response_;
};
}  // namespace capabilities2_runner