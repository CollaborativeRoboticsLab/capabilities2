#pragma once

#include <chrono>

#include <capabilities2_runner/threadtrigger_runner.hpp>

namespace capabilities2_runner
{

/**
 * @brief service runner base class
 *
 * Create an server client to run an service based capability
 */
template <typename ServiceT>
class ServiceRunner : public ThreadTriggerRunner
{
public:
  /**
   * @brief Constructor which needs to be empty due to plugin semantics
   */
  ServiceRunner() : ThreadTriggerRunner()
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

    // create a service client
    service_client_ = node_->create_client<ServiceT>(service_name);

    // wait for service server
    RCLCPP_INFO(node_->get_logger(), "waiting for service: %s", service_name.c_str());

    if (!service_client_->wait_for_service(std::chrono::seconds(3)))
    {
      RCLCPP_ERROR(node_->get_logger(), "failed to connect to service: %s", service_name.c_str());
      throw runner_exception("failed to connect to server");
    }

    RCLCPP_INFO(node_->get_logger(), "connected with service: %s", service_name.c_str());
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

    // throw an error if the service client is null
    // this can happen if the runner is not able to find the action resource

    if (!service_client_)
      throw runner_exception("cannot stop runner action that was not started");

    // emit stopped event
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

    // generate a goal from parameters if provided
    auto request_msg = std::make_shared<typename ServiceT::Request>(generate_request(parameters));

    RCLCPP_INFO(node_->get_logger(), "request generated for event :%s", instance_id.c_str());

    std::mutex block_mutex;
    std::unique_lock<std::mutex> lock(block_mutex);
    std::condition_variable cv;
    bool completed = false;

    auto result_future = service_client_->async_send_request(
        request_msg, [this,  &instance_id, &completed, &bond_id, &cv](typename rclcpp::Client<ServiceT>::SharedFuture future) {
          if (!future.valid())
          {
            RCLCPP_ERROR(node_->get_logger(), "get result call failed");

            // emit failed event
            emit_failed(bond_id, instance_id, param_on_failure());
          }
          else
          {
            RCLCPP_INFO(node_->get_logger(), "get result call succeeded for event :%s", instance_id.c_str());

            response_ = future.get();
            process_response(response_);

            // emit success event
            emit_succeeded(bond_id, instance_id, param_on_success());
          }

          completed = true;
          cv.notify_all();
        });

    // Conditional wait
    cv.wait(lock, [&completed] { return completed; });
    RCLCPP_INFO(node_->get_logger(), "Service request complete. Thread closing.");
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
  virtual typename ServiceT::Request generate_request(capabilities2_events::EventParameters& parameters) = 0;

  /**
   * @brief Process the reponse and print data as required
   *
   * @param response service reponse message
   * @param trigger_id thread id associated with this response used for logging and event emission
   * @return capabilities2_events::EventParameters containing updated parameters for the on_success event if needed
   *
   * A pattern needs to be implemented in the derived class for processing the response and extracting data if needed,
   * currently does nothing.
   */
  virtual void process_response(typename ServiceT::Response::SharedPtr /*response*/) {}

  typename rclcpp::Client<ServiceT>::SharedPtr service_client_;
  typename ServiceT::Response::SharedPtr response_;
};
}  // namespace capabilities2_runner
