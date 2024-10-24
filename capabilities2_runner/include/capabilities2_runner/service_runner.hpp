
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
    // if parameters are not provided then cannot proceed
    if (!parameters)
      throw runner_exception("cannot trigger service without parameters");

    // generate a goal from parameters if provided
    typename ServiceT::Request request_msg = generate_request(parameters);

    auto result_future = service_client_->async_send_request(request_msg);

    if (event_tracker[execute_tracker_id].on_started)
    {
      event_tracker[execute_tracker_id].on_started(
          update_on_started(event_tracker[execute_tracker_id].on_started_param));
      execute_tracker_id += 1;
    }

    // create a function to call for the result. the future will be returned to the caller and the caller
    // can provide a conversion function to handle the result

    std::function<void(tinyxml2::XMLElement*)> result_callback = [this, result_future](tinyxml2::XMLElement* result) {
      if (rclcpp::spin_until_future_complete(node_, result_future) == rclcpp::FutureReturnCode::SUCCESS)
      {
        // send success event
        if (event_tracker[execute_tracker_id].on_success)
        {
          event_tracker[execute_tracker_id].on_success(
              update_on_success(event_tracker[execute_tracker_id].on_success_param));
          execute_tracker_id += 1;
        }

        result = generate_response(result_future.get());
      }
      else
      {
        RCLCPP_ERROR(node_->get_logger(), "get result call failed");

        // send terminated event
        if (event_tracker[execute_tracker_id].on_failure)
        {
          event_tracker[execute_tracker_id].on_failure(
              update_on_failure(event_tracker[execute_tracker_id].on_failure_param));
          execute_tracker_id += 1;
        }
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
    if (event_tracker[execute_tracker_id].on_stopped)
    {
      event_tracker[execute_tracker_id].on_stopped(
          update_on_stopped(event_tracker[execute_tracker_id].on_stopped_param));
      execute_tracker_id += 1;
    }
  }

protected:
  /**
   * @brief Generate a request from parameters
   *
   * This function is used in conjunction with the trigger function to inject type erased parameters
   * into the typed action
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
   * this method is used in a callback passed to the trigger caller to get type erased result
   * from the service the reponse can be passed by the caller or ignored
   *
   * The pattern needs to be implemented in the derived class
   *
   * @param wrapped_result
   * @return tinyxml2::XMLElement*
   */
  virtual tinyxml2::XMLElement* generate_response(const typename ServiceT::Result::SharedPtr& result) = 0;

  typename rclcpp::Client<ServiceT>::SharedPtr service_client_;
};
}  // namespace capabilities2_runner