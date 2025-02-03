#pragma once
#include <tinyxml2.h>
#include <string>
#include <pluginlib/class_list_macros.hpp>
#include <capabilities2_runner/service_runner.hpp>
#include <capabilities2_msgs/srv/complete_fabric.hpp>

namespace capabilities2_runner
{
/**
 * @brief fabric completion runner
 *
 * This class is a wrapper around the capabilities2 service runner and is used to
 * call on the /capabilities_fabric/set_completion service, providing it as a 
 * capability that notifys the completion of the fabric
 */
class FabricCompletionRunner : public ServiceRunner<capabilities2_msgs::srv::CompleteFabric>
{
public:
  FabricCompletionRunner() : ServiceRunner()
  {
  }

  /**
   * @brief Starter function for starting the service runner
   *
   * @param node shared pointer to the capabilities node. Allows to use ros node related functionalities
   * @param run_config runner configuration loaded from the yaml file
   */
  virtual void start(rclcpp::Node::SharedPtr node, const runner_opts& run_config,
                     std::function<void(Event&)> print) override
  {
    init_service(node, run_config, "/capabilities_fabric/set_completion", print);
  }

protected:
  /**
   * @brief Generate a empty request.
   *
   * This function is used in conjunction with the trigger function to inject type erased parameters
   * into the typed action
   *
   * @param parameters
   * @return prompt_msgs::srv::Prompt::Request the generated request
   */
  virtual typename capabilities2_msgs::srv::CompleteFabric::Request generate_request(tinyxml2::XMLElement* parameters, int id) override
  {
    capabilities2_msgs::srv::CompleteFabric::Request request;
    return request;
  }
};

}  // namespace capabilities2_runner
