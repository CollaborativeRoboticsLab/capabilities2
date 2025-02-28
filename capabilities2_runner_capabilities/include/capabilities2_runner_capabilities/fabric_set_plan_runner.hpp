#pragma once

#include <thread>

#include <tinyxml2.h>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>

#include <capabilities2_runner/service_runner.hpp>
#include <capabilities2_msgs/srv/set_fabric_plan.hpp>

namespace capabilities2_runner
{

/**
 * @brief Executor runner class
 *
 * Class to run capabilities2 executor action based capability
 *
 */
class FabricSetPlanRunner : public ServiceRunner<capabilities2_msgs::srv::SetFabricPlan>
{
public:
  FabricSetPlanRunner() : ServiceRunner()
  {
  }

  /**
   * @brief Starter function for starting the action runner
   *
   * @param node shared pointer to the capabilities node. Allows to use ros node related functionalities
   * @param run_config runner configuration loaded from the yaml file
   */
  virtual void start(rclcpp::Node::SharedPtr node, const runner_opts& run_config,
                     std::function<void(Event&)> print) override
  {
    init_service(node, run_config, "/capabilities_fabric/set_plan", print);
  }

protected:
  /**
   * @brief This generate goal function overrides the generate_goal() function from ActionRunner()
   * @param parameters XMLElement that contains parameters in the format
   * @return ActionT::Goal the generated goal
   */
  virtual capabilities2_msgs::srv::SetFabricPlan::Request generate_request(tinyxml2::XMLElement* parameters, int id) override
  {
    tinyxml2::XMLElement* planElement = parameters->FirstChildElement("ReceievdPlan");

    capabilities2_msgs::srv::SetFabricPlan::Request request;

    // Check if the element was found and has text content
    if (planElement && planElement->GetText())
    {
      request.plan = std::string(planElement->GetText());
    }

    return request;
  }
};

}  // namespace capabilities2_runner
