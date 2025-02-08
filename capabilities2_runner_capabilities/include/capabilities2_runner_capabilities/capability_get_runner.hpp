#pragma once

#include <thread>

#include <tinyxml2.h>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>

#include <capabilities2_runner/service_runner.hpp>
#include <capabilities2_msgs/srv/get_capability_specs.hpp>

namespace capabilities2_runner
{

/**
 * @brief Executor runner class
 *
 * Class to run capabilities2 executor action based capability
 *
 */
class CapabilityGetRunner : public ServiceRunner<capabilities2_msgs::srv::GetCapabilitySpecs>
{
public:
  CapabilityGetRunner() : ServiceRunner()
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
    init_service(node, run_config, "/capabilities/get_capability_specs", print);
  }

protected:
  /**
   * @brief This generate goal function overrides the generate_goal() function from ActionRunner()
   * @param parameters XMLElement that contains parameters in the format
   '<Event name=follow_waypoints provider=WaypointRunner x='$value' y='$value' />'
   * @return ActionT::Goal the generated goal
   */
  virtual capabilities2_msgs::srv::GetCapabilitySpecs::Request generate_request(tinyxml2::XMLElement* parameters,
                                                                                int id) override
  {
    capabilities2_msgs::srv::GetCapabilitySpecs::Request request;
    return request;
  }

  virtual std::string update_on_success(std::string& parameters)
  {
    tinyxml2::XMLElement* element = convert_to_xml(parameters);

    // Create the OccupancyGrid element as a child of the existing parameters element
    tinyxml2::XMLElement* capabilitySpecsElement = element->GetDocument()->NewElement("CapabilitySpecs");
    element->InsertEndChild(capabilitySpecsElement);

    for (const auto& spec : response_->capability_specs)
    {
      // Create a <CapabilitySpec> element
      tinyxml2::XMLElement* specElement = element->GetDocument()->NewElement("CapabilitySpec");

      // Set attributes
      specElement->SetAttribute("package", spec.package.c_str());
      specElement->SetAttribute("type", spec.type.c_str());

      if (!spec.default_provider.empty())
      {
        specElement->SetAttribute("default_provider", spec.default_provider.c_str());
      }

      // Add content as a child element inside <CapabilitySpec>
      tinyxml2::XMLElement* contentElement = element->GetDocument()->NewElement("content");
      contentElement->SetText(spec.content.c_str());
      specElement->InsertEndChild(contentElement);

      // Append <CapabilitySpec> to <CapabilitySpecs>
      capabilitySpecsElement->InsertEndChild(specElement);
    }

    // Convert updated XML back to string and return
    std::string result = convert_to_string(element);
    return result;
  }
};

}  // namespace capabilities2_runner
