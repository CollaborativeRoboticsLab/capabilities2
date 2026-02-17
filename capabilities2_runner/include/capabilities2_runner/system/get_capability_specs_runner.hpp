#pragma once

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
class GetCapabilitySpecsRunner : public ServiceRunner<capabilities2_msgs::srv::GetCapabilitySpecs>
{
public:
  GetCapabilitySpecsRunner() : ServiceRunner()
  {
  }

  /**
   * @brief Starter function for starting the action runner
   *
   * @param node shared pointer to the capabilities node. Allows to use ros node related functionalities
   * @param run_config runner configuration loaded from the yaml file
   * @param bond_id unique identifier for the group of connections associated with this runner trigger event
   */
  virtual void start(rclcpp::Node::SharedPtr node, const runner_opts& run_config, const std::string& bond_id) override
  {
    init_service(node, run_config, "/capabilities/get_capability_specs");

    // emit start event
    emit_started(bond_id, "", param_on_started());
  }

protected:
  /**
   * @brief This generate goal function overrides the generate_goal() function from ActionRunner()
   * @param parameters XMLElement that contains parameters in the format
   '<Event name=follow_waypoints provider=WaypointRunner x='$value' y='$value' />'
   * @return ActionT::Goal the generated goal
   */
  virtual capabilities2_msgs::srv::GetCapabilitySpecs::Request
  generate_request(capabilities2_events::EventParameters& parameters) override
  {
    capabilities2_msgs::srv::GetCapabilitySpecs::Request request;
    return request;
  }

  /**
   * @brief This function overrides the param_on_success() function from RunnerBase to provide specific implementation for the GetCapabilitySpecsRunner
   * 
   * @param parameters EventParameters containing parameters for the trigger event
   * @return EventParameters updated parameters for on_success event
   */
  virtual capabilities2_events::EventParameters param_on_success()
  {
    // Create an Event Parameter with CapabilitySpecs content
    std::vector<std::string> capability_spec_strings;
    for (const auto& spec : response_->capability_specs)
    {
      std::string spec_string = "package: " + spec.package + ", type: " + spec.type +
                                ", default_provider: " + spec.default_provider + ", content: " + spec.content;
      capability_spec_strings.push_back(spec_string);
    }

    // Create EventParameters and set the capability specs as a parameter for the on_success event
    capabilities2_events::EventParameters parameters;

    // Set the capability specs as a parameter for the on_success event
    parameters.set_value("CapabilitySpecs", capability_spec_strings, capabilities2_events::OptionType::VECTOR_STRING);
    return parameters;
  }
};

}  // namespace capabilities2_runner
