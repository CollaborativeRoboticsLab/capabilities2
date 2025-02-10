#pragma once
#include <tinyxml2.h>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <capabilities2_runner_prompt/prompt_service_runner.hpp>

namespace capabilities2_runner
{

/**
 * @brief prompt capabilities runner
 *
 * This class is a wrapper around the capabilities2 service runner and is used to pass
 * data to prompt_tools/prompt service, providing it as a capability that prompts
 * robot capabilities.
 */
class PromptCapabilityRunner : public PromptServiceRunner
{
public:
  PromptCapabilityRunner() : PromptServiceRunner()
  {
  }

  /**
   * @brief generate the prompt used for prompting the capabilities.
   *
   * @param parameters tinyXML2 parameters
   * @return std::string
   */
  virtual std::string generate_prompt(tinyxml2::XMLElement* parameters)
  {
    tinyxml2::XMLElement* capabilitySpecsElement = parameters->FirstChildElement("CapabilitySpecs");

    tinyxml2::XMLPrinter printer;
    capabilitySpecsElement->Accept(&printer);

    std::string data(printer.CStr());

    std::string prompt = "The capabilities of the robot are given as follows" + data;

    return prompt;
  }
};

}  // namespace capabilities2_runner
