#pragma once
#include <tinyxml2.h>
#include <string>
#include <pluginlib/class_list_macros.hpp>
#include <capabilities2_runner_prompt/prompt_service_runner.hpp>

namespace capabilities2_runner
{
/**
 * @brief prompt capability runner
 *
 * This class is a wrapper around the capabilities2 service runner and is used to
 * call on the prompt_tools/prompt service, providing it as a capability that prompts
 * text values
 */
class PromptTextRunner : public PromptServiceRunner
{
public:
  PromptTextRunner() : PromptServiceRunner()
  {
  }

  /**
   * @brief generate the prompt used for prompting the text.
   *
   * @param parameters tinyXML2 parameters
   * @return std::string
   */
  virtual std::string generate_prompt(tinyxml2::XMLElement* parameters)
  {
    tinyxml2::XMLElement* textElement = parameters->FirstChildElement("Text");

    tinyxml2::XMLPrinter printer;
    textElement->Accept(&printer);

    std::string data(printer.CStr());

    std::string prompt = "The response was " + data;

    return prompt;
  }
};

}  // namespace capabilities2_runner
