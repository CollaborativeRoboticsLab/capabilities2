#pragma once
#include <tinyxml2.h>
#include <string>
#include <pluginlib/class_list_macros.hpp>
#include <capabilities2_runner_prompt/prompt_service_runner.hpp>

namespace capabilities2_runner
{
/**
 * @brief prompt pose runner
 *
 * This class is a wrapper around the capabilities2 service runner and is used to pass
 * data to prompt_tools/prompt service, providing it as a capability that prompts
 * robot pose values
 */
class PromptPoseRunner : public PromptServiceRunner
{
public:
  PromptPoseRunner() : PromptServiceRunner()
  {
  }

  /**
   * @brief generate the prompt used for prompting the capabilities.
   *
   * @param parameters tinyXML2 parameters
   * @return std::string
   */
  virtual void generate_prompt(tinyxml2::XMLElement* parameters, int id, std::string& prompt, bool& flush) override
  {
    tinyxml2::XMLElement* poseElement = parameters->FirstChildElement("Pose");

    tinyxml2::XMLPrinter printer;
    poseElement->Accept(&printer);

    std::string data(printer.CStr());

    prompt = "The position of the robot is given as a standard ros2 geometry_msgs::msg::Pose of which the content "
             "are " + data;
    flush  = false;

    info_("prompting with : " + prompt, id);
  }
};

}  // namespace capabilities2_runner
