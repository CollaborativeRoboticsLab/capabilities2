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
 * occupancy grid map values
 */
class PromptOccupancyRunner : public PromptServiceRunner
{
public:
  PromptOccupancyRunner() : PromptServiceRunner()
  {
  }

  /**
   * @brief generate the prompt used for prompting the occupancy grids.
   *
   * @param parameters tinyXML2 parameters
   * @return std::string
   */
  virtual std::string generate_prompt(tinyxml2::XMLElement* parameters, int id)
  {
    tinyxml2::XMLElement* occupancyElement = parameters->FirstChildElement("OccupancyGrid");

    tinyxml2::XMLPrinter printer;
    occupancyElement->Accept(&printer);

    std::string data(printer.CStr());

    std::string prompt = "The OccupancyGrid of the robot shows the surrounding environment of the robot. The data "
                         "is given as a ros2 nav_msgs::msg::OccupancyGrid of which the content are " +
                         data;

    info_("prompting with : " + prompt, id);

    return prompt;
  }
};

}  // namespace capabilities2_runner
