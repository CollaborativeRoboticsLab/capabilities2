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
 * capabilitie plans values
 */
class PromptPlanRunner : public PromptServiceRunner
{
public:
  PromptPlanRunner() : PromptServiceRunner()
  {
  }

  /**
   * @brief generate the prompt used for prompting the occupancy grids.
   *
   * @param parameters tinyXML2 parameters
   * @return std::string
   */
  virtual void generate_prompt(tinyxml2::XMLElement* parameters, int id, std::string& prompt, bool& flush) override
  {
    bool replan;
    const char* task;
    std::string taskString;

    parameters->QueryBoolAttribute("replan", &replan);
    parameters->QueryStringAttribute("task", &task);

    if (task)
      taskString = task;
    else
      taskString = "";

    if (!replan)
    {
      prompt = "Build a xml plan based on the availbale capabilities to acheive mentioned task of " + taskString +
               ". Return only the xml plan without explanations or comments";

      flush = true;
    }
    else
    {
      tinyxml2::XMLElement* failedElements = parameters->FirstChildElement("FailedElements");

      prompt = "Rebuild the xml plan based on the availbale capabilities to acheive mentioned task of " + taskString +
               ". Just give the xml plan without explanations or comments. These XML  "
               "elements had incompatibilities. " +
               std::string(failedElements->GetText()) + "Recorrect them as well";
      flush = true;
    }

    info_("prompting with : " + prompt, id);
  }

  /**
   * @brief Update on_success event parameters with new data if avaible.
   *
   * This function is used to inject new data into the XMLElement containing
   * parameters related to the on_success trigger event
   *
   * A pattern needs to be implemented in the derived class
   *
   * @param parameters pointer to the XMLElement containing parameters
   * @return pointer to the XMLElement containing updated parameters
   */
  virtual std::string update_on_success(std::string& parameters)
  {
    tinyxml2::XMLElement* element = convert_to_xml(parameters);

    std::string document_string = response_->response.response;

    // Create the plan element as a child of the existing parameters element
    tinyxml2::XMLElement* textElement = element->GetDocument()->NewElement("ReceievdPlan");
    element->InsertEndChild(textElement);
    textElement->SetText(document_string.c_str());

    // Return the updated parameters element with Pose added
    std::string result = convert_to_string(element);

    return result;
  };
};

}  // namespace capabilities2_runner
