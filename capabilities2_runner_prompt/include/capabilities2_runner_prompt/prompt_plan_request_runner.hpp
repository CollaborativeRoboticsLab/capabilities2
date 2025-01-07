#pragma once
#include <tinyxml2.h>
#include <string>
#include <pluginlib/class_list_macros.hpp>
#include <capabilities2_runner/service_runner.hpp>
#include <prompt_msgs/msg/model_option.hpp>
#include <prompt_msgs/srv/prompt.hpp>

namespace capabilities2_runner
{
/**
 * @brief prompt capability runner
 *
 * This class is a wrapper around the capabilities2 service runner and is used to
 * call on the prompt_tools/prompt service, providing it as a capability that prompts
 * capabilitie plans values
 */
class PromptPlanRequestRunner : public ServiceRunner<prompt_msgs::srv::Prompt>
{
public:
  PromptPlanRequestRunner() : ServiceRunner()
  {
  }

  /**
   * @brief Starter function for starting the service runner
   *
   * @param node shared pointer to the capabilities node. Allows to use ros node related functionalities
   * @param run_config runner configuration loaded from the yaml file
   */
  virtual void start(rclcpp::Node::SharedPtr node, const runner_opts& run_config) override
  {
    init_service(node, run_config, "prompt");
  }

protected:
  /**
   * @brief Generate a request from parameters given.
   *
   * This function is used in conjunction with the trigger function to inject type erased parameters
   * into the typed action
   *
   * A pattern needs to be implemented in the derived class
   *
   * @param parameters
   * @return prompt_msgs::srv::Prompt::Request the generated request
   */
  virtual typename prompt_msgs::srv::Prompt::Request generate_request(tinyxml2::XMLElement* parameters) override
  {
    parameters_ = parameters;

    bool replan;
    parameters->QueryBoolAttribute("replan", &replan);

    prompt_msgs::srv::Prompt::Request request;

    if (!replan)
    {
      request.prompt.prompt = "Build a xml plan based on the availbale capabilities to acheive mentioned task. Return "
                              "only the xml plan without explanations or comments";
    }
    else
    {
      tinyxml2::XMLElement* failedElements = parameters->FirstChildElement("FailedElements");

      request.prompt.prompt = "Rebuild the xml plan based on the availbale capabilities to acheive mentioned task. "
                              "Just give the xml plan without explanations or comments. These XML elements had "
                              "incompatibilities. " +
                              std::string(failedElements->GetText()) + "Recorrect them as well";
    }

    prompt_msgs::msg::ModelOption modelOption1;
    modelOption1.key = "model";
    modelOption1.value = "llama3.1:8b";

    request.prompt.options.push_back(modelOption1);

    prompt_msgs::msg::ModelOption modelOption2;
    modelOption2.key = "stream";
    modelOption2.value = "false";
    modelOption2.type = prompt_msgs::msg::ModelOption::BOOL_TYPE;

    request.prompt.options.push_back(modelOption2);

    return request;
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
