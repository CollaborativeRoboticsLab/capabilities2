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
 * robot pose values
 */
class PromptPoseRunner : public ServiceRunner<prompt_msgs::srv::Prompt>
{
public:
  PromptPoseRunner() : ServiceRunner()
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

    tinyxml2::XMLElement* poseElement = parameters->FirstChildElement("Pose");

    tinyxml2::XMLPrinter printer;
    poseElement->Accept(&printer);

    std::string data(printer.CStr());

    prompt_msgs::srv::Prompt::Request request;

    request.prompt.prompt = "The position of the robot is given as a standard ros2 geometry message of which the content are " + data;

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
   * @brief generate a typed erased response
   *
   * this method is used in a callback passed to the trigger caller to get type erased result
   * from the service the reponse can be passed by the caller or ignored
   *
   * The pattern needs to be implemented in the derived class
   *
   * @param wrapped_result
   * @return tinyxml2::XMLElement*
   */
  virtual tinyxml2::XMLElement*
  generate_response(const typename prompt_msgs::srv::Prompt::Response::SharedPtr& result) const override
  {
    return nullptr;
  }
};

}  // namespace capabilities2_runner
