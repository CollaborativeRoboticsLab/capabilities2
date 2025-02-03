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
 * text values
 */
class PromptTextRunner : public ServiceRunner<prompt_msgs::srv::Prompt>
{
public:
  PromptTextRunner() : ServiceRunner()
  {
  }

  /**
   * @brief Starter function for starting the service runner
   *
   * @param node shared pointer to the capabilities node. Allows to use ros node related functionalities
   * @param run_config runner configuration loaded from the yaml file
   */
  virtual void start(rclcpp::Node::SharedPtr node, const runner_opts& run_config,
                     std::function<void(Event&)> print) override
  {
    init_service(node, run_config, "prompt", print);
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
  virtual typename prompt_msgs::srv::Prompt::Request generate_request(tinyxml2::XMLElement* parameters, int id) override
  {
    tinyxml2::XMLElement* textElement = parameters->FirstChildElement("Text");

    tinyxml2::XMLPrinter printer;
    textElement->Accept(&printer);

    std::string data(printer.CStr());

    prompt_msgs::srv::Prompt::Request request;

    request.prompt.prompt = "The response was " + data;

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
};

}  // namespace capabilities2_runner
