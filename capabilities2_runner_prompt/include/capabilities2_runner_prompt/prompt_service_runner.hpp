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
 * @brief prompt service runner
 *
 * This class is a base class and a wrapper around the capabilities2 service runner
 * and is used to call on the prompt_tools/prompt service, providing it as a capability
 * that prompts capabilitie plans values
 */
class PromptServiceRunner : public ServiceRunner<prompt_msgs::srv::Prompt>
{
public:
  PromptServiceRunner() : ServiceRunner()
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
    init_service(node, run_config, "/prompt_bridge/prompt", print);
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
  virtual typename prompt_msgs::srv::Prompt::Request generate_request(tinyxml2::XMLElement* parameters, int id) override
  {
    prompt_msgs::srv::Prompt::Request request;

    prompt_msgs::msg::ModelOption modelOption1;
    modelOption1.key = "model";
    modelOption1.value = "llama3.2";

    request.prompt.options.push_back(modelOption1);

    prompt_msgs::msg::ModelOption modelOption2;
    modelOption2.key = "stream";
    modelOption2.value = false;
    modelOption2.type = prompt_msgs::msg::ModelOption::BOOL_TYPE;

    request.prompt.options.push_back(modelOption2);

    generate_prompt(parameters, id, request.prompt.prompt, request.prompt.flush);
    
    return request;
  }

  /**
   * @brief generate the prompt used for prompting the data.
   *
   * @param parameters tinyXML2 parameters
   * @return std::string
   */
  virtual void generate_prompt(tinyxml2::XMLElement* parameters, int id, std::string& prompt, bool& flush) = 0;

  virtual void process_response(typename prompt_msgs::srv::Prompt::Response::SharedPtr response, int id)
  {
    info_("response received : " + response->response.response, id);
  }
};

}  // namespace capabilities2_runner
