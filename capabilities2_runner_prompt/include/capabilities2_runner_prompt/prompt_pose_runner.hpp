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
 * @brief prompt pose runner
 *
 * This class is a wrapper around the capabilities2 service runner and is used to pass
 * data to prompt_tools/prompt service, providing it as a capability that prompts
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
  virtual void start(rclcpp::Node::SharedPtr node, const runner_opts& run_config,
                     std::function<void(Event&)> print) override
  {
    init_service(node, run_config, "prompt", print);
  }

  /**
   * @brief Generate request for trnasforming data for PrompPoseRunner
   * 
   * @param parameters parameter values
   * @param id thread id
   * @return prompt_msgs::srv::Prompt::Request 
   */
  virtual prompt_msgs::srv::Prompt::Request generate_request(tinyxml2::XMLElement* parameters, int id) override
  {
    tinyxml2::XMLElement* poseElement = parameters->FirstChildElement("Pose");

    tinyxml2::XMLPrinter printer;
    poseElement->Accept(&printer);

    std::string data(printer.CStr());

    prompt_msgs::srv::Prompt::Request request;

    request.prompt.prompt = "The position of the robot is given as a standard ros2 geometry_msgs::msg::Pose of which "
                            "the content are " + data;

    prompt_msgs::msg::ModelOption modelOption1;
    modelOption1.key = "model";
    modelOption1.value = "llama3.1:8b";

    request.prompt.options.push_back(modelOption1);

    prompt_msgs::msg::ModelOption modelOption2;
    modelOption2.key = "stream";
    modelOption2.value = "false";
    modelOption2.type = prompt_msgs::msg::ModelOption::BOOL_TYPE;

    request.prompt.options.push_back(modelOption2);

    info_("prompting with : " + request.prompt.prompt, id);

    return request;
  }
};

}  // namespace capabilities2_runner
