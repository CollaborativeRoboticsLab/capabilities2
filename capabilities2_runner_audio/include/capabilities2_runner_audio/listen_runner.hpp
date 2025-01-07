#pragma once

#include <thread>
#include <tinyxml2.h>

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>

#include <hri_audio_msgs/action/speech_to_text.hpp>

#include <capabilities2_runner/action_runner.hpp>

namespace capabilities2_runner
{

/**
 * @brief action runner base class
 *
 * Class to run speech_to_text action based capability
 */
class ListenerRunner : public ActionRunner<hri_audio_msgs::action::SpeechToText>
{
public:
  ListenerRunner() : ActionRunner()
  {
  }

  /**
   * @brief Starter function for starting the action runner
   *
   * @param node shared pointer to the capabilities node. Allows to use ros node related functionalities
   * @param run_config runner configuration loaded from the yaml file
   */
  virtual void start(rclcpp::Node::SharedPtr node, const runner_opts& run_config) override
  {
    init_action(node, run_config, "speech_to_text");
  }

protected:
  /**
   * @brief This generate goal function overrides the generate_goal() function from ActionRunner()
   *
   * @param parameters XMLElement that contains parameters in the format '<Event name=speech_to_text
   * provider=ListenerRunner/>'
   * @return ActionT::Goal the generated goal
   */
  virtual hri_audio_msgs::action::SpeechToText::Goal generate_goal(tinyxml2::XMLElement* parameters) override
  {
    parameters_ = parameters;

    hri_audio_msgs::action::SpeechToText::Goal goal_msg;

    goal_msg.header.stamp = node_->get_clock()->now();

    return goal_msg;
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
    
    // Create the Pose element as a child of the existing parameters element
    tinyxml2::XMLElement* textElement = element->GetDocument()->NewElement("Text");
    element->InsertEndChild(textElement);
    textElement->SetText(result_->stt_text.c_str());

    // Return the updated parameters element with Pose added as string
    std::string result = convert_to_string(element);

    return result;
  };
};

}  // namespace capabilities2_runner
