#pragma once

#include <thread>
#include <tinyxml2.h>

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>

#include <hri_audio_msgs/action/text_to_speech.hpp>

#include <capabilities2_runner/action_runner.hpp>

namespace capabilities2_runner
{

/**
 * @brief action runner base class
 *
 * Class to run text_to_speech action based capability
 */
class SpeakerRunner : public ActionRunner<hri_audio_msgs::action::TextToSpeech>
{
public:
  SpeakerRunner() : ActionRunner()
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
    init_action(node, run_config, "text_to_speech");
  }

protected:
  /**
   * @brief This generate goal function overrides the generate_goal() function from ActionRunner()
   * 
   * @param parameters XMLElement that contains parameters in the format '<Event name=text_to_speech provider=SpeakerRunner text=$value/>'
   * @return ActionT::Goal the generated goal
   */
  virtual hri_audio_msgs::action::TextToSpeech::Goal generate_goal(tinyxml2::XMLElement* parameters) override
  {
    parameters_ = parameters;

    const char **text;
    parameters_->QueryStringAttribute("text", text);
    std::string tts_text(*text);

    hri_audio_msgs::action::TextToSpeech::Goal goal_msg;

    goal_msg.header.stamp = node_->get_clock()->now();
    goal_msg.tts_text = tts_text;

    return goal_msg;
  }

  /**
   * @brief This generate_result function overrides the generate_result() function from ActionRunner(). Since this does not
   * return a value, return value is nullptr
   * 
   * @param result message from SpeechToText action
   * @return nullptr
   */
  virtual tinyxml2::XMLElement*
  generate_result(const hri_audio_msgs::action::TextToSpeech::Result::SharedPtr& result) override
  {
    return nullptr;
  }

};

}  // namespace capabilities2_runner