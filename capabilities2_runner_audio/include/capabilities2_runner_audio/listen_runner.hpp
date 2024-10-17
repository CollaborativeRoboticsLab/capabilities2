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
   * @param parameters XMLElement that contains parameters in the format '<Event name=speech_to_text provider=ListenerRunner/>'
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
   * @brief This generate_result function overrides the generate_result() function from ActionRunner(). Since
   * 
   * @param result message from SpeechToText action
   * @return tinyxml2::XMLElement* in the format '<Event name=speech_to_text provider=ListenerRunner text=$value/>'
   */
  virtual tinyxml2::XMLElement*
  generate_result(const hri_audio_msgs::action::SpeechToText::Result::SharedPtr& result) override
  {
    std::string text = result->stt_text;

    parameters_->SetAttribute( "text", text.c_str() );

    return parameters_;
  }

};

}  // namespace capabilities2_runner
