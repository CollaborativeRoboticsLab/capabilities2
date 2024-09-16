#pragma once

#include <tinyxml2.h>
#include <capabilities2_runner/action_runner.hpp>
#include <hri_audio_msgs/action/speech_to_text.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <thread>

namespace capabilities2_runner
{

/**
 * @brief action runner base class
 *
 * Class to run VoiceListener action based capability
 *
 */
class VoiceListenerRunner : public ActionRunner<hri_audio_msgs::action::speech_to_text>
{
public:
  VoiceListenerRunner() : ActionRunner()
  {
  }

  /**
   * @brief Starter function for starting the action runner
   *
   * @param node shared pointer to the capabilities node. Allows to use ros node related functionalities
   * @param run_config runner configuration loaded from the yaml file
   * @param on_started function pointer to trigger at the start of the action client in the runner
   * @param on_terminated function pointer to trigger at the termination of the action client in the runner
   */
  virtual void start(rclcpp::Node::SharedPtr node, const runner_opts& run_config,
                     std::function<void(const std::string&)> on_started = nullptr,
                     std::function<void(const std::string&)> on_terminated = nullptr) override
  {
    init_action(node, run_config, "speech_to_text", on_started, on_terminated);
  }

  /**
   * @brief trigger the runner
   *
   @param parameters XMLElement that contains parameters in the format '<speech_to_text recognized_text='$value'/>'
   */
  virtual void trigger(std::shared_ptr<tinyxml2::XMLElement> parameters = nullptr)
  {
    tinyxml2::XMLElement* parametersElement = parameters->FirstChildElement("speech_to_text");

    hri_audio_msgs::action::SpeechToText::Goal goal_msg;

    


    // launch runner using action client
    action_client_->async_send_goal(goal_msg, send_goal_options_);
  }

  

protected:
  std::string global_frame_;     /**The global frame of the robot*/
  std::string robot_base_frame_; /**The frame of the robot base*/

  double x, y; /**Coordinate frame parameters*/
};

}  // namespace capabilities2_runner
