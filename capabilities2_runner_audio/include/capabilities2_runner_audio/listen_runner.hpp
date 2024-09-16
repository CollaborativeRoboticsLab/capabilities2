#pragma once

#include <tinyxml2.h>
#include <capabilities2_runner/multi_action_runner.hpp>
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
class VoiceListenerRunner : public MultiActionRunner
{
public:
  VoiceListenerRunner() : MultiActionRunner()
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
                     std::function<void(const std::string&)> on_terminated = nullptr,
                     std::function<void(const std::string&)> on_stopped = nullptr) override
  {
    init_runner(node, run_config, on_started, on_terminated, on_stopped);

    init_action<hri_audio_msgs::action::SpeechToText>("speech_to_text");
  }

	/**
	 * @brief stop function to cease functionality and shutdown
	 *
	 */
	virtual void stop() override
	{
    deinit_action<hri_audio_msgs::action::SpeechToText>("speech_to_text");
	}

  /**
   * @brief trigger the runner
   *
   @param parameters XMLElement
   */
  virtual void trigger(std::shared_ptr<tinyxml2::XMLElement> parameters = nullptr)
  {
    hri_audio_msgs::action::SpeechToText::Goal goal_msg;

    trigger_action<hri_audio_msgs::action::SpeechToText>("speech_to_text", goal_msg);
  }

  

protected:
  std::string global_frame_;     /**The global frame of the robot*/
  std::string robot_base_frame_; /**The frame of the robot base*/

  double x, y; /**Coordinate frame parameters*/
};

}  // namespace capabilities2_runner
