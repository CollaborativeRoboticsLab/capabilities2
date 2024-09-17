#pragma once

#include <tinyxml2.h>
#include <capabilities2_runner/multi_action_runner.hpp>
#include <hri_audio_msgs/action/text_to_speech.hpp>
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
class VoiceSpeakerRunner : public MultiActionRunner
{
public:
  VoiceSpeakerRunner() : MultiActionRunner()
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

    init_action<hri_audio_msgs::action::TextToSpeech>("text_to_speech");
  }

	/**
	 * @brief stop function to cease functionality and shutdown
	 *
	 */
	virtual void stop() override
	{
    deinit_action<hri_audio_msgs::action::TextToSpeech>("text_to_speech");
	}

  /**
     * @brief trigger the runner
     *
     @param parameters XMLElement that contains parameters in the format '<VoiceSpeaker tts_text='$tts_text'/>'
    */
  virtual void trigger(std::shared_ptr<tinyxml2::XMLElement> parameters = nullptr)
  {
    tinyxml2::XMLElement* parametersElement = parameters->FirstChildElement("VoiceSpeaker");
    const char* tts_text_cstr = nullptr;
    parametersElement->QueryStringAttribute("tts_text", &tts_text_cstr);
    if (tts_text_cstr) {
        tts_text = tts_text_cstr; // Assign the C-style string to your std::string
    }

    hri_audio_msgs::action::TextToSpeech::Goal goal_msg;

    goal_msg.header.statts_textmp = node_->get_clock()->now();
    goal_msg.tts_text=tts_text;

    trigger_action<hri_audio_msgs::action::TextToSpeech>("text_to_speech", goal_msg);

  }

  

protected:
  std::string global_frame_;     /**The global frame of the robot*/
  std::string robot_base_frame_; /**The frame of the robot base*/

  std::string tts_text; /**Coordinate frame parameters*/
};

}  // namespace capabilities2_runner
