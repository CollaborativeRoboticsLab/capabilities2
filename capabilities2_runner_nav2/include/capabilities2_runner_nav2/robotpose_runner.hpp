#pragma once

#include <tinyxml2.h>
#include <rclcpp/rclcpp.hpp>

#include "geometry_msgs/msg/transform_stamped.hpp"

#include <capabilities2_runner/topic_runner.hpp>

#include "tf2/exceptions.h"
#include "tf2_ros/transform_listener.h"
#include "tf2_ros/buffer.h"

namespace capabilities2_runner
{

/**
 * @brief odometry runner class
 *
 * Capability Class to grab odometry data
 *
 */
class RobotPoseRunner : public RunnerBase
{
public:
  RobotPoseRunner() : RunnerBase()
  {
  }

  /**
   * @brief Starter function for starting the subscription runner
   *
   * @param node shared pointer to the capabilities node. Allows to use ros node related functionalities
   * @param run_config runner configuration loaded from the yaml file
   */
  virtual void start(rclcpp::Node::SharedPtr node, const runner_opts& run_config,
                     std::function<void(Event&)> runner_publish_func) override
  {
    // initialize the runner base by storing node pointer and run config
    init_base(node, run_config, runner_publish_func);

    tf_buffer_ = std::make_unique<tf2_ros::Buffer>(node->get_clock());
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
  }

  /**
   * @brief Trigger process to be executed.
   *
   * This method utilizes paramters set via the trigger() function
   *
   * @param parameters pointer to tinyxml2::XMLElement that contains parameters
   */
  virtual void execution(int id) override
  {
    const char* from;
    const char* to;

    execute_id += 1;

    // if parameters are not provided then cannot proceed
    if (!parameters_[id])
      throw runner_exception("cannot grab data without parameters");

    // trigger the events related to on_started state
    if (events[execute_id].on_started != "")
    {
      info_("on_started", id, events[execute_id].on_started, EventType::STARTED);
      triggerFunction_(events[execute_id].on_started, update_on_started(events[execute_id].on_started_param));
    }

    info_("Waiting for Transformation.", id);

    parameters_[id]->QueryStringAttribute("from", &from);
    parameters_[id]->QueryStringAttribute("to", &to);

    // Store frame names in variables that will be used to
    // compute transformations
    std::string fromFrame(from);
    std::string toFrame(to);

    // Look up for the transformation between target_frame and turtle2 frames
    // and send velocity commands for turtle2 to reach target_frame
    try
    {
      transform_ = tf_buffer_->lookupTransform(toFrame, fromFrame, tf2::TimePointZero);

      // trigger the events related to on_success state
      if (events[execute_id].on_success != "")
      {
        info_("on_success", id, events[execute_id].on_success, EventType::SUCCEEDED);
        triggerFunction_(events[execute_id].on_success, update_on_success(events[execute_id].on_success_param));
      }
    }
    catch (const tf2::TransformException& ex)
    {
      info_("Could not transform " + toFrame + " to " + fromFrame + " : " + std::string(ex.what()), id);

      // trigger the events related to on_failure state
      if (events[execute_id].on_failure != "")
      {
        info_("on_failure", id, events[execute_id].on_failure, EventType::FAILED);
        triggerFunction_(events[execute_id].on_failure, update_on_failure(events[execute_id].on_failure_param));
      }
    }

    info_("Thread closing.", id);
  }

  /**
   * @brief stop function to cease functionality and shutdown
   *
   */
  virtual void stop() override
  {
    // if the node pointer is empty then throw an error
    // this means that the runner was not started and is being used out of order

    if (!node_)
      throw runner_exception("cannot stop runner that was not started");

    // throw an error if the service client is null
    // this can happen if the runner is not able to find the action resource

    if (!tf_listener_)
      throw runner_exception("cannot stop runner subscriber that was not started");

    // Trigger on_stopped event if defined
    if (events[execute_id].on_stopped != "")
    {
      info_("on_stopped", -1, events[execute_id].on_stopped, EventType::STOPPED);
      triggerFunction_(events[execute_id].on_stopped, update_on_stopped(events[execute_id].on_stopped_param));
    }
  }

protected:
  /**
   * @brief Update on_success event parameters with new data if avaible.
   *
   * This function is used to inject new data into the XMLElement containing
   * parameters related to the on_success trigger event
   *
    <Pose>
        <position x="1.23" y="4.56" z="7.89"/>
        <orientation x="0.12" y="0.34" z="0.56" w="0.78"/>
    </Pose>
   *
   * @param parameters pointer to the XMLElement containing parameters
   * @return pointer to the XMLElement containing updated parameters
   */
  virtual std::string update_on_success(std::string& parameters)
  {
    tinyxml2::XMLElement* element = convert_to_xml(parameters);

    // Create the Pose element as a child of the existing parameters element
    tinyxml2::XMLElement* poseElement = element->GetDocument()->NewElement("Pose");
    element->InsertEndChild(poseElement);

    // Position element with attributes
    tinyxml2::XMLElement* positionElement = element->GetDocument()->NewElement("position");
    positionElement->SetAttribute("x", transform_.transform.translation.x);
    positionElement->SetAttribute("y", transform_.transform.translation.y);
    positionElement->SetAttribute("z", transform_.transform.translation.z);
    poseElement->InsertEndChild(positionElement);

    // Orientation element with attributes
    tinyxml2::XMLElement* orientationElement = element->GetDocument()->NewElement("orientation");
    orientationElement->SetAttribute("x", transform_.transform.rotation.x);
    orientationElement->SetAttribute("y", transform_.transform.rotation.y);
    orientationElement->SetAttribute("z", transform_.transform.rotation.z);
    orientationElement->SetAttribute("w", transform_.transform.rotation.w);
    poseElement->InsertEndChild(orientationElement);

    // Return the updated parameters element with Pose added as string
    std::string result = convert_to_string(element);

    output_("on_success trigger parameter", result);

    return result;
  };

  std::shared_ptr<tf2_ros::TransformListener> tf_listener_{ nullptr };
  std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
  geometry_msgs::msg::TransformStamped transform_;
};
}  // namespace capabilities2_runner