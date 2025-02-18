#pragma once

#include <tinyxml2.h>
#include <rclcpp/rclcpp.hpp>

#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>

#include <capabilities2_runner/topic_runner.hpp>

namespace capabilities2_runner
{

/**
 * @brief odometry runner class
 *
 * Capability Class to grab odometry data
 *
 */
class RobotPoseRunner : public TopicRunner<geometry_msgs::msg::PoseWithCovarianceStamped>
{
public:
  RobotPoseRunner() : TopicRunner()
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
    init_subscriber(node, run_config, "pose", runner_publish_func);
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
    positionElement->SetAttribute("x", latest_message_->pose.pose.position.x);
    positionElement->SetAttribute("y", latest_message_->pose.pose.position.y);
    positionElement->SetAttribute("z", latest_message_->pose.pose.position.z);
    poseElement->InsertEndChild(positionElement);

    // Orientation element with attributes
    tinyxml2::XMLElement* orientationElement = element->GetDocument()->NewElement("orientation");
    orientationElement->SetAttribute("x", latest_message_->pose.pose.orientation.x);
    orientationElement->SetAttribute("y", latest_message_->pose.pose.orientation.y);
    orientationElement->SetAttribute("z", latest_message_->pose.pose.orientation.z);
    orientationElement->SetAttribute("w", latest_message_->pose.pose.orientation.w);
    poseElement->InsertEndChild(orientationElement);

    // Return the updated parameters element with Pose added as string
    std::string result = convert_to_string(element);

    output_("on_success trigger parameter", result);

    return result;
  };
};
}  // namespace capabilities2_runner