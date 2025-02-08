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
   * <Pose>
   *     <position>
   *         <x>1.23</x>
   *         <y>4.56</y>
   *         <z>7.89</z>
   *     </position>
   *     <orientation>
   *         <x>0.12</x>
   *         <y>0.34</y>
   *         <z>0.56</z>
   *         <w>0.78</w>
   *     </orientation>
   * </Pose>
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

    // Position element
    tinyxml2::XMLElement* positionElement = element->GetDocument()->NewElement("position");
    poseElement->InsertEndChild(positionElement);

    // Position x, y, z
    tinyxml2::XMLElement* posX = element->GetDocument()->NewElement("x");
    posX->SetText(latest_message_->pose.pose.position.x);  // Set x position value
    positionElement->InsertEndChild(posX);

    tinyxml2::XMLElement* posY = element->GetDocument()->NewElement("y");
    posY->SetText(latest_message_->pose.pose.position.y);  // Set y position value
    positionElement->InsertEndChild(posY);

    tinyxml2::XMLElement* posZ = element->GetDocument()->NewElement("z");
    posZ->SetText(latest_message_->pose.pose.position.z);  // Set z position value
    positionElement->InsertEndChild(posZ);

    // Orientation element
    tinyxml2::XMLElement* orientationElement = element->GetDocument()->NewElement("orientation");
    poseElement->InsertEndChild(orientationElement);

    // Orientation x, y, z, w
    tinyxml2::XMLElement* oriX = element->GetDocument()->NewElement("x");
    oriX->SetText(latest_message_->pose.pose.orientation.x);  // Set orientation x value
    orientationElement->InsertEndChild(oriX);

    tinyxml2::XMLElement* oriY = element->GetDocument()->NewElement("y");
    oriY->SetText(latest_message_->pose.pose.orientation.y);  // Set orientation y value
    orientationElement->InsertEndChild(oriY);

    tinyxml2::XMLElement* oriZ = element->GetDocument()->NewElement("z");
    oriZ->SetText(latest_message_->pose.pose.orientation.z);  // Set orientation z value
    orientationElement->InsertEndChild(oriZ);

    tinyxml2::XMLElement* oriW = element->GetDocument()->NewElement("w");
    oriW->SetText(latest_message_->pose.pose.orientation.w);  // Set orientation w value
    orientationElement->InsertEndChild(oriW);

    // Return the updated parameters element with Pose added as string
    std::string result = convert_to_string(element);

    output_("on_success trigger parameter", result);

    return result;
  };
};
}  // namespace capabilities2_runner