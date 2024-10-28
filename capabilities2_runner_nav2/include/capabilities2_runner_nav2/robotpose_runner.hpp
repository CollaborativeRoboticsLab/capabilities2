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
  virtual void start(rclcpp::Node::SharedPtr node, const runner_opts& run_config) override
  {
    init_subscriber(node, run_config, "pose");
  }

protected:
  /**
   * @brief This generate goal function overrides the generate_message() function from TopicRunner()
   * @param result of the type geometry_msgs::msg::PoseWithCovarianceStamped

   * @return tinyxml2::XMLElement* of the robot's pose
   */
  virtual tinyxml2::XMLElement*
  generate_message(typename geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr& result) override
  {
    tinyxml2::XMLDocument doc;

    // Root element
    tinyxml2::XMLElement* poseElement = doc.NewElement("Pose");
    doc.InsertFirstChild(poseElement);

    // Position element
    tinyxml2::XMLElement* positionElement = doc.NewElement("position");
    poseElement->InsertEndChild(positionElement);

    // Position x, y, z
    tinyxml2::XMLElement* posX = doc.NewElement("x");
    posX->SetText(result->pose.pose.position.x);  // Set x position value
    positionElement->InsertEndChild(posX);

    tinyxml2::XMLElement* posY = doc.NewElement("y");
    posY->SetText(result->pose.pose.position.y);  // Set y position value
    positionElement->InsertEndChild(posY);

    tinyxml2::XMLElement* posZ = doc.NewElement("z");
    posZ->SetText(result->pose.pose.position.z);  // Set z position value
    positionElement->InsertEndChild(posZ);

    // Orientation element
    tinyxml2::XMLElement* orientationElement = doc.NewElement("orientation");
    poseElement->InsertEndChild(orientationElement);

    // Orientation x, y, z, w
    tinyxml2::XMLElement* oriX = doc.NewElement("x");
    oriX->SetText(result->pose.pose.orientation.x);  // Set orientation x value
    orientationElement->InsertEndChild(oriX);

    tinyxml2::XMLElement* oriY = doc.NewElement("y");
    oriY->SetText(result->pose.pose.orientation.y);  // Set orientation y value
    orientationElement->InsertEndChild(oriY);

    tinyxml2::XMLElement* oriZ = doc.NewElement("z");
    oriZ->SetText(result->pose.pose.orientation.z);  // Set orientation z value
    orientationElement->InsertEndChild(oriZ);

    tinyxml2::XMLElement* oriW = doc.NewElement("w");
    oriW->SetText(result->pose.pose.orientation.w);  // Set orientation w value
    orientationElement->InsertEndChild(oriW);

    return doc.FirstChildElement("Pose");
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
  virtual tinyxml2::XMLElement* update_on_success(tinyxml2::XMLElement* parameters)
  {
    // Create the Pose element as a child of the existing parameters element
    tinyxml2::XMLElement* poseElement = parameters->GetDocument()->NewElement("Pose");
    parameters->InsertEndChild(poseElement);

    // Position element
    tinyxml2::XMLElement* positionElement = parameters->GetDocument()->NewElement("position");
    poseElement->InsertEndChild(positionElement);

    // Position x, y, z
    tinyxml2::XMLElement* posX = parameters->GetDocument()->NewElement("x");
    posX->SetText(latest_message_->pose.pose.position.x);  // Set x position value
    positionElement->InsertEndChild(posX);

    tinyxml2::XMLElement* posY = parameters->GetDocument()->NewElement("y");
    posY->SetText(latest_message_->pose.pose.position.y);  // Set y position value
    positionElement->InsertEndChild(posY);

    tinyxml2::XMLElement* posZ = parameters->GetDocument()->NewElement("z");
    posZ->SetText(latest_message_->pose.pose.position.z);  // Set z position value
    positionElement->InsertEndChild(posZ);

    // Orientation element
    tinyxml2::XMLElement* orientationElement = parameters->GetDocument()->NewElement("orientation");
    poseElement->InsertEndChild(orientationElement);

    // Orientation x, y, z, w
    tinyxml2::XMLElement* oriX = parameters->GetDocument()->NewElement("x");
    oriX->SetText(latest_message_->pose.pose.orientation.x);  // Set orientation x value
    orientationElement->InsertEndChild(oriX);

    tinyxml2::XMLElement* oriY = parameters->GetDocument()->NewElement("y");
    oriY->SetText(latest_message_->pose.pose.orientation.y);  // Set orientation y value
    orientationElement->InsertEndChild(oriY);

    tinyxml2::XMLElement* oriZ = parameters->GetDocument()->NewElement("z");
    oriZ->SetText(latest_message_->pose.pose.orientation.z);  // Set orientation z value
    orientationElement->InsertEndChild(oriZ);

    tinyxml2::XMLElement* oriW = parameters->GetDocument()->NewElement("w");
    oriW->SetText(latest_message_->pose.pose.orientation.w);  // Set orientation w value
    orientationElement->InsertEndChild(oriW);

    // Return the updated parameters element with Pose added
    return parameters;
  };
};
}  // namespace capabilities2_runner