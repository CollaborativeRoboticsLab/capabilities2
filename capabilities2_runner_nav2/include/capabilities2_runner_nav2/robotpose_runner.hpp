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
    tinyxml2::XMLDocument* document = new tinyxml2::XMLDocument();
    tinyxml2::XMLNode* robot_pose = document->InsertEndChild(document->NewElement("RobotPose"));

    tinyxml2::XMLElement* pose_element = document->NewElement("Pose");

    pose_element->SetAttribute("x", result->pose.pose.position.x);
    pose_element->SetAttribute("y", result->pose.pose.position.y);
    pose_element->SetAttribute("z", result->pose.pose.position.z);

    tinyxml2::XMLElement* orie_element = document->NewElement("Orientation");

    orie_element->SetAttribute("x", result->pose.pose.orientation.x);
    orie_element->SetAttribute("y", result->pose.pose.orientation.y);
    orie_element->SetAttribute("z", result->pose.pose.orientation.z);
    orie_element->SetAttribute("w", result->pose.pose.orientation.w);

    robot_pose->InsertFirstChild(pose_element);
    robot_pose->InsertEndChild(orie_element);

    return document->FirstChildElement("RobotPose");
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
    parameters->SetAttribute("type", "geometry_msgs/msgs/Pose");
    parameters->SetAttribute("position.x", latest_message_->pose.pose.position.x);
    parameters->SetAttribute("position.y", latest_message_->pose.pose.position.y);
    parameters->SetAttribute("position.z", latest_message_->pose.pose.position.z);
    parameters->SetAttribute("orientation.x", latest_message_->pose.pose.orientation.x);
    parameters->SetAttribute("orientation.y", latest_message_->pose.pose.orientation.y);
    parameters->SetAttribute("orientation.z", latest_message_->pose.pose.orientation.z);
    parameters->SetAttribute("orientation.x", latest_message_->pose.pose.orientation.w);

    return parameters;
  };
};
}  // namespace capabilities2_runner