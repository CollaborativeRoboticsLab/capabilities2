#pragma once

#include <tinyxml2.h>
#include <rclcpp/rclcpp.hpp>

#include <nav_msgs/msg/occupancy_grid.hpp>

#include <capabilities2_runner/topic_runner.hpp>

namespace capabilities2_runner
{

/**
 * @brief odometry runner class
 *
 * Capability Class to grab odometry data
 *
 */
class OccupancyGridRunner : public TopicRunner<nav_msgs::msg::OccupancyGrid>
{
public:
  OccupancyGridRunner() : TopicRunner()
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
    init_subscriber(node, run_config, "map");
  }

protected:
  /**
   * @brief This generate goal function overrides the generate_message() function from TopicRunner()
   * @param result of the type geometry_msgs::msg::PoseWithCovarianceStamped

   * @return tinyxml2::XMLElement* of the robot's pose
   */
  virtual tinyxml2::XMLElement*
  generate_message(typename nav_msgs::msg::OccupancyGrid::SharedPtr& result) override
  {
    tinyxml2::XMLDocument* document = new tinyxml2::XMLDocument();
    tinyxml2::XMLNode* robot_pose = document->InsertEndChild(document->NewElement("OccupancyGrid"));

    tinyxml2::XMLElement* pose_element = document->NewElement("Info");

    pose_element->SetAttribute("resolution", result->info.resolution);
    pose_element->SetAttribute("width", result->info.width);
    pose_element->SetAttribute("height", result->info.height);
    pose_element->SetAttribute("origin.position.x", result->info.origin.position.x);
    pose_element->SetAttribute("origin.position.y", result->info.origin.position.y);
    pose_element->SetAttribute("origin.position.z", result->info.origin.position.z);
    pose_element->SetAttribute("origin.orientation.x", result->info.origin.orientation.x);
    pose_element->SetAttribute("origin.orientation.y", result->info.origin.orientation.y);
    pose_element->SetAttribute("origin.orientation.z", result->info.origin.orientation.z);
    pose_element->SetAttribute("origin.orientation.x", result->info.origin.orientation.w);

    tinyxml2::XMLElement* data_element = document->NewElement("Data");

    std::string data_text = "";

    for (const auto& data : result->data)
      data_text += std::to_string(data) + " ";

    data_element->SetAttribute("data", data_text.c_str());

    robot_pose->InsertFirstChild(pose_element);
    robot_pose->InsertEndChild(data_element);

    return document->FirstChildElement("OccupancyGrid");
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
    std::string data_text = "";

    for (const auto& data : latest_message_->data)
      data_text += std::to_string(data) + " ";

    parameters->SetAttribute("type", "nav_msgs/OccupancyGrid");

    parameters->SetAttribute("info.resolution", latest_message_->info.resolution);
    parameters->SetAttribute("info.width", latest_message_->info.width);
    parameters->SetAttribute("info.height", latest_message_->info.height);
    parameters->SetAttribute("info.origin.position.x", latest_message_->info.origin.position.x);
    parameters->SetAttribute("info.origin.position.y", latest_message_->info.origin.position.y);
    parameters->SetAttribute("info.origin.position.z", latest_message_->info.origin.position.z);
    parameters->SetAttribute("info.origin.orientation.x", latest_message_->info.origin.orientation.x);
    parameters->SetAttribute("info.origin.orientation.y", latest_message_->info.origin.orientation.y);
    parameters->SetAttribute("info.origin.orientation.z", latest_message_->info.origin.orientation.z);
    parameters->SetAttribute("info.origin.orientation.x", latest_message_->info.origin.orientation.w);
    parameters->SetAttribute("data", data_text.c_str());

    return parameters;
  };
};
}  // namespace capabilities2_runner