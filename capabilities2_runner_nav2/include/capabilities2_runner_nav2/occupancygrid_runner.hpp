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
 * Capability Class to grab occupancy grid data
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
  virtual void start(rclcpp::Node::SharedPtr node, const runner_opts& run_config,
                     std::function<void(Event&)> print) override
  {
    init_subscriber(node, run_config, "/map", print);
  }

protected:
  /**
   * @brief Update on_success event parameters with new data from an OccupancyGrid message if avaible.
   *
   * This function is used to inject new data into the XMLElement containing
   * parameters related to the on_success trigger event
   *
    <OccupancyGrid>
        <header frame_id="map">
            <stamp secs="1708369200" nsecs="123456789"/>
        </header>
        <info resolution="0.05" width="100" height="100">
            <origin>
                <position x="1.0" y="2.0" z="0.0"/>
                <orientation x="0.0" y="0.0" z="0.707" w="0.707"/>
            </origin>
            <map_load_time secs="1708369100" nsecs="987654321"/>
        </info>
        <data>0 100 -1 50 25 75 0 0 100 50</data>
    </OccupancyGrid>
   *
   * @param parameters pointer to the XMLElement containing parameters
   * @return pointer to the XMLElement containing updated parameters
   */
  virtual std::string update_on_success(std::string& parameters)
  {
    tinyxml2::XMLElement* element = convert_to_xml(parameters);

    // Create the OccupancyGrid element as a child of the existing parameters element
    tinyxml2::XMLElement* occupancyGridElement = element->GetDocument()->NewElement("OccupancyGrid");
    element->InsertEndChild(occupancyGridElement);

    // Header element with attributes
    tinyxml2::XMLElement* headerElement = element->GetDocument()->NewElement("header");
    headerElement->SetAttribute("frame_id", latest_message_->header.frame_id.c_str());

    tinyxml2::XMLElement* stampElement = element->GetDocument()->NewElement("stamp");
    stampElement->SetAttribute("secs", latest_message_->header.stamp.sec);
    stampElement->SetAttribute("nsecs", latest_message_->header.stamp.nanosec);
    headerElement->InsertEndChild(stampElement);

    occupancyGridElement->InsertEndChild(headerElement);

    // Info element with attributes
    tinyxml2::XMLElement* infoElement = element->GetDocument()->NewElement("info");
    infoElement->SetAttribute("resolution", latest_message_->info.resolution);
    infoElement->SetAttribute("width", latest_message_->info.width);
    infoElement->SetAttribute("height", latest_message_->info.height);

    // Origin element with position and orientation attributes
    tinyxml2::XMLElement* originElement = element->GetDocument()->NewElement("origin");

    tinyxml2::XMLElement* positionElement = element->GetDocument()->NewElement("position");
    positionElement->SetAttribute("x", latest_message_->info.origin.position.x);
    positionElement->SetAttribute("y", latest_message_->info.origin.position.y);
    positionElement->SetAttribute("z", latest_message_->info.origin.position.z);
    originElement->InsertEndChild(positionElement);

    tinyxml2::XMLElement* orientationElement = element->GetDocument()->NewElement("orientation");
    orientationElement->SetAttribute("x", latest_message_->info.origin.orientation.x);
    orientationElement->SetAttribute("y", latest_message_->info.origin.orientation.y);
    orientationElement->SetAttribute("z", latest_message_->info.origin.orientation.z);
    orientationElement->SetAttribute("w", latest_message_->info.origin.orientation.w);
    originElement->InsertEndChild(orientationElement);

    infoElement->InsertEndChild(originElement);

    // Map load time with attributes
    tinyxml2::XMLElement* mapLoadTimeElement = element->GetDocument()->NewElement("map_load_time");
    mapLoadTimeElement->SetAttribute("secs", latest_message_->info.map_load_time.sec);
    mapLoadTimeElement->SetAttribute("nsecs", latest_message_->info.map_load_time.nanosec);
    infoElement->InsertEndChild(mapLoadTimeElement);

    occupancyGridElement->InsertEndChild(infoElement);

    // Data element for occupancy data (converted to a string)
    tinyxml2::XMLElement* dataElement = element->GetDocument()->NewElement("data");

    std::string data_str;
    for (size_t i = 0; i < latest_message_->data.size(); i++)
    {
      data_str += std::to_string(latest_message_->data[i]) + " ";
    }
    dataElement->SetText(data_str.c_str());

    occupancyGridElement->InsertEndChild(dataElement);

    // Return the updated parameters element with OccupancyGrid added
    std::string result = convert_to_string(element);

    output_("on_success trigger parameter", result);

    return result;
  };
};
}  // namespace capabilities2_runner