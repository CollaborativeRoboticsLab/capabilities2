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
    init_subscriber(node, run_config, "map", print);
  }

protected:
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
  virtual std::string update_on_success(std::string& parameters)
  {
    tinyxml2::XMLElement* element = convert_to_xml(parameters);

    // Create the OccupancyGrid element as a child of the existing parameters element
    tinyxml2::XMLElement* occupancyGridElement = element->GetDocument()->NewElement("OccupancyGrid");
    element->InsertEndChild(occupancyGridElement);

    // Header element
    tinyxml2::XMLElement* headerElement = element->GetDocument()->NewElement("header");
    occupancyGridElement->InsertEndChild(headerElement);

    tinyxml2::XMLElement* stampElement = element->GetDocument()->NewElement("stamp");
    headerElement->InsertEndChild(stampElement);

    tinyxml2::XMLElement* secsElement = element->GetDocument()->NewElement("secs");
    secsElement->SetText(latest_message_->header.stamp.sec);
    stampElement->InsertEndChild(secsElement);

    tinyxml2::XMLElement* nsecsElement = element->GetDocument()->NewElement("nsecs");
    nsecsElement->SetText(latest_message_->header.stamp.nanosec);
    stampElement->InsertEndChild(nsecsElement);

    tinyxml2::XMLElement* frameIdElement = element->GetDocument()->NewElement("frame_id");
    frameIdElement->SetText(latest_message_->header.frame_id.c_str());
    headerElement->InsertEndChild(frameIdElement);

    // Info element
    tinyxml2::XMLElement* infoElement = element->GetDocument()->NewElement("info");
    occupancyGridElement->InsertEndChild(infoElement);

    tinyxml2::XMLElement* resolutionElement = element->GetDocument()->NewElement("resolution");
    resolutionElement->SetText(latest_message_->info.resolution);
    infoElement->InsertEndChild(resolutionElement);

    tinyxml2::XMLElement* widthElement = element->GetDocument()->NewElement("width");
    widthElement->SetText(latest_message_->info.width);
    infoElement->InsertEndChild(widthElement);

    tinyxml2::XMLElement* heightElement = element->GetDocument()->NewElement("height");
    heightElement->SetText(latest_message_->info.height);
    infoElement->InsertEndChild(heightElement);

    // Origin element
    tinyxml2::XMLElement* originElement = element->GetDocument()->NewElement("origin");
    infoElement->InsertEndChild(originElement);

    tinyxml2::XMLElement* positionElement = element->GetDocument()->NewElement("position");
    originElement->InsertEndChild(positionElement);

    tinyxml2::XMLElement* posX = element->GetDocument()->NewElement("x");
    posX->SetText(latest_message_->info.origin.position.x);
    positionElement->InsertEndChild(posX);

    tinyxml2::XMLElement* posY = element->GetDocument()->NewElement("y");
    posY->SetText(latest_message_->info.origin.position.y);
    positionElement->InsertEndChild(posY);

    tinyxml2::XMLElement* posZ = element->GetDocument()->NewElement("z");
    posZ->SetText(latest_message_->info.origin.position.z);
    positionElement->InsertEndChild(posZ);

    tinyxml2::XMLElement* orientationElement = element->GetDocument()->NewElement("orientation");
    originElement->InsertEndChild(orientationElement);

    tinyxml2::XMLElement* oriX = element->GetDocument()->NewElement("x");
    oriX->SetText(latest_message_->info.origin.orientation.x);
    orientationElement->InsertEndChild(oriX);

    tinyxml2::XMLElement* oriY = element->GetDocument()->NewElement("y");
    oriY->SetText(latest_message_->info.origin.orientation.y);
    orientationElement->InsertEndChild(oriY);

    tinyxml2::XMLElement* oriZ = element->GetDocument()->NewElement("z");
    oriZ->SetText(latest_message_->info.origin.orientation.z);
    orientationElement->InsertEndChild(oriZ);

    tinyxml2::XMLElement* oriW = element->GetDocument()->NewElement("w");
    oriW->SetText(latest_message_->info.origin.orientation.w);
    orientationElement->InsertEndChild(oriW);

    // Map load time
    tinyxml2::XMLElement* mapLoadTimeElement = element->GetDocument()->NewElement("map_load_time");
    infoElement->InsertEndChild(mapLoadTimeElement);

    tinyxml2::XMLElement* mapSecsElement = element->GetDocument()->NewElement("secs");
    mapSecsElement->SetText(latest_message_->info.map_load_time.sec);
    mapLoadTimeElement->InsertEndChild(mapSecsElement);

    tinyxml2::XMLElement* mapNsecsElement = element->GetDocument()->NewElement("nsecs");
    mapNsecsElement->SetText(latest_message_->info.map_load_time.nanosec);
    mapLoadTimeElement->InsertEndChild(mapNsecsElement);

    // Data element for occupancy data (converted to a string)
    tinyxml2::XMLElement* dataElement = element->GetDocument()->NewElement("data");
    occupancyGridElement->InsertEndChild(dataElement);

    std::string data_str;
    for (size_t i = 0; i < latest_message_->data.size(); ++i)
    {
      data_str += std::to_string(latest_message_->data[i]) + " ";
    }
    dataElement->SetText(data_str.c_str());

    // Return the updated parameters element with OccupancyGrid added
    std::string result = convert_to_string(element);

    return result;
  };
};
}  // namespace capabilities2_runner