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
    tinyxml2::XMLDocument doc;

    // Root element for OccupancyGrid
    tinyxml2::XMLElement* occupancyGridElement = doc.NewElement("OccupancyGrid");
    doc.InsertFirstChild(occupancyGridElement);

    // Header element
    tinyxml2::XMLElement* headerElement = doc.NewElement("header");
    occupancyGridElement->InsertEndChild(headerElement);

    // Header fields: stamp, frame_id
    tinyxml2::XMLElement* stampElement = doc.NewElement("stamp");
    headerElement->InsertEndChild(stampElement);

    tinyxml2::XMLElement* secsElement = doc.NewElement("secs");
    secsElement->SetText(result->header.stamp.sec);
    stampElement->InsertEndChild(secsElement);

    tinyxml2::XMLElement* nsecsElement = doc.NewElement("nsecs");
    nsecsElement->SetText(result->header.stamp.nanosec);
    stampElement->InsertEndChild(nsecsElement);

    tinyxml2::XMLElement* frameIdElement = doc.NewElement("frame_id");
    frameIdElement->SetText(result->header.frame_id.c_str());
    headerElement->InsertEndChild(frameIdElement);

    // Info element
    tinyxml2::XMLElement* infoElement = doc.NewElement("info");
    occupancyGridElement->InsertEndChild(infoElement);

    // Map info fields: resolution, width, height
    tinyxml2::XMLElement* resolutionElement = doc.NewElement("resolution");
    resolutionElement->SetText(result->info.resolution);
    infoElement->InsertEndChild(resolutionElement);

    tinyxml2::XMLElement* widthElement = doc.NewElement("width");
    widthElement->SetText(result->info.width);
    infoElement->InsertEndChild(widthElement);

    tinyxml2::XMLElement* heightElement = doc.NewElement("height");
    heightElement->SetText(result->info.height);
    infoElement->InsertEndChild(heightElement);

    // Origin position element
    tinyxml2::XMLElement* originElement = doc.NewElement("origin");
    infoElement->InsertEndChild(originElement);

    tinyxml2::XMLElement* positionElement = doc.NewElement("position");
    originElement->InsertEndChild(positionElement);

    tinyxml2::XMLElement* posX = doc.NewElement("x");
    posX->SetText(result->info.origin.position.x);
    positionElement->InsertEndChild(posX);

    tinyxml2::XMLElement* posY = doc.NewElement("y");
    posY->SetText(result->info.origin.position.y);
    positionElement->InsertEndChild(posY);

    tinyxml2::XMLElement* posZ = doc.NewElement("z");
    posZ->SetText(result->info.origin.position.z);
    positionElement->InsertEndChild(posZ);

    // Origin orientation element
    tinyxml2::XMLElement* orientationElement = doc.NewElement("orientation");
    originElement->InsertEndChild(orientationElement);

    tinyxml2::XMLElement* oriX = doc.NewElement("x");
    oriX->SetText(result->info.origin.orientation.x);
    orientationElement->InsertEndChild(oriX);

    tinyxml2::XMLElement* oriY = doc.NewElement("y");
    oriY->SetText(result->info.origin.orientation.y);
    orientationElement->InsertEndChild(oriY);

    tinyxml2::XMLElement* oriZ = doc.NewElement("z");
    oriZ->SetText(result->info.origin.orientation.z);
    orientationElement->InsertEndChild(oriZ);

    tinyxml2::XMLElement* oriW = doc.NewElement("w");
    oriW->SetText(result->info.origin.orientation.w);
    orientationElement->InsertEndChild(oriW);

    // Map load time
    tinyxml2::XMLElement* mapLoadTimeElement = doc.NewElement("map_load_time");
    infoElement->InsertEndChild(mapLoadTimeElement);

    tinyxml2::XMLElement* mapSecsElement = doc.NewElement("secs");
    mapSecsElement->SetText(result->info.map_load_time.sec);
    mapLoadTimeElement->InsertEndChild(mapSecsElement);

    tinyxml2::XMLElement* mapNsecsElement = doc.NewElement("nsecs");
    mapNsecsElement->SetText(result->info.map_load_time.nanosec);
    mapLoadTimeElement->InsertEndChild(mapNsecsElement);

    // Data element for occupancy grid data
    tinyxml2::XMLElement* dataElement = doc.NewElement("data");
    occupancyGridElement->InsertEndChild(dataElement);

    // Convert the occupancy grid data into a space-separated string
    std::string data_str;
    for (const auto& value : result->data) {
        data_str += std::to_string(value) + " ";
    }
    dataElement->SetText(data_str.c_str());

    return doc.FirstChildElement("OccupancyGrid");
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
    // Create the OccupancyGrid element as a child of the existing parameters element
    tinyxml2::XMLElement* occupancyGridElement = parameters->GetDocument()->NewElement("OccupancyGrid");
    parameters->InsertEndChild(occupancyGridElement);

    // Header element
    tinyxml2::XMLElement* headerElement = parameters->GetDocument()->NewElement("header");
    occupancyGridElement->InsertEndChild(headerElement);

    tinyxml2::XMLElement* stampElement = parameters->GetDocument()->NewElement("stamp");
    headerElement->InsertEndChild(stampElement);

    tinyxml2::XMLElement* secsElement = parameters->GetDocument()->NewElement("secs");
    secsElement->SetText(latest_message_->header.stamp.sec);
    stampElement->InsertEndChild(secsElement);

    tinyxml2::XMLElement* nsecsElement = parameters->GetDocument()->NewElement("nsecs");
    nsecsElement->SetText(latest_message_->header.stamp.nanosec);
    stampElement->InsertEndChild(nsecsElement);

    tinyxml2::XMLElement* frameIdElement = parameters->GetDocument()->NewElement("frame_id");
    frameIdElement->SetText(latest_message_->header.frame_id.c_str());
    headerElement->InsertEndChild(frameIdElement);

    // Info element
    tinyxml2::XMLElement* infoElement = parameters->GetDocument()->NewElement("info");
    occupancyGridElement->InsertEndChild(infoElement);

    tinyxml2::XMLElement* resolutionElement = parameters->GetDocument()->NewElement("resolution");
    resolutionElement->SetText(latest_message_->info.resolution);
    infoElement->InsertEndChild(resolutionElement);

    tinyxml2::XMLElement* widthElement = parameters->GetDocument()->NewElement("width");
    widthElement->SetText(latest_message_->info.width);
    infoElement->InsertEndChild(widthElement);

    tinyxml2::XMLElement* heightElement = parameters->GetDocument()->NewElement("height");
    heightElement->SetText(latest_message_->info.height);
    infoElement->InsertEndChild(heightElement);

    // Origin element
    tinyxml2::XMLElement* originElement = parameters->GetDocument()->NewElement("origin");
    infoElement->InsertEndChild(originElement);

    tinyxml2::XMLElement* positionElement = parameters->GetDocument()->NewElement("position");
    originElement->InsertEndChild(positionElement);

    tinyxml2::XMLElement* posX = parameters->GetDocument()->NewElement("x");
    posX->SetText(latest_message_->info.origin.position.x);
    positionElement->InsertEndChild(posX);

    tinyxml2::XMLElement* posY = parameters->GetDocument()->NewElement("y");
    posY->SetText(latest_message_->info.origin.position.y);
    positionElement->InsertEndChild(posY);

    tinyxml2::XMLElement* posZ = parameters->GetDocument()->NewElement("z");
    posZ->SetText(latest_message_->info.origin.position.z);
    positionElement->InsertEndChild(posZ);

    tinyxml2::XMLElement* orientationElement = parameters->GetDocument()->NewElement("orientation");
    originElement->InsertEndChild(orientationElement);

    tinyxml2::XMLElement* oriX = parameters->GetDocument()->NewElement("x");
    oriX->SetText(latest_message_->info.origin.orientation.x);
    orientationElement->InsertEndChild(oriX);

    tinyxml2::XMLElement* oriY = parameters->GetDocument()->NewElement("y");
    oriY->SetText(latest_message_->info.origin.orientation.y);
    orientationElement->InsertEndChild(oriY);

    tinyxml2::XMLElement* oriZ = parameters->GetDocument()->NewElement("z");
    oriZ->SetText(latest_message_->info.origin.orientation.z);
    orientationElement->InsertEndChild(oriZ);

    tinyxml2::XMLElement* oriW = parameters->GetDocument()->NewElement("w");
    oriW->SetText(latest_message_->info.origin.orientation.w);
    orientationElement->InsertEndChild(oriW);

    // Map load time
    tinyxml2::XMLElement* mapLoadTimeElement = parameters->GetDocument()->NewElement("map_load_time");
    infoElement->InsertEndChild(mapLoadTimeElement);

    tinyxml2::XMLElement* mapSecsElement = parameters->GetDocument()->NewElement("secs");
    mapSecsElement->SetText(latest_message_->info.map_load_time.sec);
    mapLoadTimeElement->InsertEndChild(mapSecsElement);

    tinyxml2::XMLElement* mapNsecsElement = parameters->GetDocument()->NewElement("nsecs");
    mapNsecsElement->SetText(latest_message_->info.map_load_time.nanosec);
    mapLoadTimeElement->InsertEndChild(mapNsecsElement);

    // Data element for occupancy data (converted to a string)
    tinyxml2::XMLElement* dataElement = parameters->GetDocument()->NewElement("data");
    occupancyGridElement->InsertEndChild(dataElement);

    std::string data_str;
    for (size_t i = 0; i < latest_message_->data.size(); ++i) {
        data_str += std::to_string(latest_message_->data[i]) + " ";
    }
    dataElement->SetText(data_str.c_str());

    // Return the updated parameters element with OccupancyGrid added
    return parameters;
  };
};
}  // namespace capabilities2_runner