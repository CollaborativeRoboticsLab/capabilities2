#pragma once

#include <map>
#include <vector>
#include <memory>
#include <string>
#include <fstream>
#include <filesystem>
#include <functional>

#include <tinyxml2.h>

#include <rclcpp/rclcpp.hpp>

#include <capabilities2_server/capabilities_api.hpp>

#include <capabilities2_msgs/msg/capability_event.hpp>
#include <capabilities2_msgs/srv/establish_bond.hpp>
#include <capabilities2_msgs/srv/start_capability.hpp>
#include <capabilities2_msgs/srv/stop_capability.hpp>
#include <capabilities2_msgs/srv/free_capability.hpp>
#include <capabilities2_msgs/srv/use_capability.hpp>
#include <capabilities2_msgs/srv/register_capability.hpp>
#include <capabilities2_msgs/srv/get_interfaces.hpp>
#include <capabilities2_msgs/srv/get_semantic_interfaces.hpp>
#include <capabilities2_msgs/srv/get_providers.hpp>
#include <capabilities2_msgs/srv/get_capability_spec.hpp>
#include <capabilities2_msgs/srv/get_capability_specs.hpp>

namespace capabilities2_server
{

class CapabilitiesServer : public rclcpp::Node, public CapabilitiesAPI
{
public:
  CapabilitiesServer(const rclcpp::NodeOptions& options = rclcpp::NodeOptions())
    : Node("capabilities", options), CapabilitiesAPI()
  {
    // params interface
    // db file
    declare_parameter("db_file", "cache/capabilities.sqlite3");
    std::string db_file = get_parameter("db_file").as_string();

    // rebuild db
    declare_parameter("rebuild", false);
    bool rebuild = get_parameter("rebuild").as_bool();

    // bond heartbeat timeout
    declare_parameter("default_heartbeat_timeout", 5.0);
    double default_heartbeat_timeout = get_parameter("default_heartbeat_timeout").as_double();

    // package path
    declare_parameter("package_paths", std::vector<std::string>());
    std::vector<std::string> package_paths = get_parameter("package_paths").as_string_array();

    if (rebuild)
    {
      // remove db file if it exists
      RCLCPP_INFO(get_logger(), "Rebuilding capabilities database");
      if (std::remove(db_file.c_str()) != 0)
      {
        RCLCPP_ERROR(get_logger(), "Error deleting file %s", db_file.c_str());
      }
    }

    // init capabilities api
    connect(db_file);

    // load capabilities from package paths
    for (const auto& package_path : package_paths)
    {
      load_capabilities(package_path);
    }

    // pubs
    // events
    // auto event_pub = create_publisher<capabilities2_msgs::msg::CapabilityEvent>("~events", 10);

    // subs

    // services
    // establish bond
    // auto establish_bond_srv = create_service<capabilities2_msgs::srv::EstablishBond>(
    //     "~establish_bond",
    //     std::bind(&CapabilitiesServer::establish_bond, this, std::placeholders::_1, std::placeholders::_2));

    // // start capability
    // auto start_capability_srv = create_service<capabilities2_msgs::srv::StartCapability>(
    //     "~start_capability",
    //     std::bind(&CapabilitiesServer::start_capability, this, std::placeholders::_1, std::placeholders::_2));

    // // stop capability
    // auto stop_capability_srv = create_service<capabilities2_msgs::srv::StopCapability>(
    //     "~stop_capability",
    //     std::bind(&CapabilitiesServer::stop_capability, this, std::placeholders::_1, std::placeholders::_2));

    // // free capability
    // auto free_capability_srv = create_service<capabilities2_msgs::srv::FreeCapability>(
    //     "~free_capability",
    //     std::bind(&CapabilitiesServer::free_capability, this, std::placeholders::_1, std::placeholders::_2));

    // // use capability
    // auto use_capability_srv = create_service<capabilities2_msgs::srv::UseCapability>(
    //     "~use_capability",
    //     std::bind(&CapabilitiesServer::use_capability, this, std::placeholders::_1, std::placeholders::_2));

    // // register capability
    // auto register_capability_srv = create_service<capabilities2_msgs::srv::RegisterCapability>(
    //     "~register_capability",
    //     std::bind(&CapabilitiesServer::register_capability, this, std::placeholders::_1, std::placeholders::_2));

    // // query capabilities
    // auto get_interfaces_srv = create_service<capabilities2_msgs::srv::GetInterfaces>(
    //     "~get_interfaces",
    //     std::bind(&CapabilitiesServer::get_interfaces, this, std::placeholders::_1, std::placeholders::_2));

    // auto get_semantic_interfaces_srv = create_service<capabilities2_msgs::srv::GetSemanticInterfaces>(
    //     "~get_semantic_interfaces",
    //     std::bind(&CapabilitiesServer::get_semantic_interfaces, this, std::placeholders::_1, std::placeholders::_2));

    // auto get_providers_srv = create_service<capabilities2_msgs::srv::GetProviders>(
    //     "~get_providers",
    //     std::bind(&CapabilitiesServer::get_providers, this, std::placeholders::_1, std::placeholders::_2));

    // auto get_capability_spec_srv = create_service<capabilities2_msgs::srv::GetCapabilitySpec>(
    //     "~get_capability_spec",
    //     std::bind(&CapabilitiesServer::get_capability_spec, this, std::placeholders::_1, std::placeholders::_2));

    // auto get_capability_specs_srv = create_service<capabilities2_msgs::srv::GetCapabilitySpecs>(
    //     "~get_capability_specs",
    //     std::bind(&CapabilitiesServer::get_capability_specs, this, std::placeholders::_1, std::placeholders::_2));
  }

  void load_capabilities(const std::string& package_path)
  {
    RCLCPP_INFO(get_logger(), "Loading capabilities from package path: %s", package_path.c_str());

    // find packages in path
    std::vector<std::string> packages;
    for (const auto& entry : std::filesystem::directory_iterator(package_path))
    {
      if (entry.is_directory())
      {
        packages.push_back(entry.path().filename());
      }
    }

    // load capabilities from packages
    for (const auto& package : packages)
    {
      RCLCPP_INFO(get_logger(), "Loading capabilities from package: %s", package.c_str());

      // package.xml exports
      std::string package_xml = package_path + "/" + package + "/package.xml";
      std::string package_xml_string;

      // read package.xml
      std::ifstream package_xml_file(package_xml, std::ifstream::in);
      if (package_xml_file.is_open())
      {
        std::string line;
        while (std::getline(package_xml_file, line))
        {
          package_xml_string += line;
        }
        package_xml_file.close();
      }
      else
      {
        RCLCPP_ERROR(get_logger(), "Unable to open package.xml file: %s", package_xml.c_str());
        continue;
      }

      // parse package.xml
      tinyxml2::XMLDocument doc;
      if (doc.Parse(package_xml_string.c_str()) != tinyxml2::XML_SUCCESS)
      {
        RCLCPP_ERROR(get_logger(), "Failed to parse package.xml file: %s", package_xml.c_str());
        continue;
      }

      // get exports
      tinyxml2::XMLElement* exports = doc.FirstChildElement("package")->FirstChildElement("export");
      if (exports == nullptr)
      {
        RCLCPP_ERROR(get_logger(), "No exports found in package.xml file: %s", package_xml.c_str());
        continue;
      }

      // get capabilities
    }
  }

private:
};

}  // namespace capabilities2_server
