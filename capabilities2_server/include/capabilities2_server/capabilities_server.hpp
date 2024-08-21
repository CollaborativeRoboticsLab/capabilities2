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
#include <capabilities2_msgs/srv/get_remappings.hpp>
#include <capabilities2_msgs/srv/get_running_capabilities.hpp>

namespace capabilities2_server
{

/**
 * @brief capabilities server
 *
 * Capabilities server node that provides a ROS service capabilities API for managing capabilities.
 * implements the CapabilitiesAPI interface for core capabilities functionality.
 *
 */
class CapabilitiesServer : public rclcpp::Node, public CapabilitiesAPI
{
public:
  CapabilitiesServer(const rclcpp::NodeOptions& options = rclcpp::NodeOptions())
    : Node("capabilities", options), CapabilitiesAPI()
  {
    // params interface
    // loop rate
    declare_parameter("loop_rate", 5.0);
    double loop_hz_ = get_parameter("loop_rate").as_double();

    // db file
    declare_parameter("db_file", "cache/capabilities.sqlite3");
    std::string db_file = get_parameter("db_file").as_string();

    // rebuild db
    declare_parameter("rebuild", false);
    bool rebuild = get_parameter("rebuild").as_bool();

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
    connect(db_file, get_node_logging_interface());

    // load capabilities from package paths
    for (const auto& package_path : package_paths)
    {
      load_capabilities(package_path);
    }

    // pubs
    // events
    event_pub_ = create_publisher<capabilities2_msgs::msg::CapabilityEvent>("~/events", 10);

    // subs

    // services
    // establish bond
    establish_bond_srv_ = create_service<capabilities2_msgs::srv::EstablishBond>(
        "~/establish_bond",
        std::bind(&CapabilitiesServer::establish_bond_cb, this, std::placeholders::_1, std::placeholders::_2));

    // start capability
    start_capability_srv_ = create_service<capabilities2_msgs::srv::StartCapability>(
        "~/start_capability",
        std::bind(&CapabilitiesServer::start_capability_cb, this, std::placeholders::_1, std::placeholders::_2));

    // stop capability
    stop_capability_srv_ = create_service<capabilities2_msgs::srv::StopCapability>(
        "~/stop_capability",
        std::bind(&CapabilitiesServer::stop_capability_cb, this, std::placeholders::_1, std::placeholders::_2));

    // free capability
    free_capability_srv_ = create_service<capabilities2_msgs::srv::FreeCapability>(
        "~/free_capability",
        std::bind(&CapabilitiesServer::free_capability_cb, this, std::placeholders::_1, std::placeholders::_2));

    // use capability
    use_capability_srv_ = create_service<capabilities2_msgs::srv::UseCapability>(
        "~/use_capability",
        std::bind(&CapabilitiesServer::use_capability_cb, this, std::placeholders::_1, std::placeholders::_2));

    // register capability
    register_capability_srv_ = create_service<capabilities2_msgs::srv::RegisterCapability>(
        "~/register_capability",
        std::bind(&CapabilitiesServer::register_capability_cb, this, std::placeholders::_1, std::placeholders::_2));

    // query capabilities
    get_interfaces_srv_ = create_service<capabilities2_msgs::srv::GetInterfaces>(
        "~/get_interfaces",
        std::bind(&CapabilitiesServer::get_interfaces_cb, this, std::placeholders::_1, std::placeholders::_2));

    get_semantic_interfaces_srv_ = create_service<capabilities2_msgs::srv::GetSemanticInterfaces>(
        "~/get_semantic_interfaces",
        std::bind(&CapabilitiesServer::get_semantic_interfaces_cb, this, std::placeholders::_1, std::placeholders::_2));

    get_providers_srv_ = create_service<capabilities2_msgs::srv::GetProviders>(
        "~/get_providers",
        std::bind(&CapabilitiesServer::get_providers_cb, this, std::placeholders::_1, std::placeholders::_2));

    get_capability_spec_srv_ = create_service<capabilities2_msgs::srv::GetCapabilitySpec>(
        "~/get_capability_spec",
        std::bind(&CapabilitiesServer::get_capability_spec_cb, this, std::placeholders::_1, std::placeholders::_2));

    get_capability_specs_srv_ = create_service<capabilities2_msgs::srv::GetCapabilitySpecs>(
        "~/get_capability_specs",
        std::bind(&CapabilitiesServer::get_capability_specs_cb, this, std::placeholders::_1, std::placeholders::_2));

    get_remappings_srv_ = create_service<capabilities2_msgs::srv::GetRemappings>(
        "~/get_remappings",
        std::bind(&CapabilitiesServer::get_remappings_cb, this, std::placeholders::_1, std::placeholders::_2));

    get_running_capabilities_srv_ = create_service<capabilities2_msgs::srv::GetRunningCapabilities>(
        "~/get_running_capabilities", std::bind(&CapabilitiesServer::get_running_capabilities_cb, this,
                                                std::placeholders::_1, std::placeholders::_2));

    // timer to manage bonds and runners
    // cache_timer_ =
    //     create_wall_timer(std::chrono::seconds(1.0 / loop_hz_), std::bind(&CapabilitiesServer::cache_timer_cb,
    //     this));
  }

  // service callbacks
  // establish bond
  void establish_bond_cb(const std::shared_ptr<capabilities2_msgs::srv::EstablishBond::Request> req,
                         std::shared_ptr<capabilities2_msgs::srv::EstablishBond::Response> res)
  {
    // set bond id to response
    res->bond_id = establish_bond(shared_from_this());
  }

  // start capability
  void start_capability_cb(const std::shared_ptr<capabilities2_msgs::srv::StartCapability::Request> req,
                           std::shared_ptr<capabilities2_msgs::srv::StartCapability::Response> res)
  {
    // try starting capability
    // XXX TODO: handle errors
    start_capability(req->capability, req->preferred_provider);

    // set response
    res->result = capabilities2_msgs::srv::StartCapability::Response::RESULT_SUCCESS;
  }

  // stop capability
  void stop_capability_cb(const std::shared_ptr<capabilities2_msgs::srv::StopCapability::Request> req,
                          std::shared_ptr<capabilities2_msgs::srv::StopCapability::Response> res)
  {
    // try stopping capability
    // XXX TODO: handle errors
    stop_capability(req->capability);

    // set response
    res->successful = true;
  }

  // free capability
  void free_capability_cb(const std::shared_ptr<capabilities2_msgs::srv::FreeCapability::Request> req,
                          std::shared_ptr<capabilities2_msgs::srv::FreeCapability::Response> res)
  {
    // free capability of this bond
    free_capability(req->capability, req->bond_id);

    // response is empty
  }

  // use capability
  void use_capability_cb(const std::shared_ptr<capabilities2_msgs::srv::UseCapability::Request> req,
                         std::shared_ptr<capabilities2_msgs::srv::UseCapability::Response> res)
  {
    // use capability with this bond
    use_capability(req->capability, req->preferred_provider, req->bond_id);

    // response is empty
  }

  // register capability
  void register_capability_cb(const std::shared_ptr<capabilities2_msgs::srv::RegisterCapability::Request> req,
                              std::shared_ptr<capabilities2_msgs::srv::RegisterCapability::Response> res)
  {
    // register capability
    add_capability(req->capability_spec);

    // response is empty
  }

  // query capabilities
  // get interfaces
  void get_interfaces_cb(const std::shared_ptr<capabilities2_msgs::srv::GetInterfaces::Request> req,
                         std::shared_ptr<capabilities2_msgs::srv::GetInterfaces::Response> res)
  {
    // set response
    // get all interfaces
    res->interfaces = get_interfaces();
  }

  // get semantic interfaces
  void get_semantic_interfaces_cb(const std::shared_ptr<capabilities2_msgs::srv::GetSemanticInterfaces::Request> req,
                                  std::shared_ptr<capabilities2_msgs::srv::GetSemanticInterfaces::Response> res)
  {
    // set response
    // get semantic interfaces for given interface
    res->semantic_interfaces = get_sematic_interfaces(req->interface);
  }

  // get providers
  void get_providers_cb(const std::shared_ptr<capabilities2_msgs::srv::GetProviders::Request> req,
                        std::shared_ptr<capabilities2_msgs::srv::GetProviders::Response> res)
  {
    // set response
    // get providers for given interface
    res->providers = get_providers(req->interface, req->include_semantic);
  }

  // get capability spec
  void get_capability_spec_cb(const std::shared_ptr<capabilities2_msgs::srv::GetCapabilitySpec::Request> req,
                              std::shared_ptr<capabilities2_msgs::srv::GetCapabilitySpec::Response> res)
  {
    // get capability spec for given capability resource
    capabilities2_msgs::msg::CapabilitySpec capability_spec = get_capability_spec(req->capability_spec);

    // if the spec is not empty set response
    if (capability_spec.content.empty())
    {
      RCLCPP_ERROR(get_logger(), "capability spec not found for resource: %s", req->capability_spec.c_str());

      // throw error
      std::runtime_error("capability spec not found for resource: " + req->capability_spec);
    }

    // set response
    res->capability_spec = capability_spec;
  }

  // get capability specs
  void get_capability_specs_cb(const std::shared_ptr<capabilities2_msgs::srv::GetCapabilitySpecs::Request> req,
                               std::shared_ptr<capabilities2_msgs::srv::GetCapabilitySpecs::Response> res)
  {
    // set response
    // get capability specs for given capability resources
    res->capability_specs = get_capability_specs();
  }

  void get_remappings_cb(const std::shared_ptr<capabilities2_msgs::srv::GetRemappings::Request> req,
                         std::shared_ptr<capabilities2_msgs::srv::GetRemappings::Response> res)
  {
    // set response
    // get remappings for given capability
    get_remappings(req->spec, res);
  }

  void get_running_capabilities_cb(const std::shared_ptr<capabilities2_msgs::srv::GetRunningCapabilities::Request> req,
                                   std::shared_ptr<capabilities2_msgs::srv::GetRunningCapabilities::Response> res)
  {
    // set response
    // get running capabilities
    res->running_capabilities = get_running_capabilities();
  }

  // cache timer callback
  void cache_timer_cb()
  {
    // XXX TODO: manage bonds and runners
  }

public:
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

      // XXX TODO: get capabilities
    }
  }

private:
  // loop hz
  double loop_hz_;

  // publishers
  // event publisher
  rclcpp::Publisher<capabilities2_msgs::msg::CapabilityEvent>::SharedPtr event_pub_;

  // services
  // establish bond
  rclcpp::Service<capabilities2_msgs::srv::EstablishBond>::SharedPtr establish_bond_srv_;
  // start capability
  rclcpp::Service<capabilities2_msgs::srv::StartCapability>::SharedPtr start_capability_srv_;
  // stop capability
  rclcpp::Service<capabilities2_msgs::srv::StopCapability>::SharedPtr stop_capability_srv_;
  // free capability
  rclcpp::Service<capabilities2_msgs::srv::FreeCapability>::SharedPtr free_capability_srv_;
  // use capability
  rclcpp::Service<capabilities2_msgs::srv::UseCapability>::SharedPtr use_capability_srv_;
  // register capability
  rclcpp::Service<capabilities2_msgs::srv::RegisterCapability>::SharedPtr register_capability_srv_;
  // query capabilities
  rclcpp::Service<capabilities2_msgs::srv::GetInterfaces>::SharedPtr get_interfaces_srv_;
  // get semantic interfaces
  rclcpp::Service<capabilities2_msgs::srv::GetSemanticInterfaces>::SharedPtr get_semantic_interfaces_srv_;
  // get providers
  rclcpp::Service<capabilities2_msgs::srv::GetProviders>::SharedPtr get_providers_srv_;
  // get capability spec
  rclcpp::Service<capabilities2_msgs::srv::GetCapabilitySpec>::SharedPtr get_capability_spec_srv_;
  // get capability specs
  rclcpp::Service<capabilities2_msgs::srv::GetCapabilitySpecs>::SharedPtr get_capability_specs_srv_;
  // get remappings
  rclcpp::Service<capabilities2_msgs::srv::GetRemappings>::SharedPtr get_remappings_srv_;
  // get running capabilities
  rclcpp::Service<capabilities2_msgs::srv::GetRunningCapabilities>::SharedPtr get_running_capabilities_srv_;

  // cache timer
  rclcpp::TimerBase::SharedPtr cache_timer_;
};

}  // namespace capabilities2_server
