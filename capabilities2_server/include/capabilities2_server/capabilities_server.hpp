#pragma once

#include <map>
#include <vector>
#include <memory>
#include <string>
#include <fstream>
#include <filesystem>
#include <functional>

#include <stdlib.h>

#include <tinyxml2.h>

#include <rclcpp/rclcpp.hpp>

#include <capabilities2_server/capabilities_api.hpp>

#include <capabilities2_msgs/msg/capability_event.hpp>
#include <capabilities2_msgs/msg/capability_spec.hpp>
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

/** */
std::string expand_tilde(std::string& path)
{
  if (path[0] == '~')
  {
    path = std::string(std::getenv("HOME")) + path.substr(1);
  }
  return path;
}

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
    declare_parameter("db_file", "~/.ros/capabilities/capabilities.sqlite3");
    std::string db_file = get_parameter("db_file").as_string();

    // rebuild db
    declare_parameter("rebuild", false);
    bool rebuild = get_parameter("rebuild").as_bool();

    // package path
    declare_parameter("package_paths", std::vector<std::string>());
    std::vector<std::string> package_paths = get_parameter("package_paths").as_string_array();

    // get full path of db file
    std::filesystem::path db_path = std::filesystem::absolute(expand_tilde(db_file));
    db_file = db_path.string();

    if (rebuild)
    {
      // remove db file if it exists
      RCLCPP_INFO(get_logger(), "Rebuilding capabilities database");
      if (std::remove(db_file.c_str()) != 0)
      {
        RCLCPP_ERROR(get_logger(), "Error deleting file %s", db_file.c_str());
      }
    }

    // if db file does not exist
    if (!std::filesystem::exists(db_path))
    {
      // create db file path
      std::filesystem::create_directories(db_path.parent_path());
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

    // create publishing event callbacks by binding the event publisher and event message callbacks
    // handled by the API class and passed around to runners
    // on started, stopped, and terminated lambdas binding event_pub_
    // init events system callbacks with lambdas
    init_events([this](const std::string& cap) { event_pub_->publish(on_capability_started(cap)); },
                [this](const std::string& cap) { event_pub_->publish(on_capability_stopped(cap)); },
                [this](const std::string& cap) { event_pub_->publish(on_capability_terminated(cap)); });

    // log ready
    RCLCPP_INFO(get_logger(), "capabilities server started");

    // publish ready event
    event_pub_->publish(on_server_ready());
  }

  // service callbacks
  // establish bond
  void establish_bond_cb(const std::shared_ptr<capabilities2_msgs::srv::EstablishBond::Request> req,
                         std::shared_ptr<capabilities2_msgs::srv::EstablishBond::Response> res)
  {
    // req is empty

    // set bond id to response
    res->bond_id = establish_bond(shared_from_this());
  }

  // start capability
  void start_capability_cb(const std::shared_ptr<capabilities2_msgs::srv::StartCapability::Request> req,
                           std::shared_ptr<capabilities2_msgs::srv::StartCapability::Response> res)
  {
    // try starting capability
    // TODO: handle errors
    start_capability(shared_from_this(), req->capability, req->preferred_provider);

    // set response
    res->result = capabilities2_msgs::srv::StartCapability::Response::RESULT_SUCCESS;
  }

  // stop capability
  void stop_capability_cb(const std::shared_ptr<capabilities2_msgs::srv::StopCapability::Request> req,
                          std::shared_ptr<capabilities2_msgs::srv::StopCapability::Response> res)
  {
    // try stopping capability
    // TODO: handle errors
    stop_capability(req->capability);

    // set response
    res->successful = true;
  }

  // free capability
  void free_capability_cb(const std::shared_ptr<capabilities2_msgs::srv::FreeCapability::Request> req,
                          std::shared_ptr<capabilities2_msgs::srv::FreeCapability::Response> res)
  {
    // guard empty values
    if (req->capability.empty())
    {
      RCLCPP_ERROR(get_logger(), "free_capability: capability is empty");
      return;
    }

    if (req->bond_id.empty())
    {
      RCLCPP_ERROR(get_logger(), "free_capability: bond_id is empty");
      return;
    }

    // free capability of this bond
    free_capability(req->capability, req->bond_id);

    // response is empty
  }

  // use capability
  void use_capability_cb(const std::shared_ptr<capabilities2_msgs::srv::UseCapability::Request> req,
                         std::shared_ptr<capabilities2_msgs::srv::UseCapability::Response> res)
  {
    // guard empty values
    if (req->capability.empty())
    {
      RCLCPP_ERROR(get_logger(), "use_capability: capability is empty");
      return;
    }

    if (req->preferred_provider.empty())
    {
      RCLCPP_ERROR(get_logger(), "use_capability: preferred_provider is empty");
      return;
    }

    if (req->bond_id.empty())
    {
      RCLCPP_ERROR(get_logger(), "use_capability: bond_id is empty");
      return;
    }

    // use capability with this bond
    use_capability(shared_from_this(), req->capability, req->preferred_provider, req->bond_id);

    // response is empty
  }

  // register capability
  void register_capability_cb(const std::shared_ptr<capabilities2_msgs::srv::RegisterCapability::Request> req,
                              std::shared_ptr<capabilities2_msgs::srv::RegisterCapability::Response> res)
  {
    // guard empty values
    if (req->capability_spec.package.empty() || req->capability_spec.content.empty())
    {
      RCLCPP_ERROR(get_logger(), "register_capability: package or content is empty");
      return;
    }

    // register capability
    add_capability(req->capability_spec);

    // response is empty
  }

  // query capabilities
  // get interfaces
  void get_interfaces_cb(const std::shared_ptr<capabilities2_msgs::srv::GetInterfaces::Request> req,
                         std::shared_ptr<capabilities2_msgs::srv::GetInterfaces::Response> res)
  {
    // req is empty

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

      // BUG: throw error causes service to crash, this is a ROS2 bug
      // std::runtime_error("capability spec not found for resource: " + req->capability_spec);
      return;
    }

    // set response
    res->capability_spec = capability_spec;
  }

  // get capability specs
  void get_capability_specs_cb(const std::shared_ptr<capabilities2_msgs::srv::GetCapabilitySpecs::Request> req,
                               std::shared_ptr<capabilities2_msgs::srv::GetCapabilitySpecs::Response> res)
  {
    // req is empty

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
    // req is empty

    // set response
    // get running capabilities
    res->running_capabilities = get_running_capabilities();
  }

private:
  void load_capabilities(const std::string& package_path)
  {
    RCLCPP_DEBUG(get_logger(), "Loading capabilities from package path: %s", package_path.c_str());

    // check if path exists
    if (!std::filesystem::exists(package_path))
    {
      RCLCPP_ERROR(get_logger(), "package path does not exist: %s", package_path.c_str());
      return;
    }

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
      RCLCPP_DEBUG(get_logger(), "loading capabilities from package: %s", package.c_str());

      // package.xml exports
      std::string package_xml = package_path + "/" + package + "/package.xml";

      // check if package.xml exists
      if (!std::filesystem::exists(package_xml))
      {
        RCLCPP_DEBUG(get_logger(), "package.xml does not exist: %s", package_xml.c_str());
        continue;
      }

      // parse package.xml
      tinyxml2::XMLDocument doc;
      try
      {
        parse_package_xml(package_xml, doc);
      }
      catch (const std::runtime_error& e)
      {
        RCLCPP_ERROR(get_logger(), "failed to parse package.xml file: %s", e.what());
        continue;
      }

      // get exports
      tinyxml2::XMLElement* exports = doc.FirstChildElement("package")->FirstChildElement("export");
      if (exports == nullptr)
      {
        RCLCPP_DEBUG(get_logger(), "No exports found in package.xml file: %s", package_xml.c_str());
        continue;
      }

      // get capability specs of each type using a lambda
      auto get_capability_specs = [&](const std::string& spec_type) {
        // get capability spec
        for (tinyxml2::XMLElement* spec = exports->FirstChildElement(spec_type.c_str()); spec != nullptr;
             spec = spec->NextSiblingElement(spec_type.c_str()))
        {
          // read spec relative path for spec file from element contents
          std::string spec_path = spec->GetText();
          // clear white spaces
          spec_path.erase(std::remove_if(spec_path.begin(), spec_path.end(), isspace), spec_path.end());
          // remove leading slash
          if (spec_path[0] == '/')
          {
            spec_path = spec_path.substr(1);
          }

          // create a spec message
          capabilities2_msgs::msg::CapabilitySpec capability_spec;

          // add package details
          capability_spec.package = package;
          capability_spec.type = spec_type;

          // try load spec file
          try
          {
            // read spec file
            load_spec_content(package_path + "/" + package + "/" + spec_path, capability_spec);

            // add capability to db
            RCLCPP_INFO(get_logger(), "adding capability: %s", (package + "/" + spec_path).c_str());
            add_capability(capability_spec);
          }
          catch (const std::runtime_error& e)
          {
            RCLCPP_ERROR(get_logger(), "failed to load spec file: %s", e.what());
          }
        }
      };

      // get interface specs
      get_capability_specs(capabilities2_msgs::msg::CapabilitySpec::CAPABILITY_INTERFACE);
      // get semantic interface specs
      get_capability_specs(capabilities2_msgs::msg::CapabilitySpec::SEMANTIC_CAPABILITY_INTERFACE);
      // get provider specs
      get_capability_specs(capabilities2_msgs::msg::CapabilitySpec::CAPABILITY_PROVIDER);
    }
  }

  /**
   * @brief parse package.xml file
   *
   * @param package_xml
   * @param doc
   */
  void parse_package_xml(const std::string& package_xml, tinyxml2::XMLDocument& doc)
  {
    // read package.xml
    std::ifstream package_xml_file(package_xml, std::ifstream::in);

    if (!package_xml_file.is_open())
    {
      throw std::runtime_error("unable to open package.xml file: " + package_xml);
    }

    // read package.xml file into string
    std::string package_xml_string{ std::istreambuf_iterator<char>(package_xml_file),
                                    std::istreambuf_iterator<char>() };
    package_xml_file.close();

    // parse package.xml to xml doc
    if (doc.Parse(package_xml_string.c_str()) != tinyxml2::XML_SUCCESS)
    {
      throw std::runtime_error("failed to parse package.xml file: " + package_xml);
    }
  }

  /**
   * @brief load capability spec content from a file
   *
   * @param spec_file
   * @param capability_spec
   */
  void load_spec_content(const std::string& spec_file, capabilities2_msgs::msg::CapabilitySpec& capability_spec)
  {
    // read spec file
    std::ifstream spec_file_file(spec_file, std::ifstream::in);

    // file failed guard
    if (!spec_file_file.is_open())
    {
      throw std::runtime_error("unable to open capability spec file: " + spec_file);
    }

    // load content into msg
    std::string content{ std::istreambuf_iterator<char>(spec_file_file), std::istreambuf_iterator<char>() };
    capability_spec.content = content;

    spec_file_file.close();
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
};

}  // namespace capabilities2_server
