#include <memory>
#include <thread>
#include <string>
#include <vector>
#include <algorithm>
#include <tinyxml2.h>

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>

#include <capabilities2_fabric/utils/xml_parser.hpp>
#include <capabilities2_fabric/utils/bond_client.hpp>

#include <capabilities2_msgs/action/plan.hpp>

#include <capabilities2_msgs/srv/establish_bond.hpp>
#include <capabilities2_msgs/srv/get_interfaces.hpp>
#include <capabilities2_msgs/srv/get_semantic_interfaces.hpp>
#include <capabilities2_msgs/srv/get_providers.hpp>
#include <capabilities2_msgs/srv/use_capability.hpp>
#include <capabilities2_msgs/srv/free_capability.hpp>
#include <capabilities2_msgs/srv/configure_capability.hpp>
#include <capabilities2_msgs/srv/trigger_capability.hpp>

#include <capabilities2_events/event_client.hpp>

/**
 * @brief Capabilities Fabric
 *
 * Capabilities fabric node that provides a ROS client for the capabilities server.
 * Able to receive a XML file that implements a plan via action server that it exposes
 *
 */

class CapabilitiesFabric : public rclcpp::Node
{
public:
  using Plan = capabilities2_msgs::action::Plan;
  using GoalHandlePlan = rclcpp_action::ServerGoalHandle<Plan>;

  using GetInterfaces = capabilities2_msgs::srv::GetInterfaces;
  using GetSemanticInterfaces = capabilities2_msgs::srv::GetSemanticInterfaces;
  using GetProviders = capabilities2_msgs::srv::GetProviders;
  using EstablishBond = capabilities2_msgs::srv::EstablishBond;
  using UseCapability = capabilities2_msgs::srv::UseCapability;
  using FreeCapability = capabilities2_msgs::srv::FreeCapability;
  using ConfigureCapability = capabilities2_msgs::srv::ConfigureCapability;
  using TriggerCapability = capabilities2_msgs::srv::TriggerCapability;

  using GetInterfacesClient = rclcpp::Client<GetInterfaces>;
  using GetSemanticInterfacesClient = rclcpp::Client<GetSemanticInterfaces>;
  using GetProvidersClient = rclcpp::Client<GetProviders>;
  using EstablishBondClient = rclcpp::Client<EstablishBond>;
  using UseCapabilityClient = rclcpp::Client<UseCapability>;
  using FreeCapabilityClient = rclcpp::Client<FreeCapability>;
  using ConfigureCapabilityClient = rclcpp::Client<ConfigureCapability>;
  using TriggerCapabilityClient = rclcpp::Client<TriggerCapability>;

  CapabilitiesFabric(const rclcpp::NodeOptions& options = rclcpp::NodeOptions()) : Node("Capabilities2_Fabric", options)
  {
    control_tag_list = xml_parser::get_control_list();
  }

  /**
   * @brief Initializer function for Ccapabilities2 fabric.
   * Configure the action server for the capabilieites fabric and configure server clients for the capability runners from the
   * capabilities2 server
   *
   */
  void initialize()
  {
    event_ = std::make_shared<EventClient>(shared_from_this(), "capabilities_fabric", "/events/capabilities_fabric");

    this->planner_server_ = rclcpp_action::create_server<Plan>(
        this, "/capabilities_fabric", std::bind(&CapabilitiesFabric::handle_goal, this, std::placeholders::_1, std::placeholders::_2),
        std::bind(&CapabilitiesFabric::handle_cancel, this, std::placeholders::_1),
        std::bind(&CapabilitiesFabric::handle_accepted, this, std::placeholders::_1));

    get_interfaces_client_ = this->create_client<GetInterfaces>("/capabilities/get_interfaces");
    get_sem_interf_client_ = this->create_client<GetSemanticInterfaces>("/capabilities/get_semantic_interfaces");
    get_providers_client_ = this->create_client<GetProviders>("/capabilities/get_providers");
    establish_bond_client_ = this->create_client<EstablishBond>("/capabilities/establish_bond");
    use_capability_client_ = this->create_client<UseCapability>("/capabilities/use_capability");
    free_capability_client_ = this->create_client<FreeCapability>("/capabilities/free_capability");
    trig_capability_client_ = this->create_client<TriggerCapability>("/capabilities/trigger_capability");
    conf_capability_client_ = this->create_client<ConfigureCapability>("/capabilities/configure_capability");

    // Wait for services to become available
    check_service(!get_interfaces_client_->wait_for_service(std::chrono::seconds(1)), "/capabilities/get_interfaces");
    check_service(!get_sem_interf_client_->wait_for_service(std::chrono::seconds(1)), "/capabilities/get_semantic_interfaces");
    check_service(!get_providers_client_->wait_for_service(std::chrono::seconds(1)), "/capabilities/get_providers");
    check_service(!establish_bond_client_->wait_for_service(std::chrono::seconds(1)), "/capabilities/establish_bond");
    check_service(!use_capability_client_->wait_for_service(std::chrono::seconds(1)), "/capabilities/use_capability");
    check_service(!free_capability_client_->wait_for_service(std::chrono::seconds(1)), "/capabilities/free_capability");
    check_service(!trig_capability_client_->wait_for_service(std::chrono::seconds(1)), "/capabilities/trigger_capability");
    check_service(!conf_capability_client_->wait_for_service(std::chrono::seconds(1)), "/capabilities/configure_capability");

    feedback_msg = std::make_shared<Plan::Feedback>();
    result_msg = std::make_shared<Plan::Result>();
  }

private:
  /**
   * @brief Handle the goal request that comes in from client. returns whether goal is accepted or rejected
   *
   *
   * @param uuid uuid of the goal
   * @param goal pointer to the action goal message
   * @return rclcpp_action::GoalResponse
   */
  rclcpp_action::GoalResponse handle_goal(const rclcpp_action::GoalUUID& uuid, std::shared_ptr<const Plan::Goal> goal)
  {
    event_->info("Received the goal request with the plan");

    (void)uuid;

    event_->info("Following plan was received :\n\n " + goal->plan);

    // try to parse the std::string plan from capabilities_msgs/Plan to the to a XMLDocument file
    tinyxml2::XMLError xml_status = document.Parse(goal->plan.c_str());

    // check if the file parsing failed
    if (xml_status != tinyxml2::XMLError::XML_SUCCESS)
    {
      event_->error("Parsing the plan from goal message failed");
      return rclcpp_action::GoalResponse::REJECT;
    }

    event_->info("Plan parsed and accepted");
    return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
  }

  /**
   * @brief Handle the goal cancel request that comes in from client.
   *
   * @param goal_handle pointer to the action goal handle
   * @return rclcpp_action::GoalResponse
   */
  rclcpp_action::CancelResponse handle_cancel(const std::shared_ptr<GoalHandlePlan> goal_handle)
  {
    event_->info("Received the request to cancel the plan");
    (void)goal_handle;

    for (auto& [bond_id, bond_client] : bond_client_cache_)
    {
      bond_client->stop();
    }

    return rclcpp_action::CancelResponse::ACCEPT;
  }

  /**
   * @brief Handle the goal accept event originating from handle_goal.
   *
   * @param goal_handle pointer to the action goal handle
   */
  void handle_accepted(const std::shared_ptr<GoalHandlePlan> goal_handle)
  {
    goal_handle_ = goal_handle;
    event_->info("Goal handle accepted");

    execution();
  }

  /**
   * @brief Start the execution of the capabilities2 fabric
   */
  void execution()
  {
    event_->info("A new execution started");

    xml_parser::add_closing_event(document);
    xml_parser::convert_to_string(document, modified_plan);

    event_->info("Plan after adding closing event :\n\n " + modified_plan);

    interface_list.clear();
    providers_list.clear();
    rejected_list.clear();
    connection_map.clear();

    expected_providers_ = 0;
    completed_providers_ = 0;

    expected_interfaces_ = 0;
    completed_interfaces_ = 0;

    expected_capabilities_ = 0;
    completed_capabilities_ = 0;
    freed_capabilities_ = 0;

    expected_configurations_ = 0;
    completed_configurations_ = 0;

    getInterfaces();
  }

  /**
   * @brief Get Interfaces available in the capabilities2 server via relavant service
   */
  void getInterfaces()
  {
    event_->info("Requesting Interface information");

    auto request_interface = std::make_shared<GetInterfaces::Request>();

    // request data from the server
    auto result_future = get_interfaces_client_->async_send_request(request_interface, [this](GetInterfacesClient::SharedFuture future) {
      auto result = std::make_shared<Plan::Result>();

      if (!future.valid())
      {
        result_msg->success = false;
        result_msg->message = "Failed to get Interface information. Server execution cancelled";
        event_->error(result_msg->message);
        goal_handle_->abort(result_msg);
        return;
      }

      auto response = future.get();
      expected_interfaces_ = response->interfaces.size();

      event_->info("Received Interfaces. Requsting " + std::to_string(expected_interfaces_) + " semantic interface information");

      // Request each interface recursively for Semantic interfaces
      getSemanticInterfaces(response->interfaces);
    });
  }

  /**
   * @brief Get the Semantic Interfaces from the capabilities2 server via related service client
   *
   * @param interfaces std::vector of interfaces for which the semantic interfaces will be requested
   */
  void getSemanticInterfaces(const std::vector<std::string>& interfaces)
  {
    std::string requested_interface = interfaces[completed_interfaces_];

    event_->info("Requesting semantic interfaces for " + requested_interface);

    auto request_semantic = std::make_shared<GetSemanticInterfaces::Request>();
    request_semantic->interface = requested_interface;

    // request semantic interface from the server
    auto result_semantic_future = get_sem_interf_client_->async_send_request(
        request_semantic, [this, interfaces, requested_interface](GetSemanticInterfacesClient::SharedFuture future) {
          if (!future.valid())
          {
            result_msg->success = false;
            result_msg->message = "Failed to get Semantic Interface information. Server execution cancelled";
            event_->error(result_msg->message);
            goal_handle_->abort(result_msg);
            return;
          }

          completed_interfaces_++;
          auto response = future.get();

          // if semantic interfaces are availble for a given interface, add the semantic interface
          if (response->semantic_interfaces.size() > 0)
          {
            for (const auto& semantic_interface : response->semantic_interfaces)
            {
              interface_list.push_back(semantic_interface);
              is_semantic_list.push_back(true);

              event_->info(std::to_string(completed_interfaces_) + "/" + std::to_string(expected_interfaces_) + " : Received " +
                               semantic_interface + " for " + requested_interface + ". So added " + semantic_interface);
            }
          }
          // if no semantic interfaces are availble for a given interface, add the interface instead
          else
          {
            interface_list.push_back(requested_interface);
            is_semantic_list.push_back(false);

            event_->info(std::to_string(completed_interfaces_) + "/" + std::to_string(expected_interfaces_) + " : Received none for " +
                             requested_interface + ". So added " + requested_interface);
          }

          if (completed_interfaces_ != expected_interfaces_)
          {
            // Request next interface recursively for Semantic interfaces
            getSemanticInterfaces(interfaces);
          }
          else
          {
            event_->info("Received all requested Interface information");

            expected_providers_ = interface_list.size();

            event_->info("Requsting Provider information for " + std::to_string(expected_providers_) + " providers");

            // request providers from the interfaces in the interfaces_list
            getProvider(interface_list, is_semantic_list);
          }
        });
  }

  /**
   * @brief Get the Provider information for the related interfaces
   *
   * @param interfaces std::vector of interfaces
   * @param is_semantic std::vector of masks about interfaces with true value for semantic interfaces
   */
  void getProvider(const std::vector<std::string>& interfaces, const std::vector<bool>& is_semantic)
  {
    std::string requested_interface = interfaces[completed_providers_];
    bool semantic_flag = is_semantic[completed_providers_];

    event_->info("Requesting provider for " + requested_interface);

    auto request_providers = std::make_shared<GetProviders::Request>();

    // request providers of the semantic interface
    request_providers->interface = requested_interface;
    request_providers->include_semantic = semantic_flag;

    auto result_providers_future = get_providers_client_->async_send_request(
        request_providers, [this, is_semantic, requested_interface, interfaces](GetProvidersClient::SharedFuture future) {
          if (!future.valid())
          {
            result_msg->success = false;
            result_msg->message = "Failed to retrieve providers for interface: " + requested_interface;
            event_->error(result_msg->message);
            goal_handle_->abort(result_msg);
            return;
          }

          completed_providers_++;
          auto response = future.get();

          if (response->default_provider != "")
          {
            // add defualt provider to the list
            providers_list.push_back(response->default_provider);

            event_->info(std::to_string(completed_providers_) + "/" + std::to_string(expected_providers_) + " : Received " +
                             response->default_provider + " for " + requested_interface + ". So added " + response->default_provider);
          }

          // add additional providers to the list if available
          if (response->providers.size() > 0)
          {
            for (const auto& provider : response->providers)
            {
              providers_list.push_back(provider);

              event_->info(std::to_string(completed_providers_) + "/" + std::to_string(expected_providers_) + " : Received and added " +
                               provider + " for " + requested_interface);
            }
          }
          else
          {
            event_->info(std::to_string(completed_providers_) + "/" + std::to_string(expected_providers_) + " : No providers for " +
                             requested_interface);
          }

          // Check if all expected calls are completed before calling verify_plan
          if (completed_providers_ != expected_providers_)
          {
            // request providers for the next interface in the interfaces_list
            getProvider(interfaces, is_semantic);
          }
          else
          {
            event_->info("All requested interface, semantic interface and provider data recieved");

            verify_and_continue();
          }
        });
  }

  /**
   * @brief Verify the plan before continuing the execution using xml parsing and collected interface, semantic interface
   * and provider information
   *
   */
  void verify_and_continue()
  {
    event_->info("Verifying the plan");

    // verify the plan
    if (!verify_plan())
    {
      event_->info("Plan verification failed");

      if (rejected_list.size() > 0)
      {
        // TODO: improve with error codes
        auto result = std::make_shared<Plan::Result>();
        result->success = false;
        result->message = "Plan verification failed. There are mismatched events";

        for (const auto& rejected_element : rejected_list)
        {
          result->failed_elements.push_back(rejected_element);
        }
        goal_handle_->abort(result);

        event_->info(result->message);
      }
      else
      {
        // TODO: improve with error codes
        result_msg->success = false;
        result_msg->message = "Plan verification failed. Server Execution Cancelled.";
        event_->error(result_msg->message);
        goal_handle_->abort(result_msg);
        return;
      }

      event_->error("Server Execution Cancelled");
    }

    event_->info("Plan verification successful");

    // extract the plan from the XMLDocument
    tinyxml2::XMLElement* plan = xml_parser::get_plan(document);

    event_->info("Plan conversion successful");

    // Extract the connections from the plan
    xml_parser::extract_connections(plan, connection_map);

    event_->info("Connection extraction successful");

    // estasblish the bond with the server
    request_bond();
  }

  /**
   * @brief verify the plan using received interfaces
   *
   * @return `true` if interface retreival is successful,`false` otherwise
   */
  bool verify_plan()
  {
    auto feedback = std::make_shared<Plan::Feedback>();
    auto result = std::make_shared<Plan::Result>();

    // verify whether document got 'plan' tags
    if (!xml_parser::check_plan_tag(document))
    {
      result_msg->success = false;
      result_msg->message = "Execution plan is not compatible. Please recheck and update";
      event_->error(result_msg->message);
      goal_handle_->abort(result_msg);
      return false;
    }

    event_->info("'Plan' tag checking successful");

    // extract the components within the 'plan' tags
    tinyxml2::XMLElement* plan = xml_parser::get_plan(document);

    event_->info("Plan extraction complete");

    // verify whether the plan is valid
    if (!xml_parser::check_tags(event_, plan, interface_list, providers_list, control_tag_list, rejected_list))
    {
      result_msg->success = false;
      result_msg->message = "Execution plan is faulty. Please recheck and update";
      event_->error(result_msg->message);
      goal_handle_->abort(result_msg);
      return false;
    }

    event_->info("Checking tags successful");
    return true;
  }

  /**
   * @brief Request the bond from the capabilities2 server
   *
   */
  void request_bond()
  {
    event_->info("Requesting bond id");

    // create bond establishing server request
    auto request_bond = std::make_shared<EstablishBond::Request>();

    // send the request
    auto result_future = establish_bond_client_->async_send_request(request_bond, [this](EstablishBondClient::SharedFuture future) {
      if (!future.valid())
      {
        result_msg->success = false;
        result_msg->message = "Failed to retrieve the bond id. Server execution cancelled";
        event_->error(result_msg->message);
        goal_handle_->abort(result_msg);
        return;
      }

      auto response = future.get();
      bond_id_ = response->bond_id;
      event_->info("Received the bond id : " + bond_id_);

      establish_bond();
    });
  }

  /**
   * @brief establish the bond with capabilities2 server
   *
   */
  void establish_bond()
  {
    bond_client_cache_[bond_id_] = std::make_unique<BondClient>(shared_from_this(), bond_id_);
    bond_client_cache_[bond_id_]->start();

    event_->info("Bond sucessfully established with bond id : " + bond_id_);

    if (bond_client_cache_.size() > 1)
    {
      for (auto& [old_bond_id, bond_client] : bond_client_cache_)
      {
        if (old_bond_id != bond_id_)
        {
          bond_client->stop();
          event_->info("Stopping and removing old bond with id : " + old_bond_id);
        }
      }
    }

    expected_capabilities_ = connection_map.size();

    event_->info("Requsting start of " + std::to_string(expected_capabilities_) + " capabilities");

    use_capability(connection_map);
  }

  /**
   * @brief Request use of capability from capabilities2 server
   *
   * @param capabilities capability list to be started
   * @param provider provider of the capability
   */
  void use_capability(std::map<int, capabilities2::node_t>& capabilities)
  {
    std::string capability = capabilities[completed_capabilities_].source.runner;
    std::string provider = capabilities[completed_capabilities_].source.provider;

    auto request_use = std::make_shared<UseCapability::Request>();
    request_use->capability = capability;
    request_use->preferred_provider = provider;
    request_use->bond_id = bond_id_;

    event_->info("Starting capability of Runner " + std::to_string(completed_capabilities_) + " : " +
                         capabilities[completed_capabilities_].source.runner);

    // send the request
    auto result_future =
        use_capability_client_->async_send_request(request_use, [this, capability, provider](UseCapabilityClient::SharedFuture future) {
          if (!future.valid())
          {
            result_msg->success = false;
            result_msg->message = "Failed to Use capability " + capability + " from " + provider + ". Server Execution Cancelled";
            event_->error(result_msg->message);
            goal_handle_->abort(result_msg);

            // release all capabilities that were used since not all started successfully
            free_capability_all(connection_map);

            for (auto& [bond_id, bond_client] : bond_client_cache_)
            {
              bond_client->stop();
            }
            return;
          }

          completed_capabilities_++;

          auto response = future.get();

          event_->info(std::to_string(completed_capabilities_) + "/" + std::to_string(expected_capabilities_) + " : start succeessful");

          // Check if all expected calls are completed before calling verify_plan
          if (completed_capabilities_ == expected_capabilities_)
          {
            event_->info("All requested capabilities have been started. Configuring the capabilities with events");

            expected_configurations_ = connection_map.size();

            event_->info("Requsting capability configuration for " + std::to_string(expected_configurations_) + " capabilities");

            configure_capabilities(connection_map);
          }
          else
          {
            use_capability(connection_map);
          }
        });
  }

  /**
   * @brief Free all started capabilities in the capabilities map
   *
   * @param capabilities map of capabilities to be freed
   */
  void free_capability_all(std::map<int, capabilities2::node_t>& capabilities)
  {
    std::string capability = capabilities[freed_capabilities_].source.runner;

    auto request_free = std::make_shared<FreeCapability::Request>();
    request_free->capability = capability;
    request_free->bond_id = bond_id_;

    // send the request
    auto result_future = free_capability_client_->async_send_request(request_free, [this, capability](FreeCapabilityClient::SharedFuture future) {
      if (!future.valid())
      {
        result_msg->success = false;
        result_msg->message = "Failed to free capability " + capability;
        event_->error(result_msg->message);
        goal_handle_->abort(result_msg);
        return;
      }

      auto response = future.get();
      event_->info("Successfully freed capability " + capability);

      freed_capabilities_++;

      // Check if all expected calls are completed before calling verify_plan
      if (freed_capabilities_ == completed_capabilities_)
      {
        event_->info("All started capabilities have been freed.");
      }
      else
      {
        free_capability_all(connection_map);
      }
    });
  }

  /**
   * @brief Request use of capability from capabilities2 server
   */
  void configure_capabilities(std::map<int, capabilities2::node_t>& capabilities)
  {
    auto request_configure = std::make_shared<ConfigureCapability::Request>();

    event_->info("Configuring capability of Runner " + std::to_string(completed_configurations_) + " named " +
                         capabilities[completed_configurations_].source.runner);

    if (xml_parser::convert_to_string(capabilities[completed_configurations_].source.parameters, request_configure->source.parameters))
    {
      request_configure->source.capability = capabilities[completed_configurations_].source.runner;
      request_configure->source.provider = capabilities[completed_configurations_].source.provider;
    }
    else
    {
      request_configure->source.capability = "";
      request_configure->source.provider = "";
    }

    if (xml_parser::convert_to_string(capabilities[completed_configurations_].target_on_start.parameters,
                                      request_configure->target_on_start.parameters))
    {
      request_configure->target_on_start.capability = capabilities[completed_configurations_].target_on_start.runner;
      request_configure->target_on_start.provider = capabilities[completed_configurations_].target_on_start.provider;
    }
    else
    {
      request_configure->target_on_start.capability = "";
      request_configure->target_on_start.provider = "";
    }

    if (xml_parser::convert_to_string(capabilities[completed_configurations_].target_on_stop.parameters,
                                      request_configure->target_on_stop.parameters))
    {
      request_configure->target_on_stop.capability = capabilities[completed_configurations_].target_on_stop.runner;
      request_configure->target_on_stop.provider = capabilities[completed_configurations_].target_on_stop.provider;
    }
    else
    {
      request_configure->target_on_stop.capability = "";
      request_configure->target_on_stop.provider = "";
    }

    if (xml_parser::convert_to_string(capabilities[completed_configurations_].target_on_success.parameters,
                                      request_configure->target_on_success.parameters))
    {
      request_configure->target_on_success.capability = capabilities[completed_configurations_].target_on_success.runner;
      request_configure->target_on_success.provider = capabilities[completed_configurations_].target_on_success.provider;
    }
    else
    {
      request_configure->target_on_success.capability = "";
      request_configure->target_on_success.provider = "";
    }

    if (xml_parser::convert_to_string(capabilities[completed_configurations_].target_on_failure.parameters,
                                      request_configure->target_on_failure.parameters))
    {
      request_configure->target_on_failure.capability = capabilities[completed_configurations_].target_on_failure.runner;
      request_configure->target_on_failure.provider = capabilities[completed_configurations_].target_on_failure.provider;
    }
    else
    {
      request_configure->target_on_failure.capability = "";
      request_configure->target_on_failure.provider = "";
    }

    std::string source_capability = capabilities[completed_configurations_].source.runner;

    // send the request
    auto result_future =
        conf_capability_client_->async_send_request(request_configure, [this, source_capability](ConfigureCapabilityClient::SharedFuture future) {
          if (!future.valid())
          {
            result_msg->success = false;
            result_msg->message = "Failed to configure capability :" + source_capability + ". Server execution cancelled";
            event_->error(result_msg->message);
            goal_handle_->abort(result_msg);
            return;
          }

          completed_configurations_++;

          auto response = future.get();

          event_->info(std::to_string(completed_configurations_) + "/" + std::to_string(expected_configurations_) +
                           " : Successfully configured capability : " + source_capability);

          // Check if all expected calls are completed before calling verify_plan
          if (completed_configurations_ == expected_configurations_)
          {
            event_->info("All requested capabilities have been configured. Triggering the first capability");

            trigger_first_node();
          }
          else
          {
            configure_capabilities(connection_map);
          }
        });
  }

  /**
   * @brief Trigger the first node
   */
  void trigger_first_node()
  {
    auto request_trigger = std::make_shared<TriggerCapability::Request>();

    std::string parameter_string;
    xml_parser::convert_to_string(connection_map[0].source.parameters, parameter_string);
    request_trigger->capability = connection_map[0].source.runner;
    request_trigger->parameters = parameter_string;

    // send the request
    auto result_future = trig_capability_client_->async_send_request(request_trigger, [this](TriggerCapabilityClient::SharedFuture future) {
      if (!future.valid())
      {
        result_msg->success = false;
        result_msg->message = "Failed to trigger capability " + connection_map[0].source.runner;
        event_->error(result_msg->message);
        goal_handle_->abort(result_msg);
        return;
      }

      auto response = future.get();
      event_->info("Successfully triggered capability " + connection_map[0].source.runner);

      result_msg->success = true;
      result_msg->message = "Successfully completed capabilities2 fabric";
      event_->info(result_msg->message);
      goal_handle_->succeed(result_msg);
    });
  }

  void check_service(bool wait_for_logic, const std::string& service_name)
  {
    while (wait_for_logic)
    {
      event_->error(service_name + " not available");
      rclcpp::shutdown();
      return;
    }
    event_->info(service_name + " connected");
  }

private:
  /** File Path link */
  std::string plan_file_path;

  /** Modified plan with closing capabilities */
  std::string modified_plan;

  /** flag to select loading from file or accepting via action server */
  bool read_file;

  int expected_interfaces_;
  int completed_interfaces_;

  int expected_providers_;
  int completed_providers_;

  int expected_capabilities_;
  int completed_capabilities_;
  int freed_capabilities_;

  int expected_configurations_;
  int completed_configurations_;

  /** Bond id */
  std::string bond_id_;

  /** Manages bond between capabilities server and this client */
  std::map<std::string, std::shared_ptr<BondClient>> bond_client_cache_;

  /** XML Document */
  tinyxml2::XMLDocument document;

  /** vector of connections */
  std::map<int, capabilities2::node_t> connection_map;

  /** Interface List */
  std::vector<bool> is_semantic_list;

  /** Interface List */
  std::vector<std::string> interface_list;

  /** Providers List */
  std::vector<std::string> providers_list;

  /** Control flow List */
  std::vector<std::string> control_tag_list;

  /** Invalid events list */
  std::vector<std::string> rejected_list;

  /** Feedback message for plan action server*/
  std::shared_ptr<Plan::Feedback> feedback_msg;

  /** Result message for plan action server*/
  std::shared_ptr<Plan::Result> result_msg;

  /** action server that exposes executor*/
  std::shared_ptr<rclcpp_action::Server<Plan>> planner_server_;

  /** action server goal handle*/
  std::shared_ptr<GoalHandlePlan> goal_handle_;

  /** Get interfaces from capabilities server */
  GetInterfacesClient::SharedPtr get_interfaces_client_;

  /** Get semantic interfaces from capabilities server */
  GetSemanticInterfacesClient::SharedPtr get_sem_interf_client_;

  /** Get providers from capabilities server */
  GetProvidersClient::SharedPtr get_providers_client_;

  /** establish bond */
  EstablishBondClient::SharedPtr establish_bond_client_;

  /** use an selected capability */
  UseCapabilityClient::SharedPtr use_capability_client_;

  /** free an selected capability */
  FreeCapabilityClient::SharedPtr free_capability_client_;

  /** configure an selected capability */
  ConfigureCapabilityClient::SharedPtr conf_capability_client_;

  /** trigger an selected capability */
  TriggerCapabilityClient::SharedPtr trig_capability_client_;

  /** Event client for publishing events */
  std::shared_ptr<EventClient> event_;

  /** capabilities2 server and fabric synchronization tools */
  // std::mutex mutex_;
  // std::condition_variable cv_;
  // bool fabric_completed_;
  // std::unique_lock<std::mutex> lock_;
};