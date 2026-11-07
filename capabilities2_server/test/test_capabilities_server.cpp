#include <cassert>
#include <cstdio>
#include <filesystem>
#include <iostream>
#include <algorithm>
#include <atomic>
#include <chrono>
#include <future>
#include <mutex>
#include <thread>

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>

#include <yaml-cpp/yaml.h>

#include <capabilities2_msgs/action/launch.hpp>
#include <capabilities2_server/capabilities_api.hpp>
#include <capabilities2_server/capabilities_db.hpp>
#include <capabilities2_server/models/interface.hpp>
#include <capabilities2_server/models/provider.hpp>
#include <capabilities2_server/models/semantic_interface.hpp>

namespace
{

capabilities2_server::models::interface_model_t load_interface(const std::string& yaml, const std::string& package)
{
  capabilities2_server::models::interface_model_t model;
  model.from_yaml(YAML::Load(yaml));
  model.header.name = package + "/" + model.header.name;
  return model;
}

capabilities2_server::models::semantic_interface_model_t load_semantic_interface(const std::string& yaml,
                                                                                 const std::string& package)
{
  capabilities2_server::models::semantic_interface_model_t model;
  model.from_yaml(YAML::Load(yaml));
  model.header.name = package + "/" + model.header.name;
  return model;
}

capabilities2_server::models::provider_model_t load_provider(const std::string& yaml, const std::string& package)
{
  capabilities2_server::models::provider_model_t model;
  model.from_yaml(YAML::Load(yaml));
  model.header.name = package + "/" + model.header.name;
  return model;
}

class ShutdownActionServer
{
public:
  using Launch = capabilities2_msgs::action::Launch;
  using GoalHandle = rclcpp_action::ServerGoalHandle<Launch>;

  ShutdownActionServer() : node(std::make_shared<rclcpp::Node>("shutdown_order_action_server"))
  {
    server = rclcpp_action::create_server<Launch>(
        node, "/shutdown_order_launch",
        [](const rclcpp_action::GoalUUID&, std::shared_ptr<const Launch::Goal>) {
          return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
        },
        [this](const std::shared_ptr<GoalHandle> goal) {
          if (reject_cancel.load())
            return rclcpp_action::CancelResponse::REJECT;
          const std::string provider = goal->get_goal()->reference_name;
          std::lock_guard<std::mutex> lock(mutex);
          cancellations.push_back(provider);
          cancel_threads.emplace_back([this, goal, provider]() {
            while (!goal->is_canceling() ||
                   (provider == "test_capabilities/OrderedNav" ? !allow_nav.load() : !allow_slam.load()))
              std::this_thread::sleep_for(std::chrono::milliseconds(10));
            auto result = std::make_shared<Launch::Result>();
            result->final_state = "canceled";
            goal->canceled(result);
          });
          return rclcpp_action::CancelResponse::ACCEPT;
        },
        [this](const std::shared_ptr<GoalHandle> goal) {
          std::lock_guard<std::mutex> lock(mutex);
          starts.push_back(goal->get_goal()->reference_name);
          goals.push_back(goal);
        });
    executor.add_node(node);
    executor_thread = std::thread([this]() { executor.spin(); });
  }

  ~ShutdownActionServer()
  {
    allow_nav = true;
    allow_slam = true;
    executor.cancel();
    executor_thread.join();
    for (auto& thread : cancel_threads)
      thread.join();
  }

  std::vector<std::string> get_cancellations()
  {
    std::lock_guard<std::mutex> lock(mutex);
    return cancellations;
  }

  std::vector<std::string> get_starts()
  {
    std::lock_guard<std::mutex> lock(mutex);
    return starts;
  }

  rclcpp::Node::SharedPtr node;
  rclcpp_action::Server<Launch>::SharedPtr server;
  rclcpp::executors::SingleThreadedExecutor executor;
  std::thread executor_thread;
  std::mutex mutex;
  std::vector<std::thread> cancel_threads;
  std::vector<std::string> cancellations;
  std::vector<std::string> starts;
  std::vector<std::shared_ptr<GoalHandle>> goals;
  std::atomic<bool> allow_nav{ false };
  std::atomic<bool> allow_slam{ false };
  std::atomic<bool> reject_cancel{ false };
};

bool wait_for_count(ShutdownActionServer& server, size_t count)
{
  for (int attempt = 0; attempt < 100; ++attempt)
  {
    if (server.get_cancellations().size() >= count)
      return true;
    std::this_thread::sleep_for(std::chrono::milliseconds(20));
  }
  return false;
}

}  // namespace

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);

  const std::filesystem::path db_path = std::filesystem::temp_directory_path() / "capabilities2_runnable_specs.sqlite3";
  std::filesystem::remove(db_path);

  capabilities2_server::CapabilitiesDB db(db_path.string());

  const std::string waypoint_interface_yaml = R"(
%YAML 1.1
---
name: WaypointRunner
spec_version: 1.2
spec_type: interface
description: Base waypoint interface.
interface:
  actions:
    "/navigate_to_pose":
      type: "nav2_msgs::action::NavigateToPose"
      description: "Navigate action"
)";

  const std::string turtlebot_waypoint_semantic_yaml = R"(
%YAML 1.1
---
name: TurtlebotWaypointRunner
spec_type: semantic_interface
spec_version: 1.2
description: TurtleBot waypoint wrapper.
redefines: test_capabilities/WaypointRunner
global_namespace: turtlebot4
remappings:
  actions:
    "/navigate_to_pose": "/turtlebot4/navigate_to_pose"
)";

  const std::string waypoint_provider_yaml = R"(
%YAML 1.1
---
name: WaypointRunner
spec_type: provider
spec_version: 1.2
description: Base waypoint provider.
implements: test_capabilities/WaypointRunner
runner: capabilities2_runner::WaypointRunner
depends_on:
  "test_capabilities/RobotPoseRunner":
    provider: "test_capabilities/RobotPoseRunner"
definition:
  command: "<Runner interface=test_capabilities/WaypointRunner provider=test_capabilities/WaypointRunner x='$x' y='$y' />"
  configuration_parameters:
    - name: x
      type: double
      description: X target.
    - name: y
      type: double
      description: Y target.
)";

  const std::string robotpose_interface_yaml = R"(
%YAML 1.1
---
name: RobotPoseRunner
spec_version: 1.2
spec_type: interface
description: Base robot pose interface.
)";

  const std::string turtlebot_robotpose_semantic_yaml = R"(
%YAML 1.1
---
name: TurtlebotRobotposeRunner
spec_type: semantic_interface
spec_version: 1.2
description: TurtleBot robot pose wrapper.
redefines: test_capabilities/RobotPoseRunner
global_namespace: turtlebot4
)";

  const std::string semantic_without_namespace_yaml = R"(
%YAML 1.1
---
name: GenericWaypointRunner
spec_type: semantic_interface
spec_version: 1.2
description: Generic waypoint wrapper without an explicit global namespace.
redefines: test_capabilities/WaypointRunner
)";

  const std::string other_robotpose_semantic_yaml = R"(
%YAML 1.1
---
name: OtherRobotposeRunner
spec_type: semantic_interface
spec_version: 1.2
description: Another robot pose wrapper.
redefines: test_capabilities/RobotPoseRunner
)";

  const std::string robotpose_provider_yaml = R"(
%YAML 1.1
---
name: RobotPoseRunner
spec_type: provider
spec_version: 1.2
description: Base robot pose provider.
implements: test_capabilities/RobotPoseRunner
runner: capabilities2_runner::RobotPoseRunner
definition:
  command: "<Runner interface=test_capabilities/RobotPoseRunner provider=test_capabilities/RobotPoseRunner map='$map' odom='$odom' robot='$robot' time_buffer='$time_buffer' />"
  configuration_parameters:
    - name: map
      type: string
      description: Map frame.
    - name: odom
      type: string
      description: Odom frame.
    - name: robot
      type: string
      description: Robot frame.
)";

  const std::string plain_interface_yaml = R"(
%YAML 1.1
---
name: PlainRunner
spec_version: 1.2
spec_type: interface
description: Base plain interface.
)";

  const std::string turtlebot_robotpose_provider_yaml = R"(
%YAML 1.1
---
name: TurtlebotRobotposeRunner
spec_type: provider
spec_version: 1.2
description: TurtleBot robot pose provider.
implements: test_semantic_capabilities/TurtlebotRobotposeRunner
runner: capabilities2_runner::RobotPoseRunner
depends_on:
  "nav2_capabilities/Nav2LaunchRunner":
    provider: "turtlebot_capabilities/TurtlebotNav2LaunchRunner"
definition:
  command: "<Runner interface=test_semantic_capabilities/TurtlebotRobotposeRunner provider=turtlebot_capabilities/TurtlebotRobotposeRunner />"
)";

  const std::string plain_provider_yaml = R"(
%YAML 1.1
---
name: PlainRunner
spec_type: provider
spec_version: 1.2
description: Base plain provider.
implements: plain_capabilities/PlainRunner
runner: capabilities2_runner::PlainRunner
definition:
  command: "<Runner interface=plain_capabilities/PlainRunner provider=plain_capabilities/PlainRunner value='$value' />"
  configuration_parameters:
    - name: value
      type: string
      description: Plain value.
)";

  const std::string second_robotpose_provider_yaml = R"(
%YAML 1.1
---
name: AlternateRobotposeRunner
spec_type: provider
spec_version: 1.2
description: Another provider for the TurtleBot robot pose wrapper.
implements: test_semantic_capabilities/TurtlebotRobotposeRunner
runner: capabilities2_runner::RobotPoseRunner
definition:
  command: "<Runner interface=test_semantic_capabilities/TurtlebotRobotposeRunner provider=turtlebot_capabilities/AlternateRobotposeRunner />"
)";

  db.insert_interface(load_interface(waypoint_interface_yaml, "test_capabilities"));
  db.insert_provider(load_provider(waypoint_provider_yaml, "test_capabilities"));
  const auto generic_only_specs = db.get_runnable_specs();
  assert(generic_only_specs.size() == 1);
  assert(generic_only_specs.front().interface.name == "test_capabilities/WaypointRunner");
  assert(generic_only_specs.front().provider.name == "test_capabilities/WaypointRunner");
  db.insert_semantic_interface(load_semantic_interface(turtlebot_waypoint_semantic_yaml, "test_semantic_capabilities"));
  db.insert_interface(load_interface(robotpose_interface_yaml, "test_capabilities"));
  db.insert_semantic_interface(load_semantic_interface(turtlebot_robotpose_semantic_yaml, "test_semantic_"
                                                                                          "capabilities"));
  db.insert_semantic_interface(load_semantic_interface(other_robotpose_semantic_yaml, "test_semantic_capabilities"));
  db.insert_semantic_interface(load_semantic_interface(semantic_without_namespace_yaml, "test_semantic_capabilities"));
  db.insert_provider(load_provider(robotpose_provider_yaml, "test_capabilities"));
  db.insert_provider(load_provider(turtlebot_robotpose_provider_yaml, "turtlebot_capabilities"));
  db.insert_provider(load_provider(second_robotpose_provider_yaml, "turtlebot_capabilities"));
  db.insert_interface(load_interface(plain_interface_yaml, "plain_capabilities"));
  db.insert_provider(load_provider(plain_provider_yaml, "plain_capabilities"));

  const auto runnable_specs = db.get_runnable_specs();
  assert(runnable_specs.size() == 6);

  const auto find_runnable = [&runnable_specs](const std::string& interface_name, const std::string& provider_name) {
    return std::find_if(runnable_specs.begin(), runnable_specs.end(),
                        [&interface_name, &provider_name](const auto& runnable) {
                          return runnable.interface.name == interface_name && runnable.provider.name == provider_name;
                        });
  };

  const auto turtlebot_waypoint = find_runnable("test_semantic_capabilities/TurtlebotWaypointRunner", "test_"
                                                                                                      "capabilities/"
                                                                                                      "WaypointRunner");
  assert(turtlebot_waypoint != runnable_specs.end());
  assert(turtlebot_waypoint->command.find("interface=test_semantic_capabilities/TurtlebotWaypointRunner") !=
         std::string::npos);
  assert(turtlebot_waypoint->command.find("provider=test_capabilities/WaypointRunner") != std::string::npos);
  const auto generic_waypoint_run_config = db.get_run_config("test_capabilities/WaypointRunner", "test_semantic_"
                                                                                                 "capabilities/"
                                                                                                 "GenericWaypointRunne"
                                                                                                 "r");
  assert(generic_waypoint_run_config.is_valid());
  assert(generic_waypoint_run_config.global_namespace.empty());
  const auto has_waypoint_relation = [&turtlebot_waypoint](const std::string& subject, const std::string& predicate,
                                                           const std::string& object) {
    return std::find_if(
               turtlebot_waypoint->relations.begin(), turtlebot_waypoint->relations.end(), [&](const auto& relation) {
                 return relation.subject == subject && relation.predicate == predicate && relation.object == object;
               }) != turtlebot_waypoint->relations.end();
  };
  assert(has_waypoint_relation("test_semantic_capabilities/TurtlebotWaypointRunner", "depends_on",
                               "test_capabilities/RobotPoseRunner"));

  assert(find_runnable("test_capabilities/WaypointRunner", "test_capabilities/WaypointRunner") == runnable_specs.end());
  assert(find_runnable("test_capabilities/RobotPoseRunner", "test_capabilities/RobotPoseRunner") == runnable_specs.end());
    assert(find_runnable("test_semantic_capabilities/OtherRobotposeRunner", "test_capabilities/RobotPoseRunner") !=
      runnable_specs.end());
    assert(find_runnable("test_semantic_capabilities/OtherRobotposeRunner",
          "turtlebot_capabilities/TurtlebotRobotposeRunner") == runnable_specs.end());
  const auto turtlebot_robotpose = find_runnable("test_semantic_capabilities/TurtlebotRobotposeRunner", "test_"
                                                                                                        "capabilities/"
                                                                                                        "RobotPoseRunne"
                                                                                                        "r");
    assert(turtlebot_robotpose == runnable_specs.end());

  const auto direct_robotpose = find_runnable("test_semantic_capabilities/TurtlebotRobotposeRunner",
                                               "turtlebot_capabilities/TurtlebotRobotposeRunner");
  assert(direct_robotpose != runnable_specs.end());
  assert(find_runnable("test_semantic_capabilities/TurtlebotRobotposeRunner",
                       "turtlebot_capabilities/AlternateRobotposeRunner") != runnable_specs.end());
  const auto direct_robotpose_config = db.get_run_config("turtlebot_capabilities/TurtlebotRobotposeRunner",
                                                         "test_semantic_capabilities/TurtlebotRobotposeRunner");
  assert(direct_robotpose_config.is_valid());
  assert(direct_robotpose_config.global_namespace == "turtlebot4");
  const auto implicit_robotpose_config = db.get_run_config("turtlebot_capabilities/TurtlebotRobotposeRunner");
  assert(implicit_robotpose_config.global_namespace == "turtlebot4");
  const auto direct_robotpose_running = db.get_running("turtlebot_capabilities/TurtlebotRobotposeRunner");
  assert(direct_robotpose_running.interface == "test_semantic_capabilities/TurtlebotRobotposeRunner");
  assert(direct_robotpose_running.dependencies.size() == 1);
  assert(direct_robotpose_running.dependencies[0].interface == "nav2_capabilities/Nav2LaunchRunner");
  assert(direct_robotpose_running.dependencies[0].provider == "turtlebot_capabilities/TurtlebotNav2LaunchRunner");

  const auto plain_runnable = find_runnable("plain_capabilities/PlainRunner", "plain_capabilities/PlainRunner");
  assert(plain_runnable != runnable_specs.end());
  assert(plain_runnable->command.find("interface=plain_capabilities/PlainRunner") != std::string::npos);
  assert(plain_runnable->command.find("provider=plain_capabilities/PlainRunner") != std::string::npos);

  const std::string nav2_launch_interface_yaml = R"(
%YAML 1.1
---
name: Nav2LaunchRunner
spec_version: 1.2
spec_type: interface
description: Launches Nav2.
)";

  const std::string nav2_launch_provider_yaml = R"(
%YAML 1.1
---
name: Nav2LaunchRunner
spec_type: provider
spec_version: 1.2
description: Launch provider used by navigation capabilities.
implements: nav2_capabilities/Nav2LaunchRunner
runner: capabilities2_runner::DummyRunner
definition:
  launch_parameters:
    - key: use_sim_time
      value: "true"
    - key: autostart
      value: "false"
  launch_readiness:
    lifecycle:
      nodes: [/planner_server, /bt_navigator]
      timeout_sec: 45.0
    action:
      name: navigate_to_pose
      type: nav2_msgs/action/NavigateToPose
      timeout: 12.0
    service:
      name: clear_costmap
      type: std_srvs/srv/Empty
      timeout: 13.0
    topic:
      name: scan
      type: sensor_msgs/msg/LaserScan
      timeout: 14.0
)";

  const std::string nav2_waypoint_interface_yaml = R"(
%YAML 1.1
---
name: WaypointRunner
spec_version: 1.2
spec_type: interface
description: Navigates to a target pose.
)";

  const std::string nav2_waypoint_provider_yaml = R"(
%YAML 1.1
---
name: WaypointRunner
spec_type: provider
spec_version: 1.2
description: Waypoint provider requiring Nav2 launch.
implements: nav2_capabilities/WaypointRunner
runner: capabilities2_runner::DummyRunner
depends_on:
  "nav2_capabilities/Nav2LaunchRunner":
    provider: "nav2_capabilities/Nav2LaunchRunner"
)";

  db.insert_interface(load_interface(nav2_launch_interface_yaml, "nav2_capabilities"));
  db.insert_provider(load_provider(nav2_launch_provider_yaml, "nav2_capabilities"));
  db.insert_interface(load_interface(nav2_waypoint_interface_yaml, "nav2_capabilities"));
  db.insert_provider(load_provider(nav2_waypoint_provider_yaml, "nav2_capabilities"));

  const std::string second_parent_interface_yaml = R"(
name: SecondParent
spec_version: 1.2
spec_type: interface
description: Another capability requiring Nav2 launch.
)";
  const std::string second_parent_provider_yaml = R"(
name: SecondParent
spec_version: 1.2
spec_type: provider
description: Another provider requiring Nav2 launch.
implements: test_capabilities/SecondParent
runner: capabilities2_runner::DummyRunner
depends_on:
  nav2_capabilities/Nav2LaunchRunner:
    provider: nav2_capabilities/Nav2LaunchRunner
)";
  db.insert_interface(load_interface(second_parent_interface_yaml, "test_capabilities"));
  db.insert_provider(load_provider(second_parent_provider_yaml, "test_capabilities"));

  const std::string failing_interface_yaml = R"(
name: ZFailingLaunch
spec_version: 1.2
spec_type: interface
description: Launches a capability with invalid readiness.
)";
  const std::string failing_provider_yaml = R"(
name: ZFailingLaunch
spec_version: 1.2
spec_type: provider
description: Invalid readiness timeout.
implements: test_capabilities/ZFailingLaunch
runner: demo.launch.py
definition:
  launch_readiness:
    lifecycle:
      nodes: [test_node]
      timeout_sec: -1
)";
  const std::string dependent_interface_yaml = R"(
name: Dependent
spec_version: 1.2
spec_type: interface
description: Capability with multiple dependencies.
)";
  const std::string dependent_provider_yaml = R"(
name: Dependent
spec_version: 1.2
spec_type: provider
description: Capability with one failing dependency.
implements: test_capabilities/Dependent
runner: capabilities2_runner::DummyRunner
depends_on:
  nav2_capabilities/Nav2LaunchRunner:
    provider: nav2_capabilities/Nav2LaunchRunner
  test_capabilities/ZFailingLaunch:
    provider: test_capabilities/ZFailingLaunch
)";
  db.insert_interface(load_interface(failing_interface_yaml, "test_capabilities"));
  db.insert_provider(load_provider(failing_provider_yaml, "test_capabilities"));
  db.insert_interface(load_interface(dependent_interface_yaml, "test_capabilities"));
  db.insert_provider(load_provider(dependent_provider_yaml, "test_capabilities"));

  const auto nav2_launch_run_config = db.get_run_config("nav2_capabilities/Nav2LaunchRunner");
  const auto nav2_launch_runner_opts = nav2_launch_run_config.to_runner_opts();
  assert(nav2_launch_runner_opts.launch_parameters.size() == 2);
  assert(nav2_launch_runner_opts.launch_parameters[0].key == "use_sim_time");
  assert(nav2_launch_runner_opts.launch_parameters[0].value == "true");
  assert(nav2_launch_runner_opts.launch_parameters[1].key == "autostart");
  assert(nav2_launch_runner_opts.launch_parameters[1].value == "false");
  assert(nav2_launch_runner_opts.launch_readiness.lifecycle_nodes ==
         std::vector<std::string>({ "/planner_server", "/bt_navigator" }));
  assert(nav2_launch_runner_opts.launch_readiness.lifecycle_timeout_sec == 45.0);
  assert(nav2_launch_runner_opts.launch_readiness.action.name == "navigate_to_pose");
  assert(nav2_launch_runner_opts.launch_readiness.action.type == "nav2_msgs/action/NavigateToPose");
  assert(nav2_launch_runner_opts.launch_readiness.action.timeout == 12.0);
  assert(nav2_launch_runner_opts.launch_readiness.service.name == "clear_costmap");
  assert(nav2_launch_runner_opts.launch_readiness.service.type == "std_srvs/srv/Empty");
  assert(nav2_launch_runner_opts.launch_readiness.service.timeout == 13.0);
  assert(nav2_launch_runner_opts.launch_readiness.topic.name == "scan");
  assert(nav2_launch_runner_opts.launch_readiness.topic.type == "sensor_msgs/msg/LaserScan");
  assert(nav2_launch_runner_opts.launch_readiness.topic.timeout == 14.0);

  capabilities2_server::CapabilitiesAPI api;
  auto node = std::make_shared<rclcpp::Node>("capabilities2_server_runtime_test");
  api.connect(db_path.string(), node->get_node_logging_interface());

  const bool started =
      api.use_capability(node, "nav2_capabilities/WaypointRunner", "nav2_capabilities/WaypointRunner", "bond-nav2");
  assert(started);

  const auto running_capabilities = api.get_running_capabilities();
  assert(running_capabilities.size() == 2);

  const auto running_waypoint =
      std::find_if(running_capabilities.begin(), running_capabilities.end(),
                   [](const auto& cap) { return cap.capability.capability == "nav2_capabilities/WaypointRunner"; });
  assert(running_waypoint != running_capabilities.end());
  assert(running_waypoint->dependent_capabilities.size() == 1);
  assert(running_waypoint->dependent_capabilities.front().capability == "nav2_capabilities/Nav2LaunchRunner");
  assert(running_waypoint->dependent_capabilities.front().provider == "nav2_capabilities/Nav2LaunchRunner");

  const auto running_nav2 = std::find_if(running_capabilities.begin(), running_capabilities.end(), [](const auto& cap) {
    return cap.capability.capability == "nav2_capabilities/Nav2LaunchRunner";
  });
  assert(running_nav2 != running_capabilities.end());

  assert(
      api.use_capability(node, "nav2_capabilities/WaypointRunner", "nav2_capabilities/WaypointRunner", "bond-shared"));
  assert(
      api.use_capability(node, "nav2_capabilities/WaypointRunner", "nav2_capabilities/WaypointRunner", "bond-shared"));
  assert(!api.use_capability(node, "nav2_capabilities/WaypointRunner", "test_capabilities/OtherProvider",
                             "bond-conflict"));
  capabilities2_msgs::msg::CapabilityConnection connection;
  connection.source.capability = "nav2_capabilities/WaypointRunner";
  connection.source.instance_id = "source";
  connection.target.capability = "nav2_capabilities/WaypointRunner";
  connection.target.instance_id = "target";
  assert(api.connect_capability("bond-nav2", connection));
  assert(api.connect_capability("bond-shared", connection));
  api.free_capability("nav2_capabilities/WaypointRunner", "bond-nav2");
  assert(api.get_running_capabilities().size() == 2);
  assert(api.use_capability(node, "nav2_capabilities/WaypointRunner", "nav2_capabilities/WaypointRunner", "bond-nav2"));
  assert(api.connect_capability("bond-nav2", connection));
  api.free_capability("nav2_capabilities/WaypointRunner", "bond-nav2");
  api.free_capability("nav2_capabilities/WaypointRunner", "bond-shared");
  assert(api.get_running_capabilities().empty());

  assert(
      api.use_capability(node, "nav2_capabilities/WaypointRunner", "nav2_capabilities/WaypointRunner", "bond-first"));
  assert(api.use_capability(node, "test_capabilities/SecondParent", "test_capabilities/SecondParent", "bond-second"));
  assert(api.get_running_capabilities().size() == 3);
  assert(!api.use_capability(node, "test_capabilities/Dependent", "test_capabilities/Dependent", "bond-failure"));
  assert(api.get_running_capabilities().size() == 3);
  api.on_bond_broken("bond-first");
  assert(api.get_running_capabilities().size() == 2);
  api.on_bond_broken("bond-first");
  assert(api.get_running_capabilities().size() == 2);
  api.on_bond_broken("bond-second");
  assert(api.get_running_capabilities().empty());

  for (int attempt = 0; attempt < 2; ++attempt)
  {
    const bool dependent_started =
        api.use_capability(node, "test_capabilities/Dependent", "test_capabilities/Dependent", "bond-failure");
    assert(!dependent_started);
    assert(api.get_running_capabilities().empty());
  }

  const std::string ordered_slam_interface = R"(
name: OrderedSlam
spec_version: 1.2
spec_type: interface
description: Ordered SLAM launch.
)";
  const std::string ordered_slam_provider = R"(
name: OrderedSlam
spec_type: provider
spec_version: 1.2
description: Ordered SLAM launch.
implements: test_capabilities/OrderedSlam
runner: slam.launch.py
)";
  const std::string ordered_nav_interface = R"(
name: OrderedNav
spec_version: 1.2
spec_type: interface
description: Ordered Nav2 launch.
)";
  const std::string ordered_nav_provider = R"(
name: OrderedNav
spec_type: provider
spec_version: 1.2
description: Ordered Nav2 launch.
implements: test_capabilities/OrderedNav
runner: nav.launch.py
depends_on:
  test_capabilities/OrderedSlam:
    provider: test_capabilities/OrderedSlam
)";
  const std::string failing_nav_interface = R"(
name: OrderedFailingNav
spec_version: 1.2
spec_type: interface
description: Launch with failing readiness.
)";
  const std::string failing_nav_provider = R"(
name: OrderedFailingNav
spec_type: provider
spec_version: 1.2
description: Launch with failing readiness.
implements: test_capabilities/OrderedFailingNav
runner: failing_nav.launch.py
depends_on:
  test_capabilities/OrderedSlam:
    provider: test_capabilities/OrderedSlam
definition:
  launch_readiness:
    lifecycle:
      nodes: [missing]
      timeout_sec: 0.3
)";
  db.insert_interface(load_interface(ordered_slam_interface, "test_capabilities"));
  db.insert_provider(load_provider(ordered_slam_provider, "test_capabilities"));
  db.insert_interface(load_interface(ordered_nav_interface, "test_capabilities"));
  db.insert_provider(load_provider(ordered_nav_provider, "test_capabilities"));
  db.insert_interface(load_interface(failing_nav_interface, "test_capabilities"));
  db.insert_provider(load_provider(failing_nav_provider, "test_capabilities"));

  {
    ShutdownActionServer launch_server;
    node->declare_parameter("launch_action_name", "/shutdown_order_launch");
    assert(api.use_capability(node, "test_capabilities/OrderedNav", "test_capabilities/OrderedNav", "first"));
    auto released = std::async(std::launch::async, [&api]() {
      api.free_capability("test_capabilities/OrderedNav", "first");
    });
    assert(wait_for_count(launch_server, 1));
    assert(launch_server.get_cancellations().front() == "test_capabilities/OrderedNav");
    auto restarted = std::async(std::launch::async, [&api, &node]() {
      return api.use_capability(node, "test_capabilities/OrderedNav", "test_capabilities/OrderedNav", "second");
    });
    assert(restarted.wait_for(std::chrono::milliseconds(200)) == std::future_status::timeout);
    assert(launch_server.get_cancellations().size() == 1);
    assert(launch_server.get_starts().size() == 2);
    launch_server.allow_nav = true;
    assert(wait_for_count(launch_server, 2));
    assert(launch_server.get_cancellations().back() == "test_capabilities/OrderedSlam");
    assert(restarted.wait_for(std::chrono::milliseconds(200)) == std::future_status::timeout);
    assert(launch_server.get_starts().size() == 2);
    launch_server.allow_slam = true;
    released.get();
    assert(restarted.get());
    assert(launch_server.get_starts() == std::vector<std::string>({
        "test_capabilities/OrderedSlam", "test_capabilities/OrderedNav",
        "test_capabilities/OrderedSlam", "test_capabilities/OrderedNav" }));
    api.free_capability("test_capabilities/OrderedNav", "second");
    assert(api.get_running_capabilities().empty());

    launch_server.reject_cancel = true;
    assert(!api.use_capability(node, "test_capabilities/OrderedFailingNav",
                               "test_capabilities/OrderedFailingNav", "failed-start"));
    assert(api.get_running_capabilities().size() == 2);
    assert(!api.use_capability(node, "test_capabilities/OrderedFailingNav",
                               "test_capabilities/OrderedFailingNav", "retry"));
    launch_server.reject_cancel = false;
    api.stop_capability("test_capabilities/OrderedFailingNav");
    assert(api.get_running_capabilities().empty());
  }

  std::filesystem::remove(db_path);
  rclcpp::shutdown();
  std::cout << "Resolved runnable spec and dependency runtime tests passed" << std::endl;
  return 0;
}
