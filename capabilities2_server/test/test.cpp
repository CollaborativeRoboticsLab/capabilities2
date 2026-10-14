#include <cassert>
#include <cstdio>
#include <filesystem>
#include <iostream>
#include <algorithm>

#include <rclcpp/rclcpp.hpp>

#include <yaml-cpp/yaml.h>

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

  db.insert_interface(load_interface(waypoint_interface_yaml, "test_capabilities"));
  db.insert_semantic_interface(load_semantic_interface(turtlebot_waypoint_semantic_yaml, "test_semantic_capabilities"));
  db.insert_provider(load_provider(waypoint_provider_yaml, "test_capabilities"));
  db.insert_interface(load_interface(robotpose_interface_yaml, "test_capabilities"));
  db.insert_semantic_interface(load_semantic_interface(turtlebot_robotpose_semantic_yaml, "test_semantic_capabilities"));
  db.insert_provider(load_provider(robotpose_provider_yaml, "test_capabilities"));
  db.insert_interface(load_interface(plain_interface_yaml, "plain_capabilities"));
  db.insert_provider(load_provider(plain_provider_yaml, "plain_capabilities"));

  const auto runnable_specs = db.get_runnable_specs();
  assert(runnable_specs.size() == 3);

  const auto find_runnable = [&runnable_specs](const std::string& interface_name, const std::string& provider_name) {
    return std::find_if(runnable_specs.begin(), runnable_specs.end(),
                        [&interface_name, &provider_name](const auto& runnable) {
                          return runnable.interface.name == interface_name && runnable.provider.name == provider_name;
                        });
  };

  const auto turtlebot_waypoint =
      find_runnable("test_semantic_capabilities/TurtlebotWaypointRunner", "test_capabilities/WaypointRunner");
  assert(turtlebot_waypoint != runnable_specs.end());
  assert(turtlebot_waypoint->command.find("interface=test_semantic_capabilities/TurtlebotWaypointRunner") !=
         std::string::npos);
  assert(turtlebot_waypoint->command.find("provider=test_capabilities/WaypointRunner") != std::string::npos);
  const auto has_waypoint_relation = [&turtlebot_waypoint](const std::string& subject, const std::string& predicate,
                                                           const std::string& object) {
    return std::find_if(turtlebot_waypoint->relations.begin(), turtlebot_waypoint->relations.end(),
                        [&](const auto& relation) {
                          return relation.subject == subject && relation.predicate == predicate &&
                                 relation.object == object;
                        }) != turtlebot_waypoint->relations.end();
  };
  assert(has_waypoint_relation("test_semantic_capabilities/TurtlebotWaypointRunner", "depends_on",
                               "test_capabilities/RobotPoseRunner"));

  const auto turtlebot_robotpose =
      find_runnable("test_semantic_capabilities/TurtlebotRobotposeRunner", "test_capabilities/RobotPoseRunner");
  assert(turtlebot_robotpose != runnable_specs.end());
  assert(turtlebot_robotpose->command.find("interface=test_semantic_capabilities/TurtlebotRobotposeRunner") !=
         std::string::npos);
  assert(turtlebot_robotpose->command.find("provider=test_capabilities/RobotPoseRunner") != std::string::npos);

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

  const auto nav2_launch_run_config = db.get_run_config("nav2_capabilities/Nav2LaunchRunner");
  const auto nav2_launch_runner_opts = nav2_launch_run_config.to_runner_opts();
  assert(nav2_launch_runner_opts.launch_parameters.size() == 2);
  assert(nav2_launch_runner_opts.launch_parameters[0].key == "use_sim_time");
  assert(nav2_launch_runner_opts.launch_parameters[0].value == "true");
  assert(nav2_launch_runner_opts.launch_parameters[1].key == "autostart");
  assert(nav2_launch_runner_opts.launch_parameters[1].value == "false");

  capabilities2_server::CapabilitiesAPI api;
  auto node = std::make_shared<rclcpp::Node>("capabilities2_server_runtime_test");
  api.connect(db_path.string(), node->get_node_logging_interface());

  const bool started = api.use_capability(node, "nav2_capabilities/WaypointRunner",
                                          "nav2_capabilities/WaypointRunner", "bond-nav2");
  assert(started);

  const auto running_capabilities = api.get_running_capabilities();
  assert(running_capabilities.size() == 2);

  const auto running_waypoint = std::find_if(running_capabilities.begin(), running_capabilities.end(),
                                             [](const auto& cap) {
                                               return cap.capability.capability == "nav2_capabilities/WaypointRunner";
                                             });
  assert(running_waypoint != running_capabilities.end());
  assert(running_waypoint->dependent_capabilities.size() == 1);
  assert(running_waypoint->dependent_capabilities.front().capability == "nav2_capabilities/Nav2LaunchRunner");
  assert(running_waypoint->dependent_capabilities.front().provider == "nav2_capabilities/Nav2LaunchRunner");

  const auto running_nav2 = std::find_if(running_capabilities.begin(), running_capabilities.end(), [](const auto& cap) {
    return cap.capability.capability == "nav2_capabilities/Nav2LaunchRunner";
  });
  assert(running_nav2 != running_capabilities.end());

  api.free_capability("nav2_capabilities/WaypointRunner", "bond-nav2");
  assert(api.get_running_capabilities().empty());

  std::filesystem::remove(db_path);
  rclcpp::shutdown();
  std::cout << "Resolved runnable spec and dependency runtime tests passed" << std::endl;
  return 0;
}
