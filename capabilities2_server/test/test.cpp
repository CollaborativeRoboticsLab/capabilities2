#include <cassert>
#include <cstdio>
#include <filesystem>
#include <iostream>
#include <algorithm>

#include <yaml-cpp/yaml.h>

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

int main()
{
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

  std::filesystem::remove(db_path);
  std::cout << "Resolved runnable spec test passed" << std::endl;
  return 0;
}
