#include <iostream>
#include <rclcpp/rclcpp.hpp>
#include <capabilities2_server/capabilities_server.hpp>

using namespace capabilities2_server;

// testing units
int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);

  // capabilities server
  capabilities2_server::CapabilitiesServer node;

  // testing
  // open yaml file
  std::ifstream file("/home/ubuntu/colcon_ws/src/capabilities2/std_capabilities/providers/empty.yaml");
  std::string data{ std::istreambuf_iterator<char>(file), std::istreambuf_iterator<char>() };

  // close file
  file.close();

  // print contents
  std::cout << data << std::endl;

  // parse yaml
  YAML::Node yml = YAML::Load(data);
  // std::cout << yml["name"] << std::endl;

  capabilities2_msgs::msg::CapabilitySpec spec;
  spec.package = "std_capabilities";
  spec.type = capabilities2_msgs::msg::CapabilitySpec::CAPABILITY_PROVIDER;
  spec.content = data;

  for (const auto& s : node.get_sematic_interfaces("std_capabilities/empty"))
  {
    std::cout << s << std::endl;
  }

  for (const auto& p : node.get_providers("std_capabilities/empty", true))
  {
    std::cout << p << std::endl;
  }

  capabilities2_msgs::msg::CapabilitySpec s = node.get_capability_spec("std_capabilities/empty");
  std::cout << s.content << std::endl;
  std::cout << "\n" << std::endl;

  std::vector<capabilities2_msgs::msg::CapabilitySpec> specs = node.get_capability_specs();

  for (const auto& s : specs)
  {
    std::cout << "package: " << s.package << std::endl;
    std::cout << "type: " << s.type << std::endl;
    std::cout << "content: " << s.content << std::endl;
  }

  rclcpp::shutdown();

  return 0;
}
