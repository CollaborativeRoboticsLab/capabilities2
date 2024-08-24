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
  std::ifstream file("/home/ubuntu/colcon_ws/src/capabilities2/std_capabilities/interfaces/nav2.yaml");
  std::string data{ std::istreambuf_iterator<char>(file), std::istreambuf_iterator<char>() };

  // close file
  file.close();

  // print contents
  std::cout << data << std::endl;

  // parse yaml
  YAML::Node yml = YAML::Load(data);

  std::cout << yml["name"] << std::endl;

  rclcpp::shutdown();

  return 0;
}
