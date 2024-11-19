#include <rclcpp/rclcpp.hpp>
#include <capabilities2_launch/launch_server.hpp>

int main(int argc, char **argv)
{
    // Initialize the ROS 2 system
    rclcpp::init(argc, argv);

    // Create an instance of the server node
    auto node = std::make_shared<LaunchServer>();

    // Spin the node to process requests
    rclcpp::spin(node);

    // Shutdown ROS 2
    rclcpp::shutdown();
    return 0;
}