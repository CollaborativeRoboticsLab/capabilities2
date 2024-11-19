#pragma once

#include <memory>
#include <iostream>
#include <unistd.h>
#include <signal.h>
#include <cstring>
#include <sys/wait.h>
#include <sys/types.h>
#include <filesystem>

#include <rclcpp/rclcpp.hpp>
#include <capabilities2_msgs/srv/launch_start.hpp>
#include <capabilities2_msgs/srv/launch_stop.hpp>

class LaunchServer : public rclcpp::Node
{
public:
  LaunchServer() : Node("Launch_File_Server")
  {
    start_server_ = this->create_service<capabilities2_msgs::srv::LaunchStart>(
        "/capabilities/launch_start",
        std::bind(&LaunchServer::start_request, this, std::placeholders::_1, std::placeholders::_2));

    stop_server_ = this->create_service<capabilities2_msgs::srv::LaunchStop>(
        "/capabilities/launch_stop",
        std::bind(&LaunchServer::stop_request, this, std::placeholders::_1, std::placeholders::_2));

    RCLCPP_INFO(this->get_logger(), "LaunchFileServer is ready.");
  }

private:
  // Callback function to handle requests
  void start_request(const std::shared_ptr<capabilities2_msgs::srv::LaunchStart::Request> request,
                     const std::shared_ptr<capabilities2_msgs::srv::LaunchStart::Response> response)
  {
    package_name = request->package_name;
    launch_name = request->launch_file_name;

    RCLCPP_INFO(this->get_logger(), "Received request: package = %s, launch file = %s", package_name.c_str(),
                launch_name.c_str());

    std::string command = "/bin/bash -c \"source install/setup.bash && ros2 launch " + package_name + " " + launch_name + "\"";

    // Execute the command using system()
    int status = system(command.c_str());
    if (status == -1)
    {
      std::string error_msg = "Failed to start " + launch_name + " from " + package_name;
      RCLCPP_ERROR(this->get_logger(), error_msg.c_str());
      throw std::runtime_error(error_msg);
    }

    // Since `system()` doesn't return the PID directly, consider improving the mechanism if PID tracking is essential.
    pid = getpid(); // This is just an example and may need improvement depending on the need.

    response->pid = pid;
  }

  void stop_request(const std::shared_ptr<capabilities2_msgs::srv::LaunchStop::Request> request,
                    const std::shared_ptr<capabilities2_msgs::srv::LaunchStop::Response> response)
  {
    pid = request->pid;

    RCLCPP_INFO(this->get_logger(), "Received request: stopping PID = %d", pid);

    if (pid != -1)
    {
      if (kill(pid, SIGTERM) == 0)
      {
        RCLCPP_INFO(this->get_logger(), "Sent SIGTERM to PID %d", pid);
      }
      else
      {
        RCLCPP_WARN(this->get_logger(), "Failed to terminate PID %d with SIGTERM", pid);
      }
    }
  }

  // Service server
  rclcpp::Service<capabilities2_msgs::srv::LaunchStart>::SharedPtr start_server_;
  rclcpp::Service<capabilities2_msgs::srv::LaunchStop>::SharedPtr stop_server_;

  std::string launch_name;
  std::string package_name;

  int pid;
};