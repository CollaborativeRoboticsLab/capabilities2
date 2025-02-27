#pragma once

#include <thread>

#include <tinyxml2.h>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>

#include <geometry_msgs/msg/pose_stamped.hpp>
#include <nav2_msgs/action/follow_waypoints.hpp>

#include <capabilities2_runner/action_runner.hpp>

namespace capabilities2_runner
{

/**
 * @brief action runner base class
 *
 * Class to run waypointfollower action based capability
 *
 */
class WayPointRunner : public ActionRunner<nav2_msgs::action::FollowWaypoints>
{
public:
  WayPointRunner() : ActionRunner()
  {
  }

  /**
   * @brief Starter function for starting the action runner
   *
   * @param node shared pointer to the capabilities node. Allows to use ros node related functionalities
   * @param run_config runner configuration loaded from the yaml file
   * @param on_started function pointer to trigger at the start of the action client in the runner
   * @param on_terminated function pointer to trigger at the termination of the action client in the runner
   */
  virtual void start(rclcpp::Node::SharedPtr node, const runner_opts& run_config,
                     std::function<void(const std::string&)> on_started = nullptr,
                     std::function<void(const std::string&)> on_terminated = nullptr,
                     std::function<void(const std::string&)> on_stopped = nullptr) override
  {
    init_action(node, run_config, "follow_waypoints", on_started, on_terminated, on_stopped);
  }

  /**
   * @brief trigger the runner
   *
   @param parameters XMLElement that contains parameters in the format '<waypointfollower x='$value' y='$value' />'
  */
  virtual std::optional<std::function<void(std::shared_ptr<tinyxml2::XMLElement>)>>
  trigger(std::shared_ptr<tinyxml2::XMLElement> parameters = nullptr)
  {
    tinyxml2::XMLElement* parametersElement = parameters->FirstChildElement("waypointfollower");

    parametersElement->QueryDoubleAttribute("x", &x);
    parametersElement->QueryDoubleAttribute("y", &y);

    nav2_msgs::action::FollowWaypoints::Goal goal_msg;
    geometry_msgs::msg::PoseStamped pose_msg;

    global_frame_ = "map";
    robot_base_frame_ = "base_link";

    pose_msg.header.stamp = node_->get_clock()->now();
    pose_msg.header.frame_id = global_frame_;
    pose_msg.pose.position.x = x;
    pose_msg.pose.position.y = y;
    pose_msg.pose.position.z = 0.0;

    goal_msg.poses.push_back(pose_msg);

    auto goal_handle_future = action_client_->async_send_goal(goal_msg);

    return [this, goal_handle_future](std::shared_ptr<tinyxml2::XMLElement> result) {
      if (rclcpp::spin_until_future_complete(node_, goal_handle_future) != rclcpp::FutureReturnCode::SUCCESS)
      {
        RCLCPP_ERROR(node_->get_logger(), "send goal call failed");
        return;
      }

      auto result_future = action_client_->async_get_result(goal_handle_future.get());
      if (rclcpp::spin_until_future_complete(node_, result_future) != rclcpp::FutureReturnCode::SUCCESS)
      {
        RCLCPP_ERROR(node_->get_logger(), "get result call failed");
        return;
      }

      auto wrapped_result = result_future.get();
      if (wrapped_result.code == rclcpp_action::ResultCode::SUCCEEDED)
      {
        RCLCPP_INFO(node_->get_logger(), "Waypoint reached");
        result->BoolAttribute("result", true);
      }
      else
      {
        RCLCPP_INFO(node_->get_logger(), "Waypoint not reached");
      }
    };
  }

protected:
  // not implemented
  virtual nav2_msgs::action::FollowWaypoints::Goal
  generate_goal(std::shared_ptr<tinyxml2::XMLElement> parameters) override
  {
    return nav2_msgs::action::FollowWaypoints::Goal();
  }

  virtual std::shared_ptr<tinyxml2::XMLElement>
  generate_result(const nav2_msgs::action::FollowWaypoints::Result::SharedPtr& result) override
  {
    return nullptr;
  }

protected:
  std::string global_frame_;     /**The global frame of the robot*/
  std::string robot_base_frame_; /**The frame of the robot base*/

  double x, y; /**Coordinate frame parameters*/
};

}  // namespace capabilities2_runner
