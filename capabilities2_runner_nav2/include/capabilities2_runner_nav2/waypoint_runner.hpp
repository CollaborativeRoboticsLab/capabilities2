#pragma once

#include <thread>
#include <chrono>
#include <tinyxml2.h>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>

#include <geometry_msgs/msg/pose_stamped.hpp>
#include <nav2_msgs/action/navigate_to_pose.hpp>

#include <capabilities2_runner/action_runner.hpp>

namespace capabilities2_runner
{

/**
 * @brief waypoint runner class
 *
 * Class to run waypointfollower action based capability
 *
 */
class WayPointRunner : public ActionRunner<nav2_msgs::action::NavigateToPose>
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
   */
  virtual void start(rclcpp::Node::SharedPtr node, const runner_opts& run_config) override
  {
    init_action(node, run_config, "navigate_to_pose");
  }

protected:
  /**
   * @brief This generate goal function overrides the generate_goal() function from ActionRunner()
   * @param parameters XMLElement that contains parameters in the format
   '<Event name=follow_waypoints provider=WaypointRunner x='$value' y='$value' />'
   * @return ActionT::Goal the generated goal
   */
  virtual nav2_msgs::action::NavigateToPose::Goal generate_goal(tinyxml2::XMLElement* parameters) override
  {
    parameters_ = parameters;

    parameters_->QueryDoubleAttribute("x", &x);
    parameters_->QueryDoubleAttribute("y", &y);

    RCLCPP_INFO(node_->get_logger(), "%s is triggered with x: %f and y: %f", run_config_.interface.c_str(), x, y);

    nav2_msgs::action::NavigateToPose::Goal goal_msg;
    geometry_msgs::msg::PoseStamped pose_msg;

    pose_msg.header.frame_id = "map";
    pose_msg.pose.position.x = x;
    pose_msg.pose.position.y = y;
    pose_msg.pose.position.z = 0.0;
    pose_msg.pose.orientation.w = 1.0;  // Set default orientation (facing forward)

    goal_msg.pose = pose_msg;

    return goal_msg;
  }

  /**
   * @brief This generate result function overrides the generate_result() function from ActionRunner(). Since
   * this is not used in this context, this returns nullptr
   * @param result message from FollowWaypoints action
   * @return nullptr
   */
  virtual tinyxml2::XMLElement*
  generate_result(const nav2_msgs::action::NavigateToPose::Result::SharedPtr& result) override
  {
    return nullptr;
  }

protected:
  std::string global_frame_;     /**The global frame of the robot*/
  std::string robot_base_frame_; /**The frame of the robot base*/

  double x, y; /**Coordinate frame parameters*/
};

}  // namespace capabilities2_runner
