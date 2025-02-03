#pragma once

#include <thread>
#include <chrono>
#include <string>
#include <tinyxml2.h>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <lifecycle_msgs/srv/get_state.hpp>

#include <geometry_msgs/msg/pose_stamped.hpp>
#include <nav2_msgs/action/navigate_to_pose.hpp>

#include <capabilities2_runner/action_runner.hpp>

using namespace std::chrono_literals;

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
  virtual void start(rclcpp::Node::SharedPtr node, const runner_opts& run_config,
                     std::function<void(Event&)> runner_publish_func) override
  {
    init_action(node, run_config, "navigate_to_pose", runner_publish_func);
  }

protected:
  /**
   * @brief This generate goal function overrides the generate_goal() function from ActionRunner()
   * @param parameters XMLElement that contains parameters in the format
   '<Event name=follow_waypoints provider=WaypointRunner x='$value' y='$value' />'
   * @return ActionT::Goal the generated goal
   */
  virtual nav2_msgs::action::NavigateToPose::Goal generate_goal(tinyxml2::XMLElement* parameters, int id) override
  {
    parameters->QueryDoubleAttribute("x", &x);
    parameters->QueryDoubleAttribute("y", &y);

    info_("goal consist of x: " + std::to_string(x) + " and y: " + std::to_string(y), id);

    nav2_msgs::action::NavigateToPose::Goal goal_msg;
    geometry_msgs::msg::PoseStamped pose_msg;

    pose_msg.header.frame_id = "map";
    pose_msg.header.stamp.sec = 0;
    pose_msg.header.stamp.nanosec = 0;

    pose_msg.pose.position.x = x;
    pose_msg.pose.position.y = y;
    pose_msg.pose.position.z = 0.0;
    pose_msg.pose.orientation.w = 1.0;  // Set default orientation (facing forward)

    goal_msg.pose = pose_msg;

    return goal_msg;
  }

  /**
   * @brief This generate feedback function overrides the generate_feedback() function from ActionRunner()
   *
   * @param msg feedback message from the action server
   * @return std::string of feedback information
   */
  virtual std::string
  generate_feedback(const typename nav2_msgs::action::NavigateToPose::Feedback::ConstSharedPtr msg) override
  {
    // std::string feedback = "x: " + std::to_string(msg->current_pose.pose.position.x) +
    //                        " y: " + std::to_string(msg->current_pose.pose.position.y);
    // return feedback;
    return "";
  }

protected:
  std::string global_frame_;     /**The global frame of the robot*/
  std::string robot_base_frame_; /**The frame of the robot base*/

  double x, y; /**Coordinate frame parameters*/

  rclcpp::Client<lifecycle_msgs::srv::GetState>::SharedPtr get_state_client_;
};

}  // namespace capabilities2_runner
