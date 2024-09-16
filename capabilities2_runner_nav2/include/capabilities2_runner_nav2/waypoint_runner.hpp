#pragma once

#include <tinyxml2.h>
#include <capabilities2_runner/action_runner.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <nav2_msgs/action/follow_waypoints.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <thread>

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
    virtual void trigger(std::shared_ptr<tinyxml2::XMLElement> parameters = nullptr)
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

        // launch runner using action client
        action_client_->async_send_goal(goal_msg, send_goal_options_);
    }

    protected:
    std::string global_frame_;     /**The global frame of the robot*/
    std::string robot_base_frame_; /**The frame of the robot base*/

    double x, y; /**Coordinate frame parameters*/
};

}  // namespace capabilities2_runner
