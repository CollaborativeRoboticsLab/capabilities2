#include <pluginlib/class_list_macros.hpp>
#include <capabilities2_runner/runner_base.hpp>
#include <capabilities2_runner_nav2/waypoint_runner.hpp>
#include <capabilities2_runner_nav2/occupancygrid_runner.hpp>
#include <capabilities2_runner_nav2/robotpose_runner.hpp>

namespace capabilities2_runner
{

}

// register runner plugins
PLUGINLIB_EXPORT_CLASS(capabilities2_runner::WayPointRunner, capabilities2_runner::RunnerBase)
PLUGINLIB_EXPORT_CLASS(capabilities2_runner::RobotPoseRunner, capabilities2_runner::RunnerBase)
PLUGINLIB_EXPORT_CLASS(capabilities2_runner::OccupancyGridRunner, capabilities2_runner::RunnerBase)
