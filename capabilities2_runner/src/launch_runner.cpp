#include <pluginlib/class_list_macros.hpp>
#include <capabilities2_runner/runner_base.hpp>
#include <capabilities2_runner_capabilities/launch_runner.hpp>

// register runner plugins
PLUGINLIB_EXPORT_CLASS(capabilities2_runner_capabilities::LaunchRunner, capabilities2_runner::RunnerBase)
