#include <pluginlib/class_list_macros.hpp>
#include <capabilities2_runners_plugins/runner_base.hpp>
#include <capabilities2_runners_plugins/launch_runner.hpp>

namespace capabilities2_runner
{

}

// register runner plugins
PLUGINLIB_EXPORT_CLASS(capabilities2_runner::LaunchRunner, capabilities2_runner::RunnerBase)