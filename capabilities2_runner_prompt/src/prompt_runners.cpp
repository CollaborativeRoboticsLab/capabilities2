#include <pluginlib/class_list_macros.hpp>
#include <capabilities2_runner/runner_base.hpp>
#include <capabilities2_runner_prompt/prompt_runner.hpp>

namespace capabilities2_runner
{

}

// register runner plugins
PLUGINLIB_EXPORT_CLASS(capabilities2_runner::PromptRunner, capabilities2_runner::RunnerBase)