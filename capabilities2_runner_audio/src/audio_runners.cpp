#include <pluginlib/class_list_macros.hpp>
#include <capabilities2_runner/runner_base.hpp>
#include <capabilities2_runner_audio/listen_runner.hpp>
#include <capabilities2_runner_audio/speak_runner.hpp>

// register runner plugins
PLUGINLIB_EXPORT_CLASS(capabilities2_runner::ListenerRunner, capabilities2_runner::RunnerBase)
PLUGINLIB_EXPORT_CLASS(capabilities2_runner::SpeakerRunner, capabilities2_runner::RunnerBase)
