#pragma once

#include <capabilities2_runner/action_runner.hpp>

namespace capabilities2_runner_audio
{
class ListenRunner : public capabilities2_runner::ActionRunner<audio_msgs::action::STT>
{
public:
  ListenRunner(/* args */);
  ~ListenRunner();

private:
};

ListenRunner::ListenRunner(/* args */)
{
}

ListenRunner::~ListenRunner()
{
}

}  // namespace capabilities2_runner_audio
