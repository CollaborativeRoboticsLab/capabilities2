#include <memory>
#include <string>
#include <vector>

#include <gtest/gtest.h>

#include <rclcpp/rclcpp.hpp>

#include <capabilities2_events/event_base.hpp>
#include <capabilities2_runner/launch_runner.hpp>

namespace
{

struct ObservedEvent
{
  std::string ownership_id;
  uint8_t event_code;
  capabilities2_msgs::msg::Capability source;
  std::string instance_id;
};

class RecordingEventEmitter : public capabilities2_events::EventBase
{
public:
  void publish_observed_event(const std::string& ownership_id, const uint8_t& event_code,
                              const capabilities2_msgs::msg::Capability& source,
                              const std::string& instance_id) override
  {
    observed_events.push_back(ObservedEvent{ ownership_id, event_code, source, instance_id });
  }

  void on_server_ready(const std::string& msg) override
  {
    (void)msg;
  }

  void on_process_launched(const std::string& pid) override
  {
    (void)pid;
  }

  void on_process_terminated(const std::string& pid) override
  {
    (void)pid;
  }

  void on_triggered(const std::string& trigger_id) override
  {
    (void)trigger_id;
  }

  void on_connected(const std::string& source, const std::string& target) override
  {
    (void)source;
    (void)target;
  }

  void on_disconnected(const std::string& source, const std::string& target) override
  {
    (void)source;
    (void)target;
  }

  std::vector<ObservedEvent> observed_events;
};

class TestableLaunchRunner : public capabilities2_runner::LaunchRunner
{
public:
  void set_launch_reference(const std::string& launch_reference)
  {
    launch_reference_ = launch_reference;
  }

  void deliver_feedback(const capabilities2_msgs::action::Launch::Feedback& feedback, const std::string& bond_id)
  {
    handle_feedback(std::make_shared<const capabilities2_msgs::action::Launch::Feedback>(feedback), bond_id);
  }
};

class LaunchRunnerTest : public ::testing::Test
{
protected:
  static void SetUpTestSuite()
  {
    if (!rclcpp::ok())
    {
      rclcpp::init(0, nullptr);
    }
  }

  static void TearDownTestSuite()
  {
    if (rclcpp::ok())
    {
      rclcpp::shutdown();
    }
  }
};

TEST_F(LaunchRunnerTest, EmitsObservedLaunchedEventOnlyOnceWhenFeedbackReportsPid)
{
  auto node = std::make_shared<rclcpp::Node>("launch_runner_test_node");
  auto events = std::make_shared<RecordingEventEmitter>();

  capabilities2_runner::runner_opts run_config;
  run_config.interface = "demo/LaunchCapability";
  run_config.provider = "demo_launch_provider";
  run_config.runner = "capabilities2_launch_proxy:demo.launch.py";

  TestableLaunchRunner runner;
  runner.init_base(node, run_config);
  runner.enable_events(events);
  runner.set_launch_reference(run_config.runner);

  capabilities2_msgs::action::Launch::Feedback feedback_without_pid;
  feedback_without_pid.launch_id = "launch-id";
  feedback_without_pid.resolved_launch_file_path = "/tmp/demo.launch.py";

  capabilities2_msgs::action::Launch::Feedback feedback_with_pid = feedback_without_pid;
  feedback_with_pid.resource.pid = 4242;
  feedback_with_pid.main_pids.push_back(4242);

  runner.deliver_feedback(feedback_without_pid, "bond-1");
  runner.deliver_feedback(feedback_with_pid, "bond-1");
  runner.deliver_feedback(feedback_with_pid, "bond-1");

  ASSERT_EQ(events->observed_events.size(), 1U);
  EXPECT_EQ(events->observed_events.front().event_code, capabilities2_msgs::msg::CapabilityEventCode::LAUNCHED);
  EXPECT_EQ(events->observed_events.front().ownership_id, "");
  EXPECT_EQ(events->observed_events.front().instance_id, "");
  EXPECT_EQ(events->observed_events.front().source.capability, run_config.interface);
  EXPECT_EQ(events->observed_events.front().source.provider, run_config.provider);
}

}  // namespace