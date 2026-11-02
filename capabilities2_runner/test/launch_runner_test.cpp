#include <atomic>
#include <future>
#include <memory>
#include <string>
#include <vector>

#include <gtest/gtest.h>

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>

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

  capabilities2_msgs::action::Launch::Goal build_goal_message() const
  {
    return make_goal_message();
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

class FakeLaunchEnvironment
{
public:
  using LaunchAction = capabilities2_msgs::action::Launch;
  using ServerGoalHandle = rclcpp_action::ServerGoalHandle<LaunchAction>;

  FakeLaunchEnvironment() : node(std::make_shared<rclcpp::Node>("fake_launch_readiness"))
  {
    action_server = rclcpp_action::create_server<LaunchAction>(
        node, "/test_launch_readiness",
        [](const rclcpp_action::GoalUUID&, std::shared_ptr<const LaunchAction::Goal>) {
          return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
        },
        [this](const std::shared_ptr<ServerGoalHandle>) {
          ++cancel_count;
          return reject_cancel.load() ? rclcpp_action::CancelResponse::REJECT : rclcpp_action::CancelResponse::ACCEPT;
        },
        [this](const std::shared_ptr<ServerGoalHandle> goal) {
          std::lock_guard<std::mutex> lock(goal_mutex);
          goal_handle = goal;
        });
    nav_service = make_service("/readiness_test/nav/get_state", nav_state);
    planner_service = make_service("/readiness_test/planner/get_state", planner_state);
    executor.add_node(node);
    thread = std::thread([this]() { executor.spin(); });
  }

  ~FakeLaunchEnvironment()
  {
    executor.cancel();
    thread.join();
  }

  rclcpp::Service<lifecycle_msgs::srv::GetState>::SharedPtr make_service(const std::string& name,
                                                                         std::atomic<uint8_t>& state)
  {
    return node->create_service<lifecycle_msgs::srv::GetState>(
        name, [this, &state](const std::shared_ptr<lifecycle_msgs::srv::GetState::Request>,
                             std::shared_ptr<lifecycle_msgs::srv::GetState::Response> response) {
          ++request_count;
          std::this_thread::sleep_for(std::chrono::milliseconds(service_delay_ms.load()));
          response->current_state.id = state.load();
        });
  }

  void abort_goal()
  {
    std::lock_guard<std::mutex> lock(goal_mutex);
    if (goal_handle)
    {
      auto result = std::make_shared<LaunchAction::Result>();
      result->final_state = "failed";
      result->message = "test launch failure";
      goal_handle->abort(result);
    }
  }

  rclcpp::Node::SharedPtr node;
  rclcpp_action::Server<LaunchAction>::SharedPtr action_server;
  rclcpp::Service<lifecycle_msgs::srv::GetState>::SharedPtr nav_service;
  rclcpp::Service<lifecycle_msgs::srv::GetState>::SharedPtr planner_service;
  rclcpp::executors::SingleThreadedExecutor executor;
  std::thread thread;
  std::atomic<uint8_t> nav_state{ lifecycle_msgs::msg::State::PRIMARY_STATE_INACTIVE };
  std::atomic<uint8_t> planner_state{ lifecycle_msgs::msg::State::PRIMARY_STATE_INACTIVE };
  std::atomic<int> request_count{ 0 };
  std::atomic<int> service_delay_ms{ 0 };
  std::atomic<int> cancel_count{ 0 };
  std::atomic<bool> reject_cancel{ false };
  std::mutex goal_mutex;
  std::shared_ptr<ServerGoalHandle> goal_handle;
};

capabilities2_runner::runner_opts readiness_config()
{
  capabilities2_runner::runner_opts config;
  config.interface = "demo/LaunchCapability";
  config.provider = "demo_launch_provider";
  config.runner = "demo.launch.py";
  config.launch_parameters.push_back({ "namespace", "readiness_test" });
  config.launch_readiness.lifecycle_nodes = { "nav", "/readiness_test/planner" };
  config.launch_readiness.lifecycle_timeout_sec = 3.0;
  return config;
}

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

TEST_F(LaunchRunnerTest, IncludesConfiguredLaunchParametersInGoal)
{
  auto node = std::make_shared<rclcpp::Node>("launch_runner_goal_test_node");

  capabilities2_runner::runner_opts run_config;
  run_config.interface = "demo/LaunchCapability";
  run_config.provider = "demo_launch_provider";
  run_config.runner = "capabilities2_launch_proxy:demo.launch.py";
  run_config.launch_parameters.push_back({ "use_sim_time", "true" });
  run_config.launch_parameters.push_back({ "robot_name", "tb4" });

  TestableLaunchRunner runner;
  runner.init_base(node, run_config);
  runner.set_launch_reference(run_config.runner);

  const auto goal = runner.build_goal_message();

  EXPECT_EQ(goal.reference_name, run_config.provider);
  EXPECT_EQ(goal.launch_file_path, run_config.runner);
  ASSERT_EQ(goal.launch_parameters.size(), 2U);
  EXPECT_EQ(goal.launch_parameters[0].key, "use_sim_time");
  EXPECT_EQ(goal.launch_parameters[0].value, "true");
  EXPECT_EQ(goal.launch_parameters[1].key, "robot_name");
  EXPECT_EQ(goal.launch_parameters[1].value, "tb4");
}

TEST_F(LaunchRunnerTest, WaitsForEveryConfiguredLifecycleNodeBeforeStarting)
{
  FakeLaunchEnvironment environment;
  auto node = std::make_shared<rclcpp::Node>("readiness_runner_test");
  node->declare_parameter("launch_action_name", "/test_launch_readiness");
  auto events = std::make_shared<RecordingEventEmitter>();
  TestableLaunchRunner runner;
  runner.enable_events(events);
  const auto config = readiness_config();

  auto started = std::async(std::launch::async, [&]() { runner.start(node, config, "bond"); });
  EXPECT_EQ(started.wait_for(std::chrono::milliseconds(400)), std::future_status::timeout);
  environment.nav_state = lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE;
  EXPECT_EQ(started.wait_for(std::chrono::milliseconds(400)), std::future_status::timeout);
  environment.planner_state = lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE;
  EXPECT_NO_THROW(started.get());

  ASSERT_EQ(events->observed_events.size(), 1U);
  EXPECT_EQ(events->observed_events.front().event_code, capabilities2_msgs::msg::CapabilityEventCode::STARTED);
  runner.stop("bond");
  EXPECT_EQ(environment.cancel_count.load(), 1);
}

TEST_F(LaunchRunnerTest, TimesOutAndCancelsLaunchWhenNodeRemainsInactive)
{
  FakeLaunchEnvironment environment;
  auto node = std::make_shared<rclcpp::Node>("readiness_timeout_test");
  node->declare_parameter("launch_action_name", "/test_launch_readiness");
  auto events = std::make_shared<RecordingEventEmitter>();
  TestableLaunchRunner runner;
  runner.enable_events(events);
  auto config = readiness_config();
  config.launch_readiness.lifecycle_timeout_sec = 0.5;

  EXPECT_THROW(runner.start(node, config, "bond"), capabilities2_runner::runner_exception);
  EXPECT_GT(environment.request_count.load(), 0);
  EXPECT_EQ(environment.cancel_count.load(), 1);
  ASSERT_EQ(events->observed_events.size(), 1U);
  EXPECT_EQ(events->observed_events.front().event_code, capabilities2_msgs::msg::CapabilityEventCode::FAILED);
}

TEST_F(LaunchRunnerTest, StartsWithoutLifecycleReadinessConfiguration)
{
  FakeLaunchEnvironment environment;
  auto node = std::make_shared<rclcpp::Node>("readiness_opt_out_test");
  node->declare_parameter("launch_action_name", "/test_launch_readiness");
  TestableLaunchRunner runner;
  auto config = readiness_config();
  config.launch_readiness.lifecycle_nodes.clear();

  EXPECT_NO_THROW(runner.start(node, config, "bond"));
  EXPECT_EQ(environment.request_count.load(), 0);
  runner.stop("bond");
}

TEST_F(LaunchRunnerTest, TimesOutWhenLifecycleServiceIsMissing)
{
  FakeLaunchEnvironment environment;
  auto node = std::make_shared<rclcpp::Node>("readiness_missing_service_test");
  node->declare_parameter("launch_action_name", "/test_launch_readiness");
  TestableLaunchRunner runner;
  auto config = readiness_config();
  config.launch_readiness.lifecycle_nodes = { "missing" };
  config.launch_readiness.lifecycle_timeout_sec = 0.4;

  EXPECT_THROW(runner.start(node, config, "bond"), capabilities2_runner::runner_exception);
  EXPECT_EQ(environment.cancel_count.load(), 1);
}

TEST_F(LaunchRunnerTest, AcceptsLifecycleResponsesSlowerThanPollingInterval)
{
  FakeLaunchEnvironment environment;
  environment.nav_state = lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE;
  environment.service_delay_ms = 250;
  auto node = std::make_shared<rclcpp::Node>("readiness_slow_response_test");
  node->declare_parameter("launch_action_name", "/test_launch_readiness");
  TestableLaunchRunner runner;
  auto config = readiness_config();
  config.launch_readiness.lifecycle_nodes = { "nav" };
  config.launch_readiness.lifecycle_timeout_sec = 2.0;

  EXPECT_NO_THROW(runner.start(node, config, "bond"));
  EXPECT_EQ(environment.request_count.load(), 1);
  runner.stop("bond");
}

TEST_F(LaunchRunnerTest, CleansUpExecutorWhenCancellationIsRejected)
{
  FakeLaunchEnvironment environment;
  environment.reject_cancel = true;
  auto node = std::make_shared<rclcpp::Node>("readiness_cancel_rejected_test");
  node->declare_parameter("launch_action_name", "/test_launch_readiness");
  TestableLaunchRunner runner;
  auto config = readiness_config();
  config.launch_readiness.lifecycle_nodes.clear();

  runner.start(node, config, "bond");
  EXPECT_THROW(runner.stop("bond"), capabilities2_runner::runner_exception);
  EXPECT_EQ(environment.cancel_count.load(), 1);
}

TEST_F(LaunchRunnerTest, FailsEarlyWhenLaunchAbortsDuringReadiness)
{
  FakeLaunchEnvironment environment;
  auto node = std::make_shared<rclcpp::Node>("readiness_abort_test");
  node->declare_parameter("launch_action_name", "/test_launch_readiness");
  auto events = std::make_shared<RecordingEventEmitter>();
  TestableLaunchRunner runner;
  runner.enable_events(events);
  const auto config = readiness_config();

  auto started = std::async(std::launch::async, [&]() { runner.start(node, config, "bond"); });
  for (int attempt = 0; attempt < 30 && environment.request_count.load() == 0; ++attempt)
  {
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
  }
  EXPECT_GT(environment.request_count.load(), 0);
  environment.abort_goal();
  EXPECT_EQ(started.wait_for(std::chrono::seconds(2)), std::future_status::ready);
  EXPECT_THROW(started.get(), capabilities2_runner::runner_exception);
  ASSERT_EQ(events->observed_events.size(), 1U);
  EXPECT_EQ(events->observed_events.front().event_code, capabilities2_msgs::msg::CapabilityEventCode::FAILED);
}

TEST_F(LaunchRunnerTest, WaitsForActionServiceAndFirstTopicMessage)
{
  FakeLaunchEnvironment environment;
  auto publisher = environment.node->create_publisher<std_msgs::msg::String>(
      "/readiness_test/status", rclcpp::SensorDataQoS());
  auto node = std::make_shared<rclcpp::Node>("endpoint_readiness_test");
  node->declare_parameter("launch_action_name", "/test_launch_readiness");
  TestableLaunchRunner runner;
  auto config = readiness_config();
  config.launch_readiness.lifecycle_nodes.clear();
  config.launch_readiness.action = { "/test_launch_readiness", "capabilities2_msgs/action/Launch", 2.0 };
  config.launch_readiness.service = { "nav/get_state", "lifecycle_msgs/srv/GetState", 2.0 };
  config.launch_readiness.topic = { "status", "std_msgs/msg/String", 2.0 };

  auto started = std::async(std::launch::async, [&]() { runner.start(node, config, "bond"); });
  EXPECT_EQ(started.wait_for(std::chrono::milliseconds(400)), std::future_status::timeout);
  std_msgs::msg::String message;
  message.data = "ready";
  publisher->publish(message);
  EXPECT_NO_THROW(started.get());
  runner.stop("bond");
}

TEST_F(LaunchRunnerTest, ReceivesTopicMessageWhileWaitingForLifecycle)
{
  FakeLaunchEnvironment environment;
  auto publisher = environment.node->create_publisher<std_msgs::msg::String>(
      "/readiness_test/status", rclcpp::SensorDataQoS());
  auto node = std::make_shared<rclcpp::Node>("early_topic_readiness_test");
  node->declare_parameter("launch_action_name", "/test_launch_readiness");
  TestableLaunchRunner runner;
  auto config = readiness_config();
  config.launch_readiness.lifecycle_nodes = { "nav" };
  config.launch_readiness.topic = { "status", "std_msgs/msg/String", 1.0 };

  auto started = std::async(std::launch::async, [&]() { runner.start(node, config, "bond"); });
  for (int attempt = 0; attempt < 30 && publisher->get_subscription_count() == 0; ++attempt)
    std::this_thread::sleep_for(std::chrono::milliseconds(50));
  ASSERT_GT(publisher->get_subscription_count(), 0U);
  publisher->publish(std_msgs::msg::String());
  std::this_thread::sleep_for(std::chrono::milliseconds(200));
  EXPECT_EQ(started.wait_for(std::chrono::milliseconds(0)), std::future_status::timeout);
  environment.nav_state = lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE;
  EXPECT_NO_THROW(started.get());
  runner.stop("bond");
}

TEST_F(LaunchRunnerTest, MissingEndpointsTimeoutAndCancelLaunch)
{
  for (const std::string kind : { "action", "service", "topic" })
  {
    FakeLaunchEnvironment environment;
    auto node = std::make_shared<rclcpp::Node>("missing_endpoint_" + kind);
    node->declare_parameter("launch_action_name", "/test_launch_readiness");
    TestableLaunchRunner runner;
    auto config = readiness_config();
    config.launch_readiness.lifecycle_nodes.clear();
    if (kind == "action")
      config.launch_readiness.action = { "missing", "capabilities2_msgs/action/Launch", 0.3 };
    else if (kind == "service")
      config.launch_readiness.service = { "missing", "lifecycle_msgs/srv/GetState", 0.3 };
    else
      config.launch_readiness.topic = { "missing", "std_msgs/msg/String", 0.3 };
    EXPECT_THROW(runner.start(node, config, "bond"), capabilities2_runner::runner_exception) << kind;
    EXPECT_EQ(environment.cancel_count.load(), 1) << kind;
  }
}

TEST_F(LaunchRunnerTest, RejectsPartialEndpointAndInvalidTimeout)
{
  FakeLaunchEnvironment environment;
  auto node = std::make_shared<rclcpp::Node>("invalid_endpoint_readiness_test");
  node->declare_parameter("launch_action_name", "/test_launch_readiness");
  auto config = readiness_config();
  config.launch_readiness.lifecycle_nodes.clear();
  config.launch_readiness.action = { "test", "", 1.0 };
  TestableLaunchRunner partial_runner;
  EXPECT_THROW(partial_runner.start(node, config, "bond"), capabilities2_runner::runner_exception);
  config.launch_readiness.action = { "test", "capabilities2_msgs/action/Launch", 0.0 };
  TestableLaunchRunner timeout_runner;
  EXPECT_THROW(timeout_runner.start(node, config, "bond"), capabilities2_runner::runner_exception);
  EXPECT_EQ(environment.cancel_count.load(), 0);
}

TEST_F(LaunchRunnerTest, RejectsEndpointsWithWrongTypes)
{
  for (const std::string kind : { "action", "service", "topic" })
  {
    FakeLaunchEnvironment environment;
    auto publisher = environment.node->create_publisher<std_msgs::msg::String>(
        "/readiness_test/status", rclcpp::SensorDataQoS());
    auto node = std::make_shared<rclcpp::Node>("wrong_type_" + kind);
    node->declare_parameter("launch_action_name", "/test_launch_readiness");
    TestableLaunchRunner runner;
    auto config = readiness_config();
    config.launch_readiness.lifecycle_nodes.clear();
    if (kind == "action")
      config.launch_readiness.action = { "/test_launch_readiness", "capabilities2_msgs/action/Capability", 0.4 };
    else if (kind == "service")
      config.launch_readiness.service = { "nav/get_state", "std_srvs/srv/Empty", 0.4 };
    else
      config.launch_readiness.topic = { "status", "std_msgs/msg/Bool", 0.4 };
    EXPECT_THROW(runner.start(node, config, "bond"), capabilities2_runner::runner_exception) << kind;
    EXPECT_EQ(environment.cancel_count.load(), 1) << kind;
  }
}

TEST_F(LaunchRunnerTest, RejectsUnavailableMessageType)
{
  FakeLaunchEnvironment environment;
  auto node = std::make_shared<rclcpp::Node>("invalid_message_type_test");
  node->declare_parameter("launch_action_name", "/test_launch_readiness");
  auto config = readiness_config();
  config.launch_readiness.lifecycle_nodes.clear();
  config.launch_readiness.topic = { "status", "nonexistent_interfaces/msg/Missing", 1.0 };
  TestableLaunchRunner runner;
  EXPECT_THROW(runner.start(node, config, "bond"), capabilities2_runner::runner_exception);
  EXPECT_EQ(environment.cancel_count.load(), 0);
}

TEST_F(LaunchRunnerTest, FailsEarlyWhenLaunchAbortsDuringEndpointWait)
{
  FakeLaunchEnvironment environment;
  auto node = std::make_shared<rclcpp::Node>("endpoint_abort_test");
  node->declare_parameter("launch_action_name", "/test_launch_readiness");
  auto config = readiness_config();
  config.launch_readiness.lifecycle_nodes.clear();
  config.launch_readiness.topic = { "missing", "std_msgs/msg/String", 5.0 };
  TestableLaunchRunner runner;
  auto started = std::async(std::launch::async, [&]() { runner.start(node, config, "bond"); });
  for (int attempt = 0; attempt < 30; ++attempt)
  {
    {
      std::lock_guard<std::mutex> lock(environment.goal_mutex);
      if (environment.goal_handle)
        break;
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(50));
  }
  environment.abort_goal();
  EXPECT_EQ(started.wait_for(std::chrono::seconds(2)), std::future_status::ready);
  EXPECT_THROW(started.get(), capabilities2_runner::runner_exception);
}

}  // namespace