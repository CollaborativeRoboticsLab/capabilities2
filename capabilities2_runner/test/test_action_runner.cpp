#include <atomic>
#include <chrono>
#include <future>
#include <memory>
#include <mutex>
#include <thread>

#include <gtest/gtest.h>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <capabilities2_msgs/action/launch.hpp>
#include <capabilities2_runner/action_runner.hpp>

namespace
{
using Action = capabilities2_msgs::action::Launch;
using ServerGoal = rclcpp_action::ServerGoalHandle<Action>;
using namespace std::chrono_literals;

class TestActionRunner : public capabilities2_runner::ActionRunner<Action>
{
public:
  void start(rclcpp::Node::SharedPtr node, const capabilities2_runner::runner_opts& config,
             const std::string&) override
  {
    init_action(node, config, "/test_action_runner", "test_action");
  }

  bool has_goal()
  {
    std::lock_guard<std::mutex> lock(goal_mutex_);
    return static_cast<bool>(goal_handle_);
  }

  std::future<void> result_received()
  {
    return result_promise_.get_future();
  }

  std::future<void> failure_received()
  {
    return failure_promise_.get_future();
  }

  void set_result_gate(std::shared_future<void> gate)
  {
    result_gate_ = gate;
  }

protected:
  Action::Goal generate_goal(capabilities2_events::EventParameters&) override
  {
    return Action::Goal();
  }

  std::string generate_feedback(const Action::Feedback::ConstSharedPtr) override
  {
    return "";
  }

  void process_result(Action::Result::SharedPtr) override
  {
    result_promise_.set_value();
    if (result_gate_.valid())
      result_gate_.wait();
  }

  capabilities2_events::EventParameters param_on_failure() override
  {
    failure_promise_.set_value();
    return {};
  }

private:
  std::promise<void> result_promise_;
  std::promise<void> failure_promise_;
  std::shared_future<void> result_gate_;
};

class ActionRunnerTest : public ::testing::Test
{
protected:
  static void SetUpTestSuite()
  {
    if (!rclcpp::ok())
      rclcpp::init(0, nullptr);
  }

  static void TearDownTestSuite()
  {
    if (rclcpp::ok())
      rclcpp::shutdown();
  }
};

class ActionEnvironment
{
public:
  explicit ActionEnvironment(bool complete_goal, bool accept_goal = true)
    : node(std::make_shared<rclcpp::Node>("action_runner_test"))
  {
    server = rclcpp_action::create_server<Action>(
        node, "/test_action_runner",
        [accept_goal](const rclcpp_action::GoalUUID&, std::shared_ptr<const Action::Goal>) {
          return accept_goal ? rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE : rclcpp_action::GoalResponse::REJECT;
        },
        [this](const std::shared_ptr<ServerGoal>) {
          ++cancel_count;
          return rclcpp_action::CancelResponse::ACCEPT;
        },
        [this, complete_goal](const std::shared_ptr<ServerGoal> goal) {
          std::lock_guard<std::mutex> lock(mutex);
          server_goal = goal;
          if (complete_goal)
            goal->succeed(std::make_shared<Action::Result>());
        });
    timer = node->create_wall_timer(10ms, [this]() {
      std::lock_guard<std::mutex> lock(mutex);
      if (server_goal && server_goal->is_canceling())
        server_goal->canceled(std::make_shared<Action::Result>());
    });
    executor.add_node(node);
    thread = std::thread([this]() { executor.spin(); });
  }

  ~ActionEnvironment()
  {
    executor.cancel();
    thread.join();
  }

  rclcpp::Node::SharedPtr node;
  rclcpp_action::Server<Action>::SharedPtr server;
  rclcpp::TimerBase::SharedPtr timer;
  rclcpp::executors::MultiThreadedExecutor executor;
  std::thread thread;
  std::atomic<int> cancel_count{ 0 };
  std::mutex mutex;
  std::shared_ptr<ServerGoal> server_goal;
};

TEST_F(ActionRunnerTest, DoesNotCancelCompletedGoal)
{
  ActionEnvironment environment(true);
  TestActionRunner runner;
  capabilities2_runner::runner_opts config{};
  runner.start(environment.node, config, "bond");
  auto result = runner.result_received();
  capabilities2_events::EventParameters parameters;
  runner.trigger(parameters, "bond", "completed");
  ASSERT_EQ(result.wait_for(5s), std::future_status::ready);
  EXPECT_FALSE(runner.has_goal());
  EXPECT_NO_THROW(runner.stop("bond"));
  EXPECT_EQ(environment.cancel_count.load(), 0);
}

TEST_F(ActionRunnerTest, CancelsActiveGoal)
{
  ActionEnvironment environment(false);
  TestActionRunner runner;
  capabilities2_runner::runner_opts config{};
  runner.start(environment.node, config, "bond");
  auto result = runner.result_received();
  capabilities2_events::EventParameters parameters;
  runner.trigger(parameters, "bond", "active");
  const auto deadline = std::chrono::steady_clock::now() + 5s;
  while (!runner.has_goal() && std::chrono::steady_clock::now() < deadline)
    std::this_thread::sleep_for(10ms);
  ASSERT_TRUE(runner.has_goal());
  EXPECT_NO_THROW(runner.stop("bond"));
  EXPECT_EQ(result.wait_for(5s), std::future_status::ready);
  EXPECT_EQ(environment.cancel_count.load(), 1);
}

TEST_F(ActionRunnerTest, RejectedGoalCompletesWithoutCancellation)
{
  ActionEnvironment environment(false, false);
  TestActionRunner runner;
  capabilities2_runner::runner_opts config{};
  runner.start(environment.node, config, "bond");
  auto failure = runner.failure_received();
  capabilities2_events::EventParameters parameters;
  runner.trigger(parameters, "bond", "rejected");
  ASSERT_EQ(failure.wait_for(5s), std::future_status::ready);
  EXPECT_NO_THROW(runner.stop("bond"));
  EXPECT_EQ(environment.cancel_count.load(), 0);
}

TEST_F(ActionRunnerTest, StopWaitsForResultProcessing)
{
  ActionEnvironment environment(true);
  TestActionRunner runner;
  capabilities2_runner::runner_opts config{};
  runner.start(environment.node, config, "bond");
  std::promise<void> release_result;
  runner.set_result_gate(release_result.get_future().share());
  auto result = runner.result_received();
  capabilities2_events::EventParameters parameters;
  runner.trigger(parameters, "bond", "blocked");
  const auto received = result.wait_for(5s);
  auto stopped = std::async(std::launch::async, [&runner]() {runner.stop("bond");});
  EXPECT_EQ(received, std::future_status::ready);
  EXPECT_EQ(stopped.wait_for(100ms), std::future_status::timeout);
  release_result.set_value();
  EXPECT_EQ(stopped.wait_for(5s), std::future_status::ready);
  EXPECT_NO_THROW(stopped.get());
}
}  // namespace