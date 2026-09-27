#include <gtest/gtest.h>

#include <capabilities2_events/published_event.hpp>

namespace
{

capabilities2_msgs::msg::Capability makeCapability(const std::string& capability,
                                                   const std::string& provider,
                                                   const std::string& instance_id = "")
{
  capabilities2_msgs::msg::Capability value;
  value.capability = capability;
  value.provider = provider;
  value.instance_id = instance_id;
  return value;
}

}  // namespace

TEST(PublishedEventTest, ReturnsOnlyNewEventsDuringNormalPolling)
{
  capabilities2_events::PublishedEvent publisher({}, 5);
  const auto source = makeCapability("nav/MoveBase", "move_base_provider", "instance-1");

  publisher.publish_observed_event("plan-1", capabilities2_msgs::msg::CapabilityEventCode::TRIGGERED, source, "instance-1");
  publisher.publish_observed_event("plan-1", capabilities2_msgs::msg::CapabilityEventCode::SUCCEEDED, source, "instance-1");

  const auto initial_snapshot = publisher.get_event_snapshot(0, 10);
  ASSERT_EQ(initial_snapshot.events.size(), 2U);
  EXPECT_EQ(initial_snapshot.oldest_sequence, 1U);
  EXPECT_EQ(initial_snapshot.latest_sequence, 2U);
  EXPECT_FALSE(initial_snapshot.truncated);
  EXPECT_EQ(initial_snapshot.events.front().event.code.code, capabilities2_msgs::msg::CapabilityEventCode::TRIGGERED);
  EXPECT_EQ(initial_snapshot.events.back().event.code.code, capabilities2_msgs::msg::CapabilityEventCode::SUCCEEDED);

  const auto delta_snapshot = publisher.get_event_snapshot(1, 10);
  ASSERT_EQ(delta_snapshot.events.size(), 1U);
  EXPECT_EQ(delta_snapshot.oldest_sequence, 1U);
  EXPECT_EQ(delta_snapshot.latest_sequence, 2U);
  EXPECT_FALSE(delta_snapshot.truncated);
  EXPECT_EQ(delta_snapshot.events.front().event.code.code, capabilities2_msgs::msg::CapabilityEventCode::SUCCEEDED);
}

TEST(PublishedEventTest, ReportsTruncationWhenCallerFallsBehindRingBuffer)
{
  capabilities2_events::PublishedEvent publisher({}, 3);
  const auto source = makeCapability("nav/MoveBase", "move_base_provider", "instance-1");

  publisher.publish_observed_event("plan-1", capabilities2_msgs::msg::CapabilityEventCode::TRIGGERED, source, "instance-1");
  publisher.publish_observed_event("plan-1", capabilities2_msgs::msg::CapabilityEventCode::STARTED, source, "instance-1");
  publisher.publish_observed_event("plan-1", capabilities2_msgs::msg::CapabilityEventCode::CONNECTED, source, "instance-1");
  publisher.publish_observed_event("plan-1", capabilities2_msgs::msg::CapabilityEventCode::SUCCEEDED, source, "instance-1");
  publisher.publish_observed_event("plan-1", capabilities2_msgs::msg::CapabilityEventCode::STOPPED, source, "instance-1");

  const auto snapshot = publisher.get_event_snapshot(1, 10);
  ASSERT_EQ(snapshot.events.size(), 3U);
  EXPECT_EQ(snapshot.oldest_sequence, 3U);
  EXPECT_EQ(snapshot.latest_sequence, 5U);
  EXPECT_TRUE(snapshot.truncated);
  EXPECT_EQ(snapshot.events.front().event.code.code, capabilities2_msgs::msg::CapabilityEventCode::CONNECTED);
  EXPECT_EQ(snapshot.events.back().event.code.code, capabilities2_msgs::msg::CapabilityEventCode::STOPPED);
}