import pathlib
import signal
import subprocess
import sys
import time
import uuid

import rclpy
from action_msgs.msg import GoalStatus
from capabilities2_msgs.action import Launch as LaunchAction
from capabilities2_msgs.msg import LaunchParameter, LaunchResourceList
from rclpy.action import ActionClient
from std_msgs.msg import String


PACKAGE_ROOT = pathlib.Path(__file__).resolve().parents[1]


class ServerHarness:
    def __init__(self, ros_args=None):
        self.process = None
        self.ros_args = ros_args or []
        self.output = ''
        self.node_name = f'capabilities2_launch_test_{uuid.uuid4().hex[:8]}'
        self.action_name = f'/{self.node_name}/launch'
        self.resource_topic = f'/{self.node_name}/resources'

    def __enter__(self):
        script_path = PACKAGE_ROOT / 'capabilities2_launch' / 'capabilities2_launch.py'
        command = [
            sys.executable,
            str(script_path),
            '--ros-args',
            '-r', f'__node:={self.node_name}',
        ] + self.ros_args
        self.process = subprocess.Popen(
            command,
            stdout=subprocess.PIPE,
            stderr=subprocess.STDOUT,
            text=True,
        )
        return self

    def __exit__(self, exc_type, exc_val, exc_tb):
        if self.process is None:
            return

        if self.process.poll() is None:
            self.process.send_signal(signal.SIGINT)
            try:
                self.process.wait(timeout=5.0)
            except subprocess.TimeoutExpired:
                self.process.kill()
                self.process.wait(timeout=5.0)

        if self.process.stdout is not None:
            self.output = self.process.stdout.read()
            self.process.stdout.close()


def _spin_until(predicate, node, timeout_sec):
    deadline = time.monotonic() + timeout_sec
    while time.monotonic() < deadline:
        rclpy.spin_once(node, timeout_sec=0.2)
        if predicate():
            return True
    return False


def test_action_resolves_package_fields_and_exposes_node():
    rclpy.init()
    with ServerHarness() as harness:
        client_node = rclpy.create_node('capabilities2_launch_action_client_fields')
        subscriber_node = rclpy.create_node('capabilities2_launch_action_subscriber_fields')
        received_messages = []

        subscriber_node.create_subscription(
            String,
            '/capabilities2_launch/test_chatter',
            lambda msg: received_messages.append(msg.data),
            10,
        )

        try:
            action_client = ActionClient(client_node, LaunchAction, harness.action_name)
            assert action_client.wait_for_server(timeout_sec=5.0)

            goal = LaunchAction.Goal()
            goal.reference_name = 'package-talker'
            goal.package_name = 'capabilities2_launch'
            goal.launch_file_name = 'package_resolved_test.launch.py'

            feedback_messages = []

            send_goal_future = action_client.send_goal_async(
                goal,
                feedback_callback=lambda feedback: feedback_messages.append(feedback.feedback),
            )
            rclpy.spin_until_future_complete(client_node, send_goal_future, timeout_sec=10.0)
            goal_handle = send_goal_future.result()

            assert goal_handle is not None
            assert goal_handle.accepted

            message_received = _spin_until(lambda: bool(received_messages), subscriber_node, timeout_sec=10.0)
            assert message_received
            assert received_messages[0] == 'hello-from-capabilities2-launch'

            feedback_observed = _spin_until(lambda: bool(feedback_messages), client_node, timeout_sec=5.0)
            assert feedback_observed
            assert feedback_messages[-1].resolved_launch_file_path.endswith('package_resolved_test.launch.py')
            assert feedback_messages[-1].state in ('starting', 'running', 'stopping')
            assert feedback_messages[-1].launch_id
            assert feedback_messages[-1].peak_sampled_cpu_usage_percent >= feedback_messages[-1].cpu_usage_percent
            assert feedback_messages[-1].peak_memory_rss_bytes >= feedback_messages[-1].memory_rss_bytes
            assert feedback_messages[-1].peak_memory_vms_bytes >= feedback_messages[-1].memory_vms_bytes
            assert feedback_messages[-1].time_above_80pct_peak_sampled_cpu_sec >= 0.0
            assert feedback_messages[-1].time_above_90pct_peak_rss_sec >= 0.0
            assert feedback_messages[-1].time_above_90pct_peak_vms_sec >= 0.0

            cancel_future = goal_handle.cancel_goal_async()
            rclpy.spin_until_future_complete(client_node, cancel_future, timeout_sec=10.0)
            cancel_response = cancel_future.result()

            assert cancel_response is not None
            assert len(cancel_response.goals_canceling) == 1

            result_future = goal_handle.get_result_async()
            rclpy.spin_until_future_complete(client_node, result_future, timeout_sec=10.0)
            result = result_future.result()

            assert result is not None
            assert result.status == GoalStatus.STATUS_CANCELED
            assert result.result.final_state == 'canceled'
            assert result.result.resolved_launch_file_path.endswith('package_resolved_test.launch.py')
            assert result.result.launch_id
        finally:
            client_node.destroy_node()
            subscriber_node.destroy_node()
            if rclpy.ok():
                rclpy.shutdown()


def test_action_accepts_package_reference_syntax():
    rclpy.init()
    with ServerHarness() as harness:
        client_node = rclpy.create_node('capabilities2_launch_action_client')
        subscriber_node = rclpy.create_node('capabilities2_launch_action_subscriber')
        received_messages = []

        subscriber_node.create_subscription(
            String,
            '/capabilities2_launch/test_chatter',
            lambda msg: received_messages.append(msg.data),
            10,
        )

        try:
            action_client = ActionClient(client_node, LaunchAction, harness.action_name)
            assert action_client.wait_for_server(timeout_sec=5.0)

            goal = LaunchAction.Goal()
            goal.reference_name = 'package-reference-syntax'
            goal.launch_file_path = 'capabilities2_launch:package_resolved_test.launch.py'

            send_goal_future = action_client.send_goal_async(goal)
            rclpy.spin_until_future_complete(client_node, send_goal_future, timeout_sec=10.0)
            goal_handle = send_goal_future.result()

            assert goal_handle is not None
            assert goal_handle.accepted

            message_received = _spin_until(lambda: bool(received_messages), subscriber_node, timeout_sec=10.0)
            assert message_received
            assert received_messages[0] == 'hello-from-capabilities2-launch'

            cancel_future = goal_handle.cancel_goal_async()
            rclpy.spin_until_future_complete(client_node, cancel_future, timeout_sec=10.0)
            cancel_response = cancel_future.result()

            assert cancel_response is not None
            assert len(cancel_response.goals_canceling) == 1

            result_future = goal_handle.get_result_async()
            rclpy.spin_until_future_complete(client_node, result_future, timeout_sec=10.0)
            result = result_future.result()

            assert result is not None
            assert result.status == GoalStatus.STATUS_CANCELED
            assert result.result.final_state == 'canceled'
            assert result.result.resolved_launch_file_path.endswith('package_resolved_test.launch.py')
            assert result.result.launch_id
        finally:
            client_node.destroy_node()
            subscriber_node.destroy_node()
            if rclpy.ok():
                rclpy.shutdown()


def test_action_feedback_returns_error_logs_without_mirroring_child_output_to_server_terminal():
    rclpy.init()
    with ServerHarness() as harness:
        client_node = rclpy.create_node('capabilities2_launch_error_feedback_client')

        try:
            action_client = ActionClient(client_node, LaunchAction, harness.action_name)
            assert action_client.wait_for_server(timeout_sec=5.0)

            goal = LaunchAction.Goal()
            goal.reference_name = 'error-feedback'
            goal.package_name = 'capabilities2_launch'
            goal.launch_file_name = 'package_resolved_error_test.launch.py'

            feedback_messages = []

            send_goal_future = action_client.send_goal_async(
                goal,
                feedback_callback=lambda feedback: feedback_messages.append(feedback.feedback),
            )
            rclpy.spin_until_future_complete(client_node, send_goal_future, timeout_sec=10.0)
            goal_handle = send_goal_future.result()

            assert goal_handle is not None
            assert goal_handle.accepted

            error_feedback_observed = _spin_until(
                lambda: any(message.log_level == 'error' for message in feedback_messages),
                client_node,
                timeout_sec=10.0,
            )
            assert error_feedback_observed

            error_feedback = next(
                message for message in feedback_messages if message.log_level == 'error'
            )
            assert error_feedback.log_source.startswith('python3-')
            assert 'simulated error from launched process' in error_feedback.log_text

            cancel_future = goal_handle.cancel_goal_async()
            rclpy.spin_until_future_complete(client_node, cancel_future, timeout_sec=10.0)

            result_future = goal_handle.get_result_async()
            rclpy.spin_until_future_complete(client_node, result_future, timeout_sec=10.0)
        finally:
            client_node.destroy_node()
            if rclpy.ok():
                rclpy.shutdown()

    mirrored_log_prefix = (
        f'[{harness.node_name}]: [capabilities2_launch/package_resolved_error_test.launch.py]'
    )
    assert mirrored_log_prefix not in harness.output


def test_action_can_optionally_mirror_child_output_to_rosout():
    rclpy.init()
    with ServerHarness([
        '--ros-args',
        '-p', 'mirror_child_logs_to_rosout:=true',
    ]) as harness:
        client_node = rclpy.create_node('capabilities2_launch_error_feedback_client_with_rosout')

        try:
            action_client = ActionClient(client_node, LaunchAction, harness.action_name)
            assert action_client.wait_for_server(timeout_sec=5.0)

            goal = LaunchAction.Goal()
            goal.reference_name = 'error-feedback-rosout'
            goal.package_name = 'capabilities2_launch'
            goal.launch_file_name = 'package_resolved_error_test.launch.py'

            send_goal_future = action_client.send_goal_async(goal)
            rclpy.spin_until_future_complete(client_node, send_goal_future, timeout_sec=10.0)
            goal_handle = send_goal_future.result()

            assert goal_handle is not None
            assert goal_handle.accepted

            time.sleep(1.0)

            cancel_future = goal_handle.cancel_goal_async()
            rclpy.spin_until_future_complete(client_node, cancel_future, timeout_sec=10.0)

            result_future = goal_handle.get_result_async()
            rclpy.spin_until_future_complete(client_node, result_future, timeout_sec=10.0)
        finally:
            client_node.destroy_node()
            if rclpy.ok():
                rclpy.shutdown()

    mirrored_log_prefix = (
        f'[{harness.node_name}]: [capabilities2_launch/package_resolved_error_test.launch.py]'
    )
    assert mirrored_log_prefix in harness.output
    assert 'simulated error from launched process' in harness.output


def test_action_passes_launch_parameters_to_launch_file():
    rclpy.init()
    with ServerHarness() as harness:
        client_node = rclpy.create_node('capabilities2_launch_argument_client')
        subscriber_node = rclpy.create_node('capabilities2_launch_argument_subscriber')
        received_messages = []

        subscriber_node.create_subscription(
            String,
            '/capabilities2_launch/argument_test_chatter',
            lambda msg: received_messages.append(msg.data),
            10,
        )

        try:
            action_client = ActionClient(client_node, LaunchAction, harness.action_name)
            assert action_client.wait_for_server(timeout_sec=5.0)

            goal = LaunchAction.Goal()
            goal.reference_name = 'argument-test'
            goal.package_name = 'capabilities2_launch'
            goal.launch_file_name = 'package_resolved_argument_test.launch.py'
            goal.launch_parameters = [
                LaunchParameter(key='test_message', value='hello-from-launch-argument'),
            ]

            send_goal_future = action_client.send_goal_async(goal)
            rclpy.spin_until_future_complete(client_node, send_goal_future, timeout_sec=10.0)
            goal_handle = send_goal_future.result()

            assert goal_handle is not None
            assert goal_handle.accepted

            message_received = _spin_until(lambda: bool(received_messages), subscriber_node, timeout_sec=10.0)
            assert message_received
            assert received_messages[0] == 'hello-from-launch-argument'

            cancel_future = goal_handle.cancel_goal_async()
            rclpy.spin_until_future_complete(client_node, cancel_future, timeout_sec=10.0)
            cancel_response = cancel_future.result()

            assert cancel_response is not None
            assert len(cancel_response.goals_canceling) == 1

            result_future = goal_handle.get_result_async()
            rclpy.spin_until_future_complete(client_node, result_future, timeout_sec=10.0)
            result = result_future.result()

            assert result is not None
            assert result.status == GoalStatus.STATUS_CANCELED
        finally:
            client_node.destroy_node()
            subscriber_node.destroy_node()
            if rclpy.ok():
                rclpy.shutdown()


def test_resource_topic_reports_main_and_child_processes():
    rclpy.init()
    with ServerHarness() as harness:
        client_node = rclpy.create_node('capabilities2_launch_resource_client')
        resource_node = rclpy.create_node('capabilities2_launch_resource_subscriber')
        resource_messages = []

        resource_node.create_subscription(
            LaunchResourceList,
            harness.resource_topic,
            lambda msg: resource_messages.append(msg),
            10,
        )

        try:
            action_client = ActionClient(client_node, LaunchAction, harness.action_name)
            assert action_client.wait_for_server(timeout_sec=5.0)

            goal = LaunchAction.Goal()
            goal.reference_name = 'process-tree'
            goal.package_name = 'capabilities2_launch'
            goal.launch_file_name = 'package_resolved_process_tree_test.launch.py'

            send_goal_future = action_client.send_goal_async(goal)
            rclpy.spin_until_future_complete(client_node, send_goal_future, timeout_sec=10.0)
            goal_handle = send_goal_future.result()

            assert goal_handle is not None
            assert goal_handle.accepted

            resource_observed = _spin_until(
                lambda: any(
                    msg.child_process_count > 0 and any(resource.reference_name == 'process-tree' for resource in msg.resources)
                    for msg in resource_messages
                ),
                resource_node,
                timeout_sec=10.0,
            )
            assert resource_observed

            matching_message = next(
                msg for msg in resource_messages
                if msg.child_process_count > 0 and any(resource.reference_name == 'process-tree' for resource in msg.resources)
            )
            matching_resources = [resource for resource in matching_message.resources if resource.reference_name == 'process-tree']

            assert any(resource.is_main_process for resource in matching_resources)
            assert any(not resource.is_main_process for resource in matching_resources)
            assert matching_message.total_memory_rss_bytes > 0
            assert matching_message.resource_count >= len(matching_resources)
            assert all(resource.key.startswith('process-tree-') for resource in matching_resources)
            assert matching_message.total_peak_sampled_cpu_usage_percent >= matching_message.total_cpu_usage_percent
            assert matching_message.total_peak_memory_rss_bytes >= matching_message.total_memory_rss_bytes
            assert matching_message.total_peak_memory_vms_bytes >= matching_message.total_memory_vms_bytes
            assert matching_message.total_time_above_80pct_peak_sampled_cpu_sec >= 0.0
            assert matching_message.total_time_above_90pct_peak_rss_sec >= 0.0
            assert matching_message.total_time_above_90pct_peak_vms_sec >= 0.0
            assert all(resource.peak_sampled_cpu_usage_percent >= resource.cpu_usage_percent for resource in matching_resources)
            assert all(resource.peak_memory_rss_bytes >= resource.memory_rss_bytes for resource in matching_resources)
            assert all(resource.peak_memory_vms_bytes >= resource.memory_vms_bytes for resource in matching_resources)
            assert all(resource.time_above_80pct_peak_sampled_cpu_sec >= 0.0 for resource in matching_resources)
            assert all(resource.time_above_90pct_peak_rss_sec >= 0.0 for resource in matching_resources)
            assert all(resource.time_above_90pct_peak_vms_sec >= 0.0 for resource in matching_resources)

            cancel_future = goal_handle.cancel_goal_async()
            rclpy.spin_until_future_complete(client_node, cancel_future, timeout_sec=10.0)
            cancel_response = cancel_future.result()

            assert cancel_response is not None
            assert len(cancel_response.goals_canceling) == 1

            result_future = goal_handle.get_result_async()
            rclpy.spin_until_future_complete(client_node, result_future, timeout_sec=10.0)
            result = result_future.result()

            assert result is not None
            assert result.status == GoalStatus.STATUS_CANCELED
        finally:
            client_node.destroy_node()
            resource_node.destroy_node()
            if rclpy.ok():
                rclpy.shutdown()