import os
import threading
from typing import Optional

from capabilities2_msgs.action import Launch as LaunchAction
import rclpy
from rclpy.action import ActionServer, CancelResponse
from rclpy.action.server import ServerGoalHandle
from rclpy.executors import ExternalShutdownException, MultiThreadedExecutor
from rclpy.node import Node

try:
    from capabilities2_launch.launch_runtime import LaunchRuntime
except ModuleNotFoundError:
    from launch_runtime import LaunchRuntime


class CapabilitiesLaunchServer(Node):
    def __init__(self, node_name: str = 'capabilities2_launch'):
        super().__init__(node_name)

        self.declare_parameter('mirror_child_logs_to_rosout', False)
        self.declare_parameter('feedback_child_log_level', 'error')

        self.runtime = LaunchRuntime(self)

        self.launch_action_server = ActionServer(
            self,
            LaunchAction,
            '~/launch',
            handle_accepted_callback=self.handle_accepted_cb,
            execute_callback=self.execute_cb,
            cancel_callback=self.cancel_cb,
        )

        self.get_logger().info('Capabilities2 launch server is ready.')

    def handle_accepted_cb(self, goal_handle: ServerGoalHandle) -> None:
        threading.Thread(target=goal_handle.execute, daemon=True).start()

    def cancel_cb(self, goal_handle: ServerGoalHandle) -> CancelResponse:
        return CancelResponse.ACCEPT

    def execute_cb(self, goal_handle: ServerGoalHandle) -> LaunchAction.Result:
        request = goal_handle.request

        try:
            reference_name = request.reference_name.strip() or ''
            launch_parameters = self.runtime.normalize_launch_parameters(request.launch_parameters)
            launch_file_path, launch_key = self.runtime.resolve_launch_reference(
                request.launch_file_path,
                request.package_name,
                request.launch_file_name,
            )
            active_launch, status = self.runtime.start_launch(
                launch_file_path,
                launch_key,
                reference_name or launch_key,
                launch_parameters,
            )
        except RuntimeError as exc:
            message = str(exc)
            self.get_logger().error(message)
            goal_handle.abort()
            return self.make_result(
                launch_id='',
                final_state='failed',
                resolved_launch_file_path='',
                message=message,
            )

        if active_launch is None:
            self.get_logger().error(status)
            goal_handle.abort()
            return self.make_result(
                launch_id='',
                final_state='rejected',
                resolved_launch_file_path=launch_file_path,
                message=status,
            )

        self.get_logger().info(
            f"Action goal started launch '{launch_key}' from '{launch_file_path}'")

        rate = self.create_rate(2.0)

        while rclpy.ok():
            if goal_handle.is_cancel_requested:
                self.runtime.stop_launch(launch_file_path)
                goal_handle.canceled()
                return self.make_result(
                    launch_id=active_launch.launch_id,
                    final_state='canceled',
                    resolved_launch_file_path=active_launch.launch_file_path,
                    message='launch canceled by action client',
                )

            if active_launch.terminal_event.is_set():
                if active_launch.state == 'failed':
                    goal_handle.abort()
                else:
                    goal_handle.succeed()
                return self.make_result(
                    launch_id=active_launch.launch_id,
                    final_state=active_launch.state,
                    resolved_launch_file_path=active_launch.launch_file_path,
                    message=active_launch.last_message or f"launch {active_launch.state}",
                )

            feedback = LaunchAction.Feedback()
            feedback.header.stamp = self.get_clock().now().to_msg()
            feedback.launch_id = active_launch.launch_id
            feedback.state = active_launch.state
            feedback.resolved_launch_file_path = active_launch.launch_file_path
            metrics = self.runtime.resource_tracker.get_or_refresh_metrics(active_launch)
            self.runtime.resource_tracker.apply_metrics_to_feedback(feedback, metrics, len(active_launch.active_pids))
            feedback.message = active_launch.last_message or f"launch {active_launch.state}"
            log_event = self.runtime.pop_feedback_log(active_launch)
            if log_event is not None:
                feedback.log_level, feedback.log_source, feedback.log_text = log_event
            goal_handle.publish_feedback(feedback)
            rate.sleep()

        goal_handle.abort()
        return self.make_result(
            launch_id='',
            final_state='failed',
            resolved_launch_file_path='',
            message='launch server shutdown before completion',
        )

    def make_result(
        self,
        launch_id: str,
        final_state: str,
        resolved_launch_file_path: str,
        message: str,
    ) -> LaunchAction.Result:
        result = LaunchAction.Result()
        result.header.stamp = self.get_clock().now().to_msg()
        result.launch_id = launch_id
        result.final_state = final_state
        result.resolved_launch_file_path = resolved_launch_file_path
        result.message = message
        return result


def main(args=None):
    try:
        rclpy.init(args=args)
        launch_server = CapabilitiesLaunchServer()

        executor = MultiThreadedExecutor()
        executor.add_node(launch_server)

        executor_thread = threading.Thread(target=executor.spin, daemon=True)
        executor_thread.start()

        launch_server.get_logger().info('running LaunchService')
        launch_server.runtime.launch_service.run(shutdown_when_idle=False)
    except ExternalShutdownException:
        pass
    except KeyboardInterrupt:
        pass
    finally:
        try:
            launch_server.runtime.launch_service.shutdown()
            executor.shutdown()
            executor_thread.join(timeout=5.0)
            launch_server.destroy_node()
        except UnboundLocalError:
            pass

        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()