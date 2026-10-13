import os
import threading
from typing import Optional

import rclpy
from rclpy.action import ActionServer, CancelResponse
from rclpy.action.server import ServerGoalHandle
from rclpy.executors import ExternalShutdownException, MultiThreadedExecutor
from rclpy.node import Node

from capabilities2_launch_proxy.launch_runtime import LaunchRuntime
from capabilities2_msgs.action import Launch as LaunchAction
from capabilities2_msgs.srv import Launch as LaunchService


class CapabilitiesLaunchServer(Node):
    RUNTIME_PARAMETER_DEFAULTS = {
        'mirror_child_logs_to_rosout': False,
        'feedback_child_log_level': 'error',
        'override_output_method_to_log': False,
        'suppress_child_process_output_to_screen': False,
        'expand_opaque_functions_for_tracking': True,
        'stop_wait_timeout_sec': 10.0,
    }

    def __init__(self, node_name: str = 'capabilities2_launch_proxy'):
        super().__init__(node_name)

        self.declare_runtime_parameters()
        self.log_runtime_parameters()

        self.runtime = LaunchRuntime(self)

        self.launch_service_server = self.create_service(
            LaunchService,
            '~/launch',
            self.launch_service_cb,
        )
        self.stop_launch_service_server = self.create_service(
            LaunchService,
            '~/stop_launch',
            self.stop_launch_service_cb,
        )

        self.launch_action_server = ActionServer(
            self,
            LaunchAction,
            '~/launch',
            handle_accepted_callback=self.handle_accepted_cb,
            execute_callback=self.execute_cb,
            cancel_callback=self.cancel_cb,
        )

        self.get_logger().info('Capabilities2 launch server is ready.')

    def declare_runtime_parameters(self) -> None:
        for name, default_value in self.RUNTIME_PARAMETER_DEFAULTS.items():
            self.declare_parameter(name, default_value)

    def get_runtime_parameters(self) -> dict:
        return {
            name: self.get_parameter(name).value
            for name in self.RUNTIME_PARAMETER_DEFAULTS
        }

    def log_runtime_parameters(self) -> None:
        runtime_parameters = self.get_runtime_parameters()
        formatted_parameters = ', '.join(
            f'{name}={value}'
            for name, value in runtime_parameters.items()
        )
        self.get_logger().info(f'main ROS parameters loaded: {formatted_parameters}')

    def resolve_start_request(
        self,
        launch_file_path: str,
        launch_key: str,
        reference_name: str,
        launch_parameters,
    ):
        return self.runtime.start_launch(
            launch_file_path,
            launch_key,
            reference_name,
            launch_parameters,
        )

    def launch_service_cb(
        self,
        request: LaunchService.Request,
        response: LaunchService.Response,
    ) -> LaunchService.Response:
        response.header.stamp = self.get_clock().now().to_msg()

        try:
            launch_file_path, launch_key = self.resolve_service_request(request)
            reference_name = request.reference_name.strip() or launch_key
            launch_parameters = self.runtime.normalize_launch_parameters(request.launch_parameters)
            active_launch, status = self.resolve_start_request(
                launch_file_path,
                launch_key,
                reference_name,
                launch_parameters,
            )
        except RuntimeError as exc:
            message = str(exc)
            self.get_logger().error(message)
            response.status = message
            return response

        if active_launch is None:
            response.status = status
            return response

        response.status = status
        self.get_logger().info(
            f"Service started launch '{reference_name}' from '{launch_file_path}'")
        return response

    def stop_launch_service_cb(
        self,
        request: LaunchService.Request,
        response: LaunchService.Response,
    ) -> LaunchService.Response:
        response.header.stamp = self.get_clock().now().to_msg()

        try:
            launch_file_path, _ = self.resolve_service_request(request)
            response.status = self.runtime.stop_launch(
                launch_file_path,
                wait_timeout_sec=self.runtime.stop_wait_timeout_sec,
            )
        except RuntimeError as exc:
            message = str(exc)
            self.get_logger().error(message)
            response.status = message

        return response

    def resolve_service_request(self, request: LaunchService.Request):
        launch_file_path = request.launch_file_path.strip()
        package_name = request.package_name.strip()
        launch_file_name = request.launch_file_name.strip()

        if launch_file_path:
            return self.runtime.resolve_launch_reference(launch_file_path)

        if package_name:
            return self.runtime.resolve_launch_reference('', package_name, launch_file_name)

        return self.runtime.resolve_launch_reference(launch_file_name)

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
            active_launch, status = self.resolve_start_request(
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
                stop_status = self.runtime.stop_launch(
                    launch_file_path,
                    wait_timeout_sec=self.runtime.stop_wait_timeout_sec,
                )
                goal_handle.canceled()
                return self.make_result(
                    launch_id=active_launch.launch_id,
                    final_state='canceled',
                    resolved_launch_file_path=active_launch.launch_file_path,
                    message=stop_status,
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
            self.runtime.resource_tracker.apply_metrics_to_feedback(
                feedback,
                active_launch,
                metrics,
                len(active_launch.active_pids),
            )
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