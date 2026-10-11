from collections import deque
import os
import threading
from dataclasses import dataclass, field
from typing import Deque, Dict, List, Optional, Set, Tuple

from ament_index_python.packages import PackageNotFoundError
from capabilities2_msgs.action import Launch as LaunchAction
from launch import LaunchService as RosLaunchService
from launch.actions import EmitEvent, ExecuteProcess, RegisterEventHandler
from launch.event_handlers import OnProcessExit, OnProcessIO, OnProcessStart
from launch.events.process import ShutdownProcess
from launch.launch_description_sources import AnyLaunchDescriptionSource
import rclpy
from rclpy.action import ActionServer, CancelResponse
from rclpy.action.server import ServerGoalHandle
from rclpy.executors import ExternalShutdownException, MultiThreadedExecutor
from rclpy.node import Node
from ros2launch.api import MultipleLaunchFilesError, get_share_file_path_from_package


@dataclass
class ActiveLaunch:
    launch_file_path: str
    launch_key: str
    launch_id: str
    stop_requested: bool = False
    state: str = 'starting'
    process_ids: Set[int] = field(default_factory=set)
    active_pids: Set[int] = field(default_factory=set)
    terminal_event: threading.Event = field(default_factory=threading.Event)
    tracked_action_count: int = 0
    last_message: str = ''
    feedback_logs: Deque[Tuple[str, str, str]] = field(default_factory=deque)
    feedback_lock: threading.Lock = field(default_factory=threading.Lock)


class CapabilitiesLaunchServer(Node):
    def __init__(self, node_name: str = 'capabilities2_launch'):
        super().__init__(node_name)

        self.declare_parameter('mirror_child_logs_to_rosout', False)
        self.declare_parameter('feedback_child_log_level', 'error')

        self.launch_service = RosLaunchService()
        self.registry_lock = threading.Lock()
        self.active_launches: Dict[str, ActiveLaunch] = {}
        self.last_terminal_status: Dict[str, str] = {}
        self.mirror_child_logs_to_rosout = bool(
            self.get_parameter('mirror_child_logs_to_rosout').value)
        self.feedback_child_log_level = str(
            self.get_parameter('feedback_child_log_level').value or 'error').lower()

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
            launch_file_path, launch_key = self.resolve_launch_reference(
                request.launch_file_path,
                request.package_name,
                request.launch_file_name,
            )
            active_launch, status = self.start_launch(launch_file_path, launch_key)
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
                self.stop_launch(launch_file_path)
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
            feedback.active_process_count = len(active_launch.active_pids)
            feedback.message = active_launch.last_message or f"launch {active_launch.state}"
            log_event = self.pop_feedback_log(active_launch)
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

    def resolve_launch_reference(
        self,
        reference: str,
        package_name: str = '',
        launch_file_name: str = '',
    ) -> Tuple[str, str]:
        if package_name and launch_file_name:
            return self.resolve_package_launch(package_name, launch_file_name), f'{package_name}/{launch_file_name}'

        if os.path.isfile(reference):
            return os.path.realpath(reference), os.path.realpath(reference)

        if ':' in reference:
            package_name, launch_file_name = reference.split(':', 1)
            return self.resolve_package_launch(package_name, launch_file_name), reference

        if not os.path.isabs(reference) and '/' in reference:
            package_name, launch_file_name = reference.split('/', 1)
            return self.resolve_package_launch(package_name, launch_file_name), reference

        raise RuntimeError(
            "Launch goal must provide an existing path or package_name plus launch_file_name, or use 'package:file.launch.py' or 'package/file.launch.py'")

    def resolve_package_launch(self, package_name: str, launch_file_name: str) -> str:
        try:
            return str(get_share_file_path_from_package(
                package_name=package_name,
                file_name=launch_file_name,
            ))
        except PackageNotFoundError as exc:
            raise RuntimeError(f"Package '{package_name}' not found: {exc}") from exc
        except (FileNotFoundError, MultipleLaunchFilesError) as exc:
            raise RuntimeError(str(exc)) from exc

    def start_launch(self, launch_file_path: str, launch_key: str) -> Tuple[Optional[ActiveLaunch], str]:
        canonical_path = os.path.realpath(launch_file_path)

        with self.registry_lock:
            if canonical_path in self.active_launches:
                return None, 'already started. ignoring'

            active_launch = ActiveLaunch(
                launch_file_path=canonical_path,
                launch_key=launch_key,
                launch_id=canonical_path,
                last_message='launch accepted',
            )
            self.active_launches[canonical_path] = active_launch
            self.last_terminal_status[canonical_path] = 'starting'

        try:
            description = AnyLaunchDescriptionSource(canonical_path).get_launch_description(
                self.launch_service.context)
            execute_actions = self.collect_execute_actions(description.entities)
            active_launch.tracked_action_count = len(execute_actions)

            for execute_action in execute_actions:
                description.add_action(RegisterEventHandler(OnProcessStart(
                    target_action=execute_action,
                    on_start=self.make_on_process_start(active_launch),
                )))
                description.add_action(RegisterEventHandler(OnProcessIO(
                    target_action=execute_action,
                    on_stdout=self.make_on_process_stdout(active_launch),
                    on_stderr=self.make_on_process_stderr(active_launch),
                )))
                description.add_action(RegisterEventHandler(OnProcessExit(
                    target_action=execute_action,
                    on_exit=self.make_on_process_exit(active_launch),
                )))

            self.launch_service.include_launch_description(description)
            active_launch.last_message = 'launch included in LaunchService'
            self.get_logger().info(f"launch included: {launch_key} -> {canonical_path}")
            return active_launch, 'successfully started'
        except Exception as exc:
            with self.registry_lock:
                self.active_launches.pop(canonical_path, None)
                self.last_terminal_status[canonical_path] = 'failed'
            raise RuntimeError(f"failed to include launch '{launch_key}': {exc}") from exc

    def stop_launch(self, launch_file_path: str) -> str:
        canonical_path = os.path.realpath(launch_file_path)

        with self.registry_lock:
            active_launch = self.active_launches.get(canonical_path)
            if active_launch is None:
                return 'already stopped. ignoring'

            active_launch.stop_requested = True
            if active_launch.state not in ('stopped', 'failed'):
                active_launch.state = 'stopping'
                active_launch.last_message = 'stop requested'

        self.get_logger().info(f"stop requested for {active_launch.launch_key}")

        if active_launch.process_ids:
            self.launch_service.emit_event(ShutdownProcess(
                process_matcher=lambda action, pids=set(active_launch.process_ids): (
                    action.process_details['pid'] in pids
                ),
            ))

        return 'successfully stopped'

    def make_on_process_start(self, active_launch: ActiveLaunch):
        def on_process_start(event, context):
            del context
            active_launch.process_ids.add(event.pid)
            active_launch.active_pids.add(event.pid)
            active_launch.state = 'running'
            active_launch.last_message = f'process started: {event.process_name}'
            self.last_terminal_status[active_launch.launch_file_path] = 'running'
            if active_launch.stop_requested:
                return EmitEvent(event=ShutdownProcess(
                    process_matcher=lambda action, pid=event.pid: action.process_details['pid'] == pid,
                ))

            return None

        return on_process_start

    def make_on_process_stdout(self, active_launch: ActiveLaunch):
        def on_process_stdout(event):
            self.log_process_output(active_launch, event, is_stderr=False)
            return None

        return on_process_stdout

    def make_on_process_stderr(self, active_launch: ActiveLaunch):
        def on_process_stderr(event):
            self.log_process_output(active_launch, event, is_stderr=True)
            return None

        return on_process_stderr

    def make_on_process_exit(self, active_launch: ActiveLaunch):
        def on_process_exit(event, context):
            del context
            active_launch.active_pids.discard(event.pid)
            return_code = getattr(event, 'returncode', None)
            active_launch.last_message = (
                f'process exited: {event.process_name} returncode={return_code}'
            )
            if return_code not in (0, None) and not active_launch.stop_requested:
                active_launch.state = 'failed'
                active_launch.last_message = (
                    f'launch failed because process {event.process_name} exited with code {return_code}'
                )
                self.queue_feedback_log(
                    active_launch,
                    'error',
                    event.process_name,
                    active_launch.last_message,
                )
                self.last_terminal_status[active_launch.launch_file_path] = 'failed'

            if not active_launch.active_pids:
                if active_launch.state != 'failed':
                    active_launch.state = 'stopped'
                    if active_launch.stop_requested:
                        active_launch.last_message = 'launch stopped after cancel request'
                    else:
                        active_launch.last_message = 'launch completed'
                    self.last_terminal_status[active_launch.launch_file_path] = 'stopped'

                with self.registry_lock:
                    self.active_launches.pop(active_launch.launch_file_path, None)

                active_launch.terminal_event.set()

            return None

        return on_process_exit

    def log_process_output(self, active_launch: ActiveLaunch, event, is_stderr: bool) -> None:
        text = event.text.decode(errors='replace') if isinstance(event.text, bytes) else str(event.text)
        for line in text.splitlines():
            if not line.strip():
                continue

            log_level = self.classify_log_level(line, is_stderr)
            message = f"[{active_launch.launch_key}] [{event.process_name}:{event.pid}] {line}"

            if self.should_send_feedback_log(log_level):
                self.queue_feedback_log(active_launch, log_level, event.process_name, line)

            if self.mirror_child_logs_to_rosout:
                self.log_to_rosout(log_level, message)

    def collect_execute_actions(self, entities) -> List[ExecuteProcess]:
        execute_actions: List[ExecuteProcess] = []

        for entity in entities:
            if isinstance(entity, ExecuteProcess):
                execute_actions.append(entity)

            nested_entities = getattr(entity, 'entities', None)
            if nested_entities:
                execute_actions.extend(self.collect_execute_actions(nested_entities))

        return execute_actions

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

    def classify_log_level(self, line: str, is_stderr: bool) -> str:
        normalized = line.upper()

        if '[FATAL]' in normalized:
            return 'fatal'
        if '[ERROR]' in normalized:
            return 'error'
        if '[WARN]' in normalized:
            return 'warn'
        if '[INFO]' in normalized:
            return 'info'
        if is_stderr:
            return 'error'
        return 'info'

    def should_send_feedback_log(self, log_level: str) -> bool:
        threshold_order = {
            'debug': 10,
            'info': 20,
            'warn': 30,
            'error': 40,
            'fatal': 50,
        }
        threshold = threshold_order.get(self.feedback_child_log_level, 40)
        current = threshold_order.get(log_level, 20)
        return current >= threshold

    def queue_feedback_log(self, active_launch: ActiveLaunch, log_level: str, log_source: str, log_text: str) -> None:
        with active_launch.feedback_lock:
            active_launch.feedback_logs.append((log_level, log_source, log_text))

    def pop_feedback_log(self, active_launch: ActiveLaunch) -> Optional[Tuple[str, str, str]]:
        with active_launch.feedback_lock:
            if not active_launch.feedback_logs:
                return None
            return active_launch.feedback_logs.popleft()

    def log_to_rosout(self, log_level: str, message: str) -> None:
        if log_level in ('fatal', 'error'):
            self.get_logger().error(message)
        elif log_level == 'warn':
            self.get_logger().warning(message)
        else:
            self.get_logger().info(message)


def main(args=None):
    try:
        rclpy.init(args=args)
        launch_server = CapabilitiesLaunchServer()

        executor = MultiThreadedExecutor()
        executor.add_node(launch_server)

        executor_thread = threading.Thread(target=executor.spin, daemon=True)
        executor_thread.start()

        launch_server.get_logger().info('running LaunchService')
        launch_server.launch_service.run(shutdown_when_idle=False)
    except ExternalShutdownException:
        pass
    except KeyboardInterrupt:
        pass
    finally:
        try:
            launch_server.launch_service.shutdown()
            executor.shutdown()
            executor_thread.join(timeout=5.0)
            launch_server.destroy_node()
        except UnboundLocalError:
            pass

        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()