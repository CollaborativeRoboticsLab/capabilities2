import os
import threading
from contextlib import contextmanager
from typing import Dict, List, Optional, Tuple

from ament_index_python.packages import PackageNotFoundError
from launch import LaunchDescription, LaunchService as RosLaunchService
from launch.actions import DeclareLaunchArgument, EmitEvent, ExecuteProcess, IncludeLaunchDescription, OpaqueFunction, RegisterEventHandler, SetLaunchConfiguration
from launch.event_handlers import OnProcessExit, OnProcessIO, OnProcessStart
from launch.events.process import ShutdownProcess
from launch.launch_description_sources import AnyLaunchDescriptionSource
from ros2launch.api import MultipleLaunchFilesError, get_share_file_path_from_package

from capabilities2_launch_proxy.launch_process_matcher import LaunchProcessMatcher
from capabilities2_launch_proxy.resource_tracker import ActiveLaunch, ResourceTracker
from capabilities2_msgs.msg import LaunchResourceList


class LaunchRuntime:
    def __init__(self, node):
        self.node = node
        self.launch_service = RosLaunchService()
        self.registry_lock = threading.Lock()
        self.active_launches: Dict[str, ActiveLaunch] = {}
        self.last_terminal_status: Dict[str, str] = {}
        self.resource_tracker = ResourceTracker()
        self.resource_publisher = self.node.create_publisher(LaunchResourceList, '~/resources', 10)
        self.resource_timer = self.node.create_timer(0.5, self.publish_resource_snapshot)
        self.mirror_child_logs_to_rosout = bool(
            self.node.get_parameter('mirror_child_logs_to_rosout').value)
        self.feedback_child_log_level = str(
            self.node.get_parameter('feedback_child_log_level').value or 'error').lower()
        self.override_output_method_to_log = bool(
            self.node.get_parameter('override_output_method_to_log').value)
        self.suppress_child_process_output_to_screen = bool(
            self.node.get_parameter('suppress_child_process_output_to_screen').value)
        self.expand_opaque_functions_for_tracking = bool(
            self.node.get_parameter('expand_opaque_functions_for_tracking').value)
        self.stop_wait_timeout_sec = float(
            self.node.get_parameter('stop_wait_timeout_sec').value)

        launch_output_override = self.get_launch_process_output_override()
        if launch_output_override is not None:
            os.environ['OVERRIDE_LAUNCH_PROCESS_OUTPUT'] = launch_output_override

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

    def normalize_launch_parameters(self, launch_parameters) -> List[Tuple[str, str]]:
        normalized_parameters: List[Tuple[str, str]] = []

        for launch_parameter in launch_parameters:
            key = launch_parameter.key.strip()
            if not key:
                raise RuntimeError('Launch parameters must provide a non-empty key')

            normalized_parameters.append((key, launch_parameter.value))

        return normalized_parameters

    def build_launch_description(
        self,
        launch_file_path: str,
        launch_parameters: List[Tuple[str, str]],
    ) -> LaunchDescription:
        with self.override_launch_process_output_to_log():
            launch_description = AnyLaunchDescriptionSource(launch_file_path).get_launch_description(
                self.launch_service.context)

        if not launch_parameters:
            return launch_description

        launch_actions = [
            SetLaunchConfiguration(name=key, value=value)
            for key, value in launch_parameters
        ]
        launch_actions.extend(launch_description.entities)

        return LaunchDescription(launch_actions)

    @contextmanager
    def override_launch_process_output_to_log(self):
        launch_output_override = self.get_launch_process_output_override()
        if launch_output_override is None:
            yield
            return

        previous_value = os.environ.get('OVERRIDE_LAUNCH_PROCESS_OUTPUT')
        os.environ['OVERRIDE_LAUNCH_PROCESS_OUTPUT'] = launch_output_override
        try:
            yield
        finally:
            if previous_value is None:
                os.environ.pop('OVERRIDE_LAUNCH_PROCESS_OUTPUT', None)
            else:
                os.environ['OVERRIDE_LAUNCH_PROCESS_OUTPUT'] = previous_value

    def get_launch_process_output_override(self) -> Optional[str]:
        if self.suppress_child_process_output_to_screen:
            return 'own_log'
        if self.override_output_method_to_log:
            return 'log'
        return None

    def start_launch(
        self,
        launch_file_path: str,
        launch_key: str,
        reference_name: str,
        launch_parameters: List[Tuple[str, str]],
    ) -> Tuple[Optional[ActiveLaunch], str]:
        canonical_path = os.path.realpath(launch_file_path)

        with self.registry_lock:
            if canonical_path in self.active_launches:
                return None, 'already started. ignoring'

            active_launch = ActiveLaunch(
                launch_file_path=canonical_path,
                launch_key=launch_key,
                launch_id=canonical_path,
                reference_name=reference_name,
                last_message='launch accepted',
            )
            self.active_launches[canonical_path] = active_launch
            self.last_terminal_status[canonical_path] = 'starting'

        try:
            description = self.build_launch_description(canonical_path, launch_parameters)
            self.prepare_active_launch(description, active_launch)

            self.launch_service.include_launch_description(description)
            active_launch.last_message = 'launch included in LaunchService'
            self.node.get_logger().info(f"launch included: {launch_key} -> {canonical_path}")
            return active_launch, 'successfully started'
        except Exception as exc:
            with self.registry_lock:
                self.active_launches.pop(canonical_path, None)
                self.last_terminal_status[canonical_path] = 'failed'
            raise RuntimeError(f"failed to include launch '{launch_key}': {exc}") from exc

    def prepare_active_launch(
        self,
        description: LaunchDescription,
        active_launch: ActiveLaunch,
    ) -> None:
        self.launch_service.context._push_launch_configurations()
        try:
            execute_actions = self.collect_execute_actions(
                description.entities,
                context=self.launch_service.context,
            )
        finally:
            self.launch_service.context._pop_launch_configurations()

        if self.suppress_child_process_output_to_screen:
            self.configure_process_output_for_feedback_only(execute_actions)

        active_launch.tracked_action_count = len(execute_actions)
        active_launch.process_matcher = LaunchProcessMatcher.from_execute_actions(execute_actions)
        self.register_process_event_handlers(description, active_launch)

    def register_process_event_handlers(
        self,
        description: LaunchDescription,
        active_launch: ActiveLaunch,
    ) -> None:
        description.add_action(RegisterEventHandler(OnProcessStart(
            on_start=self.make_on_process_start(active_launch),
        )))
        description.add_action(RegisterEventHandler(OnProcessIO(
            on_stdout=self.make_on_process_stdout(active_launch),
            on_stderr=self.make_on_process_stderr(active_launch),
        )))
        description.add_action(RegisterEventHandler(OnProcessExit(
            on_exit=self.make_on_process_exit(active_launch),
        )))

    def stop_launch(self, launch_file_path: str, wait_timeout_sec: Optional[float] = None) -> str:
        canonical_path = os.path.realpath(launch_file_path)

        with self.registry_lock:
            active_launch = self.active_launches.get(canonical_path)
            if active_launch is None:
                return 'already stopped. ignoring'

            active_launch.stop_requested = True
            if active_launch.state not in ('stopped', 'failed'):
                active_launch.state = 'stopping'
                active_launch.last_message = 'stop requested'

        self.node.get_logger().info(f"stop requested for {active_launch.launch_key}")

        if active_launch.process_ids:
            self.launch_service.emit_event(ShutdownProcess(
                process_matcher=lambda action, pids=set(active_launch.process_ids): (
                    action.process_details['pid'] in pids
                ),
            ))

        if wait_timeout_sec is None:
            return 'successfully stopped'

        if wait_timeout_sec <= 0.0 or active_launch.terminal_event.wait(wait_timeout_sec):
            return 'successfully stopped'

        return 'stop requested; shutdown still in progress'

    def make_on_process_start(self, active_launch: ActiveLaunch):
        def on_process_start(event, context):
            del context
            if not self.event_matches_active_launch(active_launch, event):
                return None

            active_launch.process_ids.add(event.pid)
            active_launch.active_pids.add(event.pid)
            active_launch.process_names[event.pid] = event.process_name
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
            if not self.event_matches_active_launch(active_launch, event):
                return None

            self.log_process_output(active_launch, event, is_stderr=False)
            return None

        return on_process_stdout

    def make_on_process_stderr(self, active_launch: ActiveLaunch):
        def on_process_stderr(event):
            if not self.event_matches_active_launch(active_launch, event):
                return None

            self.log_process_output(active_launch, event, is_stderr=True)
            return None

        return on_process_stderr

    def make_on_process_exit(self, active_launch: ActiveLaunch):
        def on_process_exit(event, context):
            del context
            if not self.event_matches_active_launch(active_launch, event):
                return None

            active_launch.active_pids.discard(event.pid)
            active_launch.process_names.pop(event.pid, None)
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

    def publish_resource_snapshot(self) -> None:
        with self.registry_lock:
            active_launches = list(self.active_launches.values())

        message = LaunchResourceList()
        message.header.stamp = self.node.get_clock().now().to_msg()
        message.active_launch_count = len(active_launches)

        resources = []

        for active_launch in active_launches:
            metrics = self.resource_tracker.refresh_metrics(active_launch)
            self.resource_tracker.accumulate_resource_totals(message, metrics)

            for resource in metrics.resources:
                resources.append(self.resource_tracker.make_launch_resource_message(resource))

        message.resource_count = len(resources)
        message.resources = resources
        self.resource_publisher.publish(message)

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

    def collect_execute_actions(self, entities, context=None) -> List[ExecuteProcess]:
        execute_actions: List[ExecuteProcess] = []

        for entity in entities:
            if isinstance(entity, ExecuteProcess):
                execute_actions.append(entity)

            if isinstance(entity, IncludeLaunchDescription) and context is not None:
                context._push_launch_configurations()
                try:
                    expanded_entities = entity.execute(context)
                    if expanded_entities:
                        execute_actions.extend(
                            self.collect_execute_actions(expanded_entities, context=context)
                        )
                finally:
                    context._pop_launch_configurations()
                continue

            if isinstance(entity, DeclareLaunchArgument) and context is not None:
                entity.execute(context)
                continue

            if isinstance(entity, SetLaunchConfiguration) and context is not None:
                entity.execute(context)
                continue

            if (
                isinstance(entity, OpaqueFunction)
                and context is not None
                and self.expand_opaque_functions_for_tracking
            ):
                expanded_entities = entity.execute(context)
                if expanded_entities:
                    execute_actions.extend(
                        self.collect_execute_actions(expanded_entities, context=context)
                    )
                continue

            launch_description_source = getattr(entity, 'launch_description_source', None)
            if launch_description_source is not None and context is not None:
                included_launch_description = launch_description_source.get_launch_description(context)
                execute_actions.extend(
                    self.collect_execute_actions(included_launch_description.entities, context=context)
                )

            sub_entities_getter = getattr(entity, 'get_sub_entities', None)
            if callable(sub_entities_getter):
                sub_entities = sub_entities_getter()
                if sub_entities:
                    execute_actions.extend(self.collect_execute_actions(sub_entities, context=context))

            nested_entities = getattr(entity, 'entities', None)
            if nested_entities:
                execute_actions.extend(self.collect_execute_actions(nested_entities, context=context))

            nested_actions = getattr(entity, 'actions', None)
            if nested_actions:
                execute_actions.extend(self.collect_execute_actions(nested_actions, context=context))

        return execute_actions

    def configure_process_output_for_feedback_only(
        self,
        execute_actions: List[ExecuteProcess],
    ) -> None:
        for execute_action in execute_actions:
            execute_action._ExecuteLocal__output = {'both': 'own_log'}

    def collect_process_action_signatures(self, execute_actions: List[ExecuteProcess]) -> set:
        return {
            self.make_process_action_signature(execute_action)
            for execute_action in execute_actions
        }

    def event_matches_active_launch(self, active_launch: ActiveLaunch, event) -> bool:
        with self.registry_lock:
            if self.active_launches.get(active_launch.launch_file_path) is not active_launch:
                return False

        if active_launch.process_matcher is None:
            return False

        return active_launch.process_matcher.matches(event)

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
            self.node.get_logger().error(message)
        elif log_level == 'warn':
            self.node.get_logger().warning(message)
        else:
            self.node.get_logger().info(message)