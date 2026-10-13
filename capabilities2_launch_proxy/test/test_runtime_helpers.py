from types import SimpleNamespace
import os
import threading

from capabilities2_msgs.action import Launch as LaunchAction
from capabilities2_msgs.msg import LaunchParameter, LaunchResourceList
from launch import LaunchContext
from launch.actions import DeclareLaunchArgument, ExecuteProcess, OpaqueFunction, SetLaunchConfiguration
from launch.substitutions import LaunchConfiguration
import pytest

from capabilities2_launch_proxy.launch_runtime import LaunchRuntime
import capabilities2_launch_proxy.launch_runtime as launch_runtime_module
from capabilities2_launch_proxy.resource_tracker import ActiveLaunch, ProcessMetrics, ProcessResourceSnapshot, ProcessStats, ResourceTracker
import capabilities2_launch_proxy.resource_tracker as resource_tracker_module


class ContainerEntity:
    def __init__(self, *, entities=None, actions=None):
        self.entities = entities
        self.actions = actions


def make_runtime(feedback_child_log_level='error'):
    runtime = LaunchRuntime.__new__(LaunchRuntime)
    runtime.feedback_child_log_level = feedback_child_log_level
    runtime.override_output_method_to_log = False
    runtime.suppress_child_process_output_to_screen = False
    runtime.expand_opaque_functions_for_tracking = True
    return runtime


def test_normalize_launch_parameters_strips_keys_and_preserves_values():
    runtime = make_runtime()
    launch_parameter = LaunchParameter()
    launch_parameter.key = '  world_name '
    launch_parameter.value = 'maze'

    assert runtime.normalize_launch_parameters([launch_parameter]) == [('world_name', 'maze')]


def test_normalize_launch_parameters_rejects_empty_keys():
    runtime = make_runtime()
    launch_parameter = LaunchParameter()
    launch_parameter.key = '   '
    launch_parameter.value = 'maze'

    with pytest.raises(RuntimeError, match='non-empty key'):
        runtime.normalize_launch_parameters([launch_parameter])


def test_collect_execute_actions_descends_nested_entities():
    runtime = make_runtime()
    root_process = ExecuteProcess(cmd=['python3', '-c', 'print("root")'])
    nested_process = ExecuteProcess(cmd=['python3', '-c', 'print("nested")'])
    entities = [
        root_process,
        ContainerEntity(entities=[ContainerEntity(actions=[nested_process])]),
    ]

    assert runtime.collect_execute_actions(entities) == [root_process, nested_process]


def test_collect_execute_actions_expands_context_dependent_include_sources():
    runtime = make_runtime()
    included_process = ExecuteProcess(cmd=['python3', '-c', 'print("included")'])

    class FakeLaunchDescriptionSource:
        def get_launch_description(self, context):
            assert context is include_context
            return SimpleNamespace(entities=[included_process])

    include_context = object()
    include_entity = SimpleNamespace(launch_description_source=FakeLaunchDescriptionSource())

    assert runtime.collect_execute_actions([include_entity], context=include_context) == [included_process]


def test_collect_execute_actions_expands_opaque_function_actions():
    runtime = make_runtime()
    included_process = ExecuteProcess(cmd=['python3', '-c', 'print("opaque")'])
    opaque = OpaqueFunction(function=lambda context: [included_process])

    assert runtime.collect_execute_actions([opaque], context=LaunchContext()) == [included_process]


def test_collect_execute_actions_can_skip_opaque_function_expansion():
    runtime = make_runtime()
    runtime.expand_opaque_functions_for_tracking = False
    included_process = ExecuteProcess(cmd=['python3', '-c', 'print("opaque")'])
    opaque = OpaqueFunction(function=lambda context: [included_process])

    assert runtime.collect_execute_actions([opaque], context=LaunchContext()) == []


def test_collect_execute_actions_applies_launch_configurations_before_opaque_functions():
    runtime = make_runtime()
    included_process = ExecuteProcess(cmd=['python3', '-c', 'print("configured")'])

    def expand_with_context(context):
        if LaunchConfiguration('robot_name').perform(context) != 'turtlebot4':
            return []
        return [included_process]

    entities = [
        SetLaunchConfiguration('robot_name', 'turtlebot4'),
        OpaqueFunction(function=expand_with_context),
    ]

    assert runtime.collect_execute_actions(entities, context=LaunchContext()) == [included_process]


def test_collect_execute_actions_applies_declared_launch_argument_defaults_before_opaque_functions():
    runtime = make_runtime()
    included_process = ExecuteProcess(cmd=['python3', '-c', 'print("defaulted")'])

    def expand_with_context(context):
        if LaunchConfiguration('robot_name').perform(context) != 'turtlebot4':
            return []
        return [included_process]

    entities = [
        DeclareLaunchArgument('robot_name', default_value='turtlebot4'),
        OpaqueFunction(function=expand_with_context),
    ]

    assert runtime.collect_execute_actions(entities, context=LaunchContext()) == [included_process]


def test_configure_process_output_for_feedback_only_disables_screen_output():
    runtime = make_runtime()
    root_process = ExecuteProcess(cmd=['python3', '-c', 'print("root")'])
    nested_process = ExecuteProcess(cmd=['python3', '-c', 'print("nested")'])

    runtime.configure_process_output_for_feedback_only([root_process, nested_process])

    assert root_process.output == {'both': 'own_log'}
    assert nested_process.output == {'both': 'own_log'}


def test_build_launch_description_temporarily_overrides_launch_output(monkeypatch):
    runtime = make_runtime()
    runtime.override_output_method_to_log = True
    runtime.launch_service = SimpleNamespace(context=object())
    captured = {}

    class FakeLaunchDescriptionSource:
        def __init__(self, launch_file_path):
            captured['launch_file_path'] = launch_file_path

        def get_launch_description(self, context):
            captured['context'] = context
            captured['override_output'] = os.environ.get('OVERRIDE_LAUNCH_PROCESS_OUTPUT')
            return SimpleNamespace(entities=['child-entity'])

    monkeypatch.setattr(launch_runtime_module, 'AnyLaunchDescriptionSource', FakeLaunchDescriptionSource)
    monkeypatch.delenv('OVERRIDE_LAUNCH_PROCESS_OUTPUT', raising=False)

    description = runtime.build_launch_description('/tmp/demo.launch.py', [])

    assert captured['launch_file_path'] == '/tmp/demo.launch.py'
    assert captured['context'] is runtime.launch_service.context
    assert captured['override_output'] == 'log'
    assert os.environ.get('OVERRIDE_LAUNCH_PROCESS_OUTPUT') is None
    assert list(description.entities) == ['child-entity']


def test_build_launch_description_prefers_own_log_when_screen_suppression_enabled(monkeypatch):
    runtime = make_runtime()
    runtime.override_output_method_to_log = True
    runtime.suppress_child_process_output_to_screen = True
    runtime.launch_service = SimpleNamespace(context=object())
    captured = {}

    class FakeLaunchDescriptionSource:
        def __init__(self, launch_file_path):
            captured['launch_file_path'] = launch_file_path

        def get_launch_description(self, context):
            captured['context'] = context
            captured['override_output'] = os.environ.get('OVERRIDE_LAUNCH_PROCESS_OUTPUT')
            return SimpleNamespace(entities=['child-entity'])

    monkeypatch.setattr(launch_runtime_module, 'AnyLaunchDescriptionSource', FakeLaunchDescriptionSource)
    monkeypatch.delenv('OVERRIDE_LAUNCH_PROCESS_OUTPUT', raising=False)

    description = runtime.build_launch_description('/tmp/demo.launch.py', [])

    assert captured['launch_file_path'] == '/tmp/demo.launch.py'
    assert captured['context'] is runtime.launch_service.context
    assert captured['override_output'] == 'own_log'
    assert os.environ.get('OVERRIDE_LAUNCH_PROCESS_OUTPUT') is None
    assert list(description.entities) == ['child-entity']


def test_log_classification_and_feedback_thresholds_follow_configured_level():
    runtime = make_runtime(feedback_child_log_level='warn')

    assert runtime.classify_log_level('[WARN] disk nearly full', is_stderr=False) == 'warn'
    assert runtime.classify_log_level('plain stderr line', is_stderr=True) == 'error'
    assert runtime.classify_log_level('plain stdout line', is_stderr=False) == 'info'
    assert runtime.should_send_feedback_log('warn') is True
    assert runtime.should_send_feedback_log('info') is False


def test_stop_launch_waits_for_terminal_event_before_reporting_success():
    runtime = make_runtime()
    emitted_events = []
    active_launch = ActiveLaunch(
        launch_file_path='/tmp/demo.launch.py',
        launch_key='demo',
        launch_id='demo-id',
        reference_name='demo',
        process_ids={123},
    )
    runtime.registry_lock = threading.Lock()
    runtime.active_launches = {'/tmp/demo.launch.py': active_launch}
    runtime.launch_service = SimpleNamespace(emit_event=lambda event: emitted_events.append(event))
    runtime.node = SimpleNamespace(get_logger=lambda: SimpleNamespace(info=lambda message: None))

    threading.Timer(0.01, active_launch.terminal_event.set).start()

    assert runtime.stop_launch('/tmp/demo.launch.py', wait_timeout_sec=0.5) == 'successfully stopped'
    assert active_launch.stop_requested is True
    assert active_launch.state == 'stopping'
    assert len(emitted_events) == 1


def test_resource_tracker_applies_and_accumulates_metrics():
    tracker = ResourceTracker.__new__(ResourceTracker)
    feedback = LaunchAction.Feedback()
    totals = LaunchResourceList()
    active_launch = ActiveLaunch(
        launch_file_path='/tmp/demo.launch.py',
        launch_key='demo',
        launch_id='demo-id',
        reference_name='demo',
        state='running',
    )
    metrics = ProcessMetrics(
        main_pids=[101],
        child_pids=[202, 303],
        cpu_usage_percent=12.5,
        peak_sampled_cpu_usage_percent=18.0,
        time_above_80pct_peak_sampled_cpu_sec=1.5,
        memory_rss_bytes=4096,
        peak_memory_rss_bytes=8192,
        time_above_90pct_peak_rss_sec=2.5,
        memory_vms_bytes=16384,
        peak_memory_vms_bytes=32768,
        time_above_90pct_peak_vms_sec=3.5,
        launch_uptime_sec=9.0,
    )

    tracker.apply_metrics_to_feedback(feedback, active_launch, metrics, active_process_count=1)
    tracker.accumulate_resource_totals(totals, metrics)

    assert feedback.active_process_count == 1
    assert feedback.child_process_count == 2
    assert list(feedback.main_pids) == [101]
    assert list(feedback.child_pids) == [202, 303]
    assert feedback.resource.launch_id == 'demo-id'
    assert feedback.resource.pid == 101
    assert feedback.resource.peak_memory_vms_bytes == 32768
    assert totals.main_process_count == 1
    assert totals.child_process_count == 2
    assert totals.total_peak_memory_rss_bytes == 8192
    assert totals.total_time_above_90pct_peak_vms_sec == pytest.approx(3.5)


def test_resource_tracker_refresh_metrics_tracks_children_and_peaks(monkeypatch):
    tracker = ResourceTracker.__new__(ResourceTracker)
    tracker.clock_ticks_per_second = 100
    tracker.memory_page_size = 4096
    active_launch = ActiveLaunch(
        launch_file_path='/tmp/demo.launch.py',
        launch_key='demo',
        launch_id='demo-id',
        reference_name='demo',
        state='running',
        process_ids={11},
        active_pids={11},
        process_names={11: 'main'},
        started_at_monotonic=90.0,
        last_metrics_sample_timestamp=98.0,
    )
    active_launch.cpu_samples = {
        11: (150, 98.0),
        22: (80, 98.0),
    }

    monkeypatch.setattr(resource_tracker_module.time, 'monotonic', lambda: 100.0)
    monkeypatch.setattr(tracker, 'read_system_uptime_sec', lambda: 500.0)
    monkeypatch.setattr(
        tracker,
        'collect_process_tree',
        lambda root_pids, root_process_names: {
            11: ProcessStats(
                pid=11,
                ppid=1,
                process_name='main',
                start_ticks=1000,
                cpu_jiffies=200,
                rss_bytes=4096,
                vms_bytes=8192,
            ),
            22: ProcessStats(
                pid=22,
                ppid=11,
                process_name='child',
                start_ticks=1200,
                cpu_jiffies=120,
                rss_bytes=2048,
                vms_bytes=4096,
            ),
        },
    )

    metrics = tracker.refresh_metrics(active_launch)

    assert metrics.main_pids == [11]
    assert metrics.child_pids == [22]
    assert metrics.cpu_usage_percent == pytest.approx(45.0)
    assert metrics.peak_sampled_cpu_usage_percent == pytest.approx(45.0)
    assert metrics.time_above_80pct_peak_sampled_cpu_sec == pytest.approx(2.0)
    assert metrics.memory_rss_bytes == 6144
    assert metrics.peak_memory_rss_bytes == 6144
    assert metrics.time_above_90pct_peak_rss_sec == pytest.approx(2.0)
    assert metrics.memory_vms_bytes == 12288
    assert metrics.peak_memory_vms_bytes == 12288
    assert metrics.time_above_90pct_peak_vms_sec == pytest.approx(2.0)
    assert len(metrics.resources) == 2
    assert {resource.pid for resource in metrics.resources} == {11, 22}
    assert any(resource.is_main_process for resource in metrics.resources)
    assert active_launch.latest_metrics is metrics


def test_make_launch_resource_message_maps_snapshot_fields():
    tracker = ResourceTracker.__new__(ResourceTracker)
    snapshot = ProcessResourceSnapshot(
        key='demo-main-11',
        reference_name='demo',
        launch_id='launch-id',
        state='running',
        resolved_launch_file_path='/tmp/demo.launch.py',
        process_name='main',
        pid=11,
        parent_pid=1,
        is_main_process=True,
        launch_uptime_sec=5.0,
        process_uptime_sec=4.0,
        cpu_usage_percent=12.0,
        peak_sampled_cpu_usage_percent=20.0,
        time_above_80pct_peak_sampled_cpu_sec=1.0,
        memory_rss_bytes=100,
        peak_memory_rss_bytes=200,
        time_above_90pct_peak_rss_sec=2.0,
        memory_vms_bytes=300,
        peak_memory_vms_bytes=400,
        time_above_90pct_peak_vms_sec=3.0,
    )

    message = tracker.make_launch_resource_message(snapshot)

    assert message.key == 'demo-main-11'
    assert message.pid == 11
    assert message.is_main_process is True
    assert message.peak_sampled_cpu_usage_percent == pytest.approx(20.0)
    assert message.peak_memory_vms_bytes == 400