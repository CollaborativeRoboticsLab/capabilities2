from types import SimpleNamespace

from capabilities2_msgs.action import Launch as LaunchAction
from capabilities2_msgs.msg import LaunchParameter, LaunchResourceList
from launch.actions import ExecuteProcess
import pytest

from capabilities2_launch.launch_runtime import LaunchRuntime
from capabilities2_launch.resource_tracker import ActiveLaunch, ProcessMetrics, ProcessResourceSnapshot, ProcessStats, ResourceTracker
import capabilities2_launch.resource_tracker as resource_tracker_module


class ContainerEntity:
    def __init__(self, *, entities=None, actions=None):
        self.entities = entities
        self.actions = actions


def make_runtime(feedback_child_log_level='error'):
    runtime = LaunchRuntime.__new__(LaunchRuntime)
    runtime.feedback_child_log_level = feedback_child_log_level
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


def test_log_classification_and_feedback_thresholds_follow_configured_level():
    runtime = make_runtime(feedback_child_log_level='warn')

    assert runtime.classify_log_level('[WARN] disk nearly full', is_stderr=False) == 'warn'
    assert runtime.classify_log_level('plain stderr line', is_stderr=True) == 'error'
    assert runtime.classify_log_level('plain stdout line', is_stderr=False) == 'info'
    assert runtime.should_send_feedback_log('warn') is True
    assert runtime.should_send_feedback_log('info') is False


def test_resource_tracker_applies_and_accumulates_metrics():
    tracker = ResourceTracker.__new__(ResourceTracker)
    feedback = LaunchAction.Feedback()
    totals = LaunchResourceList()
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

    tracker.apply_metrics_to_feedback(feedback, metrics, active_process_count=1)
    tracker.accumulate_resource_totals(totals, metrics)

    assert feedback.active_process_count == 1
    assert feedback.child_process_count == 2
    assert list(feedback.main_pids) == [101]
    assert list(feedback.child_pids) == [202, 303]
    assert feedback.peak_memory_vms_bytes == 32768
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