from collections import deque
import os
import threading
import time
from dataclasses import dataclass, field
from typing import Any, Deque, Dict, List, Optional, Set, Tuple

from capabilities2_msgs.action import Launch as LaunchAction
from capabilities2_msgs.msg import LaunchResource, LaunchResourceList


@dataclass
class ActiveLaunch:
    launch_file_path: str
    launch_key: str
    launch_id: str
    reference_name: str
    stop_requested: bool = False
    state: str = 'starting'
    process_ids: Set[int] = field(default_factory=set)
    active_pids: Set[int] = field(default_factory=set)
    process_names: Dict[int, str] = field(default_factory=dict)
    terminal_event: threading.Event = field(default_factory=threading.Event)
    tracked_action_count: int = 0
    last_message: str = ''
    feedback_logs: Deque[Tuple[str, str, str]] = field(default_factory=deque)
    feedback_lock: threading.Lock = field(default_factory=threading.Lock)
    started_at_monotonic: float = field(default_factory=time.monotonic)
    cpu_samples: Dict[int, Tuple[int, float]] = field(default_factory=dict)
    process_peak_sampled_cpu_usage_percent: Dict[int, float] = field(default_factory=dict)
    process_peak_memory_rss_bytes: Dict[int, int] = field(default_factory=dict)
    process_peak_memory_vms_bytes: Dict[int, int] = field(default_factory=dict)
    process_time_above_80pct_peak_sampled_cpu_sec: Dict[int, float] = field(default_factory=dict)
    process_time_above_90pct_peak_rss_sec: Dict[int, float] = field(default_factory=dict)
    process_time_above_90pct_peak_vms_sec: Dict[int, float] = field(default_factory=dict)
    peak_sampled_cpu_usage_percent: float = 0.0
    peak_memory_rss_bytes: int = 0
    peak_memory_vms_bytes: int = 0
    time_above_80pct_peak_sampled_cpu_sec: float = 0.0
    time_above_90pct_peak_rss_sec: float = 0.0
    time_above_90pct_peak_vms_sec: float = 0.0
    last_metrics_sample_timestamp: Optional[float] = None
    latest_metrics: Optional['ProcessMetrics'] = None
    metrics_lock: threading.Lock = field(default_factory=threading.Lock)
    process_matcher: Optional[Any] = None


@dataclass
class ProcessStats:
    pid: int
    ppid: int
    process_name: str
    start_ticks: int
    cpu_jiffies: int
    rss_bytes: int
    vms_bytes: int


@dataclass
class ProcessResourceSnapshot:
    key: str
    reference_name: str
    launch_id: str
    state: str
    resolved_launch_file_path: str
    process_name: str
    pid: int
    parent_pid: int
    is_main_process: bool
    launch_uptime_sec: float
    process_uptime_sec: float
    cpu_usage_percent: float
    peak_sampled_cpu_usage_percent: float
    time_above_80pct_peak_sampled_cpu_sec: float
    memory_rss_bytes: int
    peak_memory_rss_bytes: int
    time_above_90pct_peak_rss_sec: float
    memory_vms_bytes: int
    peak_memory_vms_bytes: int
    time_above_90pct_peak_vms_sec: float


@dataclass
class ProcessMetrics:
    main_pids: List[int] = field(default_factory=list)
    child_pids: List[int] = field(default_factory=list)
    cpu_usage_percent: float = 0.0
    peak_sampled_cpu_usage_percent: float = 0.0
    time_above_80pct_peak_sampled_cpu_sec: float = 0.0
    memory_rss_bytes: int = 0
    peak_memory_rss_bytes: int = 0
    time_above_90pct_peak_rss_sec: float = 0.0
    memory_vms_bytes: int = 0
    peak_memory_vms_bytes: int = 0
    time_above_90pct_peak_vms_sec: float = 0.0
    launch_uptime_sec: float = 0.0
    resources: List[ProcessResourceSnapshot] = field(default_factory=list)


class ResourceTracker:
    def __init__(self):
        self.clock_ticks_per_second = os.sysconf(os.sysconf_names['SC_CLK_TCK'])
        self.memory_page_size = os.sysconf(os.sysconf_names['SC_PAGE_SIZE'])

    def apply_metrics_to_feedback(
        self,
        feedback: LaunchAction.Feedback,
        active_launch: ActiveLaunch,
        metrics: ProcessMetrics,
        active_process_count: int,
    ) -> None:
        feedback.active_process_count = active_process_count
        feedback.child_process_count = len(metrics.child_pids)
        feedback.main_pids = metrics.main_pids
        feedback.child_pids = metrics.child_pids
        feedback.resource = self.make_feedback_resource_message(active_launch, metrics)

    def accumulate_resource_totals(
        self,
        message: LaunchResourceList,
        metrics: ProcessMetrics,
    ) -> None:
        message.main_process_count += len(metrics.main_pids)
        message.child_process_count += len(metrics.child_pids)
        message.total_cpu_usage_percent += metrics.cpu_usage_percent
        message.total_peak_sampled_cpu_usage_percent += metrics.peak_sampled_cpu_usage_percent
        message.total_time_above_80pct_peak_sampled_cpu_sec += metrics.time_above_80pct_peak_sampled_cpu_sec
        message.total_memory_rss_bytes += metrics.memory_rss_bytes
        message.total_peak_memory_rss_bytes += metrics.peak_memory_rss_bytes
        message.total_time_above_90pct_peak_rss_sec += metrics.time_above_90pct_peak_rss_sec
        message.total_memory_vms_bytes += metrics.memory_vms_bytes
        message.total_peak_memory_vms_bytes += metrics.peak_memory_vms_bytes
        message.total_time_above_90pct_peak_vms_sec += metrics.time_above_90pct_peak_vms_sec

    def make_launch_resource_message(self, resource: ProcessResourceSnapshot) -> LaunchResource:
        resource_message = LaunchResource()
        resource_message.key = resource.key
        resource_message.reference_name = resource.reference_name
        resource_message.launch_id = resource.launch_id
        resource_message.state = resource.state
        resource_message.resolved_launch_file_path = resource.resolved_launch_file_path
        resource_message.process_name = resource.process_name
        resource_message.pid = resource.pid
        resource_message.parent_pid = resource.parent_pid
        resource_message.is_main_process = resource.is_main_process
        resource_message.launch_uptime_sec = resource.launch_uptime_sec
        resource_message.process_uptime_sec = resource.process_uptime_sec
        resource_message.cpu_usage_percent = resource.cpu_usage_percent
        resource_message.peak_sampled_cpu_usage_percent = resource.peak_sampled_cpu_usage_percent
        resource_message.time_above_80pct_peak_sampled_cpu_sec = resource.time_above_80pct_peak_sampled_cpu_sec
        resource_message.memory_rss_bytes = resource.memory_rss_bytes
        resource_message.peak_memory_rss_bytes = resource.peak_memory_rss_bytes
        resource_message.time_above_90pct_peak_rss_sec = resource.time_above_90pct_peak_rss_sec
        resource_message.memory_vms_bytes = resource.memory_vms_bytes
        resource_message.peak_memory_vms_bytes = resource.peak_memory_vms_bytes
        resource_message.time_above_90pct_peak_vms_sec = resource.time_above_90pct_peak_vms_sec
        return resource_message

    def make_feedback_resource_message(
        self,
        active_launch: ActiveLaunch,
        metrics: ProcessMetrics,
    ) -> LaunchResource:
        resource_message = LaunchResource()
        primary_pid = metrics.main_pids[0] if metrics.main_pids else 0
        primary_resource = next(
            (resource for resource in metrics.resources if resource.pid == primary_pid),
            None,
        )
        resource_message.key = active_launch.launch_key
        resource_message.reference_name = active_launch.reference_name
        resource_message.launch_id = active_launch.launch_id
        resource_message.state = active_launch.state
        resource_message.resolved_launch_file_path = active_launch.launch_file_path
        resource_message.process_name = '' if primary_resource is None else primary_resource.process_name
        resource_message.pid = primary_pid
        resource_message.parent_pid = 0 if primary_resource is None else primary_resource.parent_pid
        resource_message.is_main_process = primary_pid != 0
        resource_message.launch_uptime_sec = metrics.launch_uptime_sec
        resource_message.process_uptime_sec = (
            metrics.launch_uptime_sec if primary_resource is None else primary_resource.process_uptime_sec
        )
        resource_message.cpu_usage_percent = metrics.cpu_usage_percent
        resource_message.peak_sampled_cpu_usage_percent = metrics.peak_sampled_cpu_usage_percent
        resource_message.time_above_80pct_peak_sampled_cpu_sec = metrics.time_above_80pct_peak_sampled_cpu_sec
        resource_message.memory_rss_bytes = metrics.memory_rss_bytes
        resource_message.peak_memory_rss_bytes = metrics.peak_memory_rss_bytes
        resource_message.time_above_90pct_peak_rss_sec = metrics.time_above_90pct_peak_rss_sec
        resource_message.memory_vms_bytes = metrics.memory_vms_bytes
        resource_message.peak_memory_vms_bytes = metrics.peak_memory_vms_bytes
        resource_message.time_above_90pct_peak_vms_sec = metrics.time_above_90pct_peak_vms_sec
        return resource_message

    def get_or_refresh_metrics(self, active_launch: ActiveLaunch) -> ProcessMetrics:
        with active_launch.metrics_lock:
            if active_launch.latest_metrics is not None:
                return active_launch.latest_metrics

        return self.refresh_metrics(active_launch)

    def refresh_metrics(self, active_launch: ActiveLaunch) -> ProcessMetrics:
        with active_launch.metrics_lock:
            process_stats = self.collect_process_tree(active_launch.active_pids, active_launch.process_names)
            live_main_pids = sorted(pid for pid in active_launch.active_pids if pid in process_stats)
            child_pids = sorted(pid for pid in process_stats if pid not in live_main_pids)
            now = time.monotonic()
            sample_elapsed = 0.0 if active_launch.last_metrics_sample_timestamp is None else max(
                now - active_launch.last_metrics_sample_timestamp,
                0.0,
            )
            launch_uptime_sec = max(now - active_launch.started_at_monotonic, 0.0)
            cpu_usage_percent = 0.0
            memory_rss_bytes = 0
            memory_vms_bytes = 0
            resources: List[ProcessResourceSnapshot] = []
            system_uptime_sec = self.read_system_uptime_sec()

            for pid in sorted(process_stats):
                stats = process_stats[pid]
                memory_rss_bytes += stats.rss_bytes
                memory_vms_bytes += stats.vms_bytes

                process_cpu_usage = 0.0
                process_sample_elapsed = 0.0
                previous_sample = active_launch.cpu_samples.get(pid)
                if previous_sample is not None:
                    previous_jiffies, previous_timestamp = previous_sample
                    process_sample_elapsed = max(now - previous_timestamp, 0.0)
                    if process_sample_elapsed > 0.0:
                        cpu_delta_seconds = (
                            (stats.cpu_jiffies - previous_jiffies) / self.clock_ticks_per_second
                        )
                        process_cpu_usage = max((cpu_delta_seconds / process_sample_elapsed) * 100.0, 0.0)

                active_launch.cpu_samples[pid] = (stats.cpu_jiffies, now)
                cpu_usage_percent += process_cpu_usage

                peak_process_cpu_usage = max(
                    active_launch.process_peak_sampled_cpu_usage_percent.get(pid, 0.0),
                    process_cpu_usage,
                )
                active_launch.process_peak_sampled_cpu_usage_percent[pid] = peak_process_cpu_usage
                if process_sample_elapsed > 0.0 and peak_process_cpu_usage > 0.0 and process_cpu_usage >= 0.8 * peak_process_cpu_usage:
                    active_launch.process_time_above_80pct_peak_sampled_cpu_sec[pid] = (
                        active_launch.process_time_above_80pct_peak_sampled_cpu_sec.get(pid, 0.0) + process_sample_elapsed
                    )

                peak_process_rss = max(
                    active_launch.process_peak_memory_rss_bytes.get(pid, 0),
                    stats.rss_bytes,
                )
                active_launch.process_peak_memory_rss_bytes[pid] = peak_process_rss
                if sample_elapsed > 0.0 and peak_process_rss > 0 and stats.rss_bytes >= 0.9 * peak_process_rss:
                    active_launch.process_time_above_90pct_peak_rss_sec[pid] = (
                        active_launch.process_time_above_90pct_peak_rss_sec.get(pid, 0.0) + sample_elapsed
                    )

                peak_process_vms = max(
                    active_launch.process_peak_memory_vms_bytes.get(pid, 0),
                    stats.vms_bytes,
                )
                active_launch.process_peak_memory_vms_bytes[pid] = peak_process_vms
                if sample_elapsed > 0.0 and peak_process_vms > 0 and stats.vms_bytes >= 0.9 * peak_process_vms:
                    active_launch.process_time_above_90pct_peak_vms_sec[pid] = (
                        active_launch.process_time_above_90pct_peak_vms_sec.get(pid, 0.0) + sample_elapsed
                    )

                is_main_process = pid in live_main_pids
                process_uptime_sec = max(
                    system_uptime_sec - (stats.start_ticks / self.clock_ticks_per_second),
                    0.0,
                )
                resource_key = f'{active_launch.reference_name}-{stats.process_name}-{pid}'
                resources.append(ProcessResourceSnapshot(
                    key=resource_key,
                    reference_name=active_launch.reference_name,
                    launch_id=active_launch.launch_id,
                    state=active_launch.state,
                    resolved_launch_file_path=active_launch.launch_file_path,
                    process_name=stats.process_name,
                    pid=pid,
                    parent_pid=stats.ppid,
                    is_main_process=is_main_process,
                    launch_uptime_sec=launch_uptime_sec,
                    process_uptime_sec=process_uptime_sec,
                    cpu_usage_percent=process_cpu_usage,
                    peak_sampled_cpu_usage_percent=peak_process_cpu_usage,
                    time_above_80pct_peak_sampled_cpu_sec=active_launch.process_time_above_80pct_peak_sampled_cpu_sec.get(pid, 0.0),
                    memory_rss_bytes=stats.rss_bytes,
                    peak_memory_rss_bytes=peak_process_rss,
                    time_above_90pct_peak_rss_sec=active_launch.process_time_above_90pct_peak_rss_sec.get(pid, 0.0),
                    memory_vms_bytes=stats.vms_bytes,
                    peak_memory_vms_bytes=peak_process_vms,
                    time_above_90pct_peak_vms_sec=active_launch.process_time_above_90pct_peak_vms_sec.get(pid, 0.0),
                ))

            for stale_pid in set(active_launch.cpu_samples) - set(process_stats):
                active_launch.cpu_samples.pop(stale_pid, None)
                active_launch.process_peak_sampled_cpu_usage_percent.pop(stale_pid, None)
                active_launch.process_peak_memory_rss_bytes.pop(stale_pid, None)
                active_launch.process_peak_memory_vms_bytes.pop(stale_pid, None)
                active_launch.process_time_above_80pct_peak_sampled_cpu_sec.pop(stale_pid, None)
                active_launch.process_time_above_90pct_peak_rss_sec.pop(stale_pid, None)
                active_launch.process_time_above_90pct_peak_vms_sec.pop(stale_pid, None)

            active_launch.peak_sampled_cpu_usage_percent = max(
                active_launch.peak_sampled_cpu_usage_percent,
                cpu_usage_percent,
            )
            if sample_elapsed > 0.0 and active_launch.peak_sampled_cpu_usage_percent > 0.0 and cpu_usage_percent >= 0.8 * active_launch.peak_sampled_cpu_usage_percent:
                active_launch.time_above_80pct_peak_sampled_cpu_sec += sample_elapsed

            active_launch.peak_memory_rss_bytes = max(
                active_launch.peak_memory_rss_bytes,
                memory_rss_bytes,
            )
            if sample_elapsed > 0.0 and active_launch.peak_memory_rss_bytes > 0 and memory_rss_bytes >= 0.9 * active_launch.peak_memory_rss_bytes:
                active_launch.time_above_90pct_peak_rss_sec += sample_elapsed

            active_launch.peak_memory_vms_bytes = max(
                active_launch.peak_memory_vms_bytes,
                memory_vms_bytes,
            )
            if sample_elapsed > 0.0 and active_launch.peak_memory_vms_bytes > 0 and memory_vms_bytes >= 0.9 * active_launch.peak_memory_vms_bytes:
                active_launch.time_above_90pct_peak_vms_sec += sample_elapsed

            active_launch.last_metrics_sample_timestamp = now

            metrics = ProcessMetrics(
                main_pids=live_main_pids,
                child_pids=child_pids,
                cpu_usage_percent=max(cpu_usage_percent, 0.0),
                peak_sampled_cpu_usage_percent=active_launch.peak_sampled_cpu_usage_percent,
                time_above_80pct_peak_sampled_cpu_sec=active_launch.time_above_80pct_peak_sampled_cpu_sec,
                memory_rss_bytes=memory_rss_bytes,
                peak_memory_rss_bytes=active_launch.peak_memory_rss_bytes,
                time_above_90pct_peak_rss_sec=active_launch.time_above_90pct_peak_rss_sec,
                memory_vms_bytes=memory_vms_bytes,
                peak_memory_vms_bytes=active_launch.peak_memory_vms_bytes,
                time_above_90pct_peak_vms_sec=active_launch.time_above_90pct_peak_vms_sec,
                launch_uptime_sec=launch_uptime_sec,
                resources=resources,
            )
            active_launch.latest_metrics = metrics
            return metrics

    def collect_process_tree(self, root_pids: Set[int], root_process_names: Dict[int, str]) -> Dict[int, ProcessStats]:
        if not root_pids:
            return {}

        all_processes: Dict[int, ProcessStats] = {}
        children_by_ppid: Dict[int, List[int]] = {}

        for entry in os.listdir('/proc'):
            if not entry.isdigit():
                continue

            pid = int(entry)
            stats = self.read_process_stats(pid, root_process_names.get(pid))
            if stats is None:
                continue

            all_processes[pid] = stats
            children_by_ppid.setdefault(stats.ppid, []).append(pid)

        collected: Dict[int, ProcessStats] = {}
        pending = [pid for pid in root_pids if pid in all_processes]

        while pending:
            pid = pending.pop()
            if pid in collected:
                continue

            stats = all_processes.get(pid)
            if stats is None:
                continue

            collected[pid] = stats
            pending.extend(children_by_ppid.get(pid, []))

        return collected

    def read_process_stats(self, pid: int, fallback_process_name: Optional[str] = None) -> Optional[ProcessStats]:
        stat_path = os.path.join('/proc', str(pid), 'stat')

        try:
            with open(stat_path, 'r', encoding='utf-8') as stat_file:
                content = stat_file.read().strip()
        except OSError:
            return None

        right_paren = content.rfind(')')
        if right_paren == -1 or right_paren + 2 >= len(content):
            return None

        process_name = fallback_process_name or content[content.find('(') + 1:right_paren]
        fields = content[right_paren + 2:].split()
        if len(fields) < 22:
            return None

        try:
            ppid = int(fields[1])
            start_ticks = int(fields[19])
            utime = int(fields[11])
            stime = int(fields[12])
            vms_bytes = int(fields[20])
            rss_pages = int(fields[21])
        except ValueError:
            return None

        return ProcessStats(
            pid=pid,
            ppid=ppid,
            process_name=process_name,
            start_ticks=start_ticks,
            cpu_jiffies=utime + stime,
            rss_bytes=max(rss_pages, 0) * self.memory_page_size,
            vms_bytes=max(vms_bytes, 0),
        )

    def read_system_uptime_sec(self) -> float:
        try:
            with open('/proc/uptime', 'r', encoding='utf-8') as uptime_file:
                return float(uptime_file.read().split()[0])
        except (OSError, ValueError, IndexError):
            return 0.0