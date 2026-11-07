# Current limitations

### Duplicate launch prevention is path-based

The same launch file cannot be active twice at the same time because active launches are keyed by canonical launch path.

### Shutdown confirmation

A clean supervisor exit confirms that ROS launch has finished its managed actions. Processes that detach from ROS launch are not guaranteed to exit with it. An unclean supervisor exit quarantines that launch path until the proxy restarts; inspect and clean up any remaining processes before restarting, because the quarantine is not persistent.

If the proxy reports a shutdown timeout, the capability runner retains its dependency claims and rejects reuse until shutdown can be confirmed. Automatic reconciliation after an unconfirmed action result is not yet available. The runner waits up to 60 seconds for the action result, so configuring the proxy stop timeout above that can cause the runner to time out first.

### Resource reporting is current-state only

Resource reporting is designed for live observability, not historical accounting.

- CPU is based on recent sample deltas, not lifetime averages.
- Peak CPU and peak memory are retained only for the current lifetime of the active launch.
- Memory includes both current sampled values and retained peak sampled values, but not post-exit history or system-wide GPU/VRAM statistics.
- Exited processes are removed from subsequent snapshots.

### Result status depends on action lifecycle

The action remains active for the lifetime of the launched system. This is intentional for lifecycle ownership, but it means `ros2 action send_goal` behaves like a long-running session unless the goal is canceled or the launch exits.