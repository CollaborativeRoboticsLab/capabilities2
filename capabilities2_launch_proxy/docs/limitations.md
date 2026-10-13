# Current limitations

### Duplicate launch prevention is path-based

The same launch file cannot be active twice at the same time because active launches are keyed by canonical launch path.

### Resource reporting is current-state only

Resource reporting is designed for live observability, not historical accounting.

- CPU is based on recent sample deltas, not lifetime averages.
- Peak CPU and peak memory are retained only for the current lifetime of the active launch.
- Memory includes both current sampled values and retained peak sampled values, but not post-exit history or system-wide GPU/VRAM statistics.
- Exited processes are removed from subsequent snapshots.

### Result status depends on action lifecycle

The action remains active for the lifetime of the launched system. This is intentional for lifecycle ownership, but it means `ros2 action send_goal` behaves like a long-running session unless the goal is canceled or the launch exits.