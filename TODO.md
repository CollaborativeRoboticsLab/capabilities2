# TODO list

## Problems

- [x] names need to be in package/name format everywhere
- [x] better docs
- [x] BUG: handle "'" in db queries
- [x] BUG: escape db function variables
- [x] document deferred launch-runner support and planned replacement for current ROS2 launch incompatibilities
- [x] restore launch-runner support once the replacement approach is ready
- [x] add descriptions to packages with TODO in package.xml
- [x] standardise trigger prototype
- [x] bump cmake min version to 3.16
- [ ] check thread safety for runner execution threads
- [ ] add note on threaded execution in base trigger function

## Capability Lifecycle And Launch

- [x] Share capabilities2 server and launch-proxy configuration through launch arguments
- [x] Add a proxy setting to disable child-log action feedback while retaining status feedback and independent ROS log mirroring; focused tests and affected package builds passed.
- [ ] Complete context-aware provider selection for generated plans. Base providers remain advertised for their base interface and can still be selected despite a conflicting running dependency; multiple direct providers may also be available. Direct semantic providers now take precedence for their own wrapper over inherited generic providers, without affecting other wrappers; descriptor regressions pass.
- [ ] Complete synchronization and thread-safety review for compatible runner reuse, bond claims, provider/configuration conflicts, and concurrent acquisition; reuse and conflict checks are implemented. Shared action and service runners still hold per-execution state in members, and action cancellation while a goal is awaiting acceptance needs a bounded lifecycle.
- [ ] Validate dependency ownership and exact rollback across remaining failure and bond-break cases. Per-parent dependency bonds and the `release_dependencies` helper are implemented.
- [x] Return explicit success or failure for use, connect, and trigger requests and rebuild affected service consumers.
- [ ] Extend lifecycle tests for shared runners and dependencies, failed startup rollback, conflicting providers, repeated claims, and bond-break cleanup. Focused server tests now include repeated bond-break cleanup while another parent retains the shared dependency; client tests cover separate bond claims.
- [x] Investigate why stopping the earlier launch coincided with SLAM receiving SIGINT during plan handoff: the generated plan selected a waypoint provider with a conflicting Nav2 dependency. A later run using the TurtleBot provider retained Nav2 and SLAM through handoff and completed navigation.
- [x] Avoid cancelling already-completed action goals during runner cleanup. Action-server regressions cover completed, active, and rejected goals and stop during result processing; a terminal cancellation response is handled as a normal race outcome.

## Features

- [x] try using ros package to find exports automatically
- [x] improve the event system
- [ ] implement provider definition handling in runner
- [ ] move to established db handler lib
- [ ] better bt runner impl
- [ ] db traits: identifiable, modifiable, soft_deleteable, header, remappable, predicateable

## Refactoring

- [x] remove fan out project work
- [x] merge various system runners into base runner package
- [x] custom logger needs to be removed
- [x] events should be incorporated as core function
- [x] server, runner base, api have event
- [ ] increment package versions
- [ ] interfaces and providers for cap caps
