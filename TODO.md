# TODO list

## Problems

- [x] names need to be in package/name format everywhere
- [x] better docs
- [x] BUG: handle "'" in db queries
- [x] BUG: escape db function variables
- [x] document deferred launch-runner support and planned replacement for current ROS2 launch incompatibilities
- [ ] restore launch-runner support once the replacement approach is ready
- [x] add descriptions to packages with TODO in package.xml
- [x] standardise trigger prototype
- [x] bump cmake min version to 3.16
- [ ] check thread safety for runner execution threads
- [ ] add note on threaded execution in base trigger function

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
