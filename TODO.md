# TODO list

## Problems

- [x] names need to be in package/name format everywhere
- [x] better docs
- [x] BUG: handle "'" in db queries
- [x] BUG: escape db function variables
- [ ] close or change communication to launch proxy so that it can't be accessed from ros network
- [ ] BUG: fix issue with connecting to services and actions started using launch proxy
- [ ] Launch proxy does not work, maybe remove and note as future work, or suggest alternatives
- [x] add descriptions to packages with TODO in package.xml
- [ ] standardise trigger prototype
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
- [ ] custom logger needs to be removed
- [ ] events should be incorporated as core function
- [ ] server, runner base, api have event
- [ ] increment package versions
- [ ] interfaces and providers for cap caps
