# Changelog

## 0.2.0 (2025-09-16)

- add new service types
- upgrade event subsystem
- implement basic db traits
- add system and capability runner plugins
- threaded runner execution
- runners can be chained through trigger
- triggering between runners handled by dynamic link list style structure
- event system upgraded and refactored to be more generic and easier to use

## 0.1.2 (2024-09-20)

- working on bt runner plugins
- identifiable trait on models: can be used to define a unique identifier for a model which could allow versions of interfaces to be used
- move trigger cap to first class concept in cap API
- change trigger logic in runner cache
- guard and throw in trigger logic when cap not found
- fix cap entity versioning to be a string in the database
- doco on cap entities with examples

## 0.1.1 (2024-09-19)

- increment version of cap entities to 1.1
- add soft delete trait
- add database traits modifyable, defineable, predicateable
- defineable trait: use a behaviour language to define a provider
- predicateable trait: use a ontological triple to define relations between cap entities
