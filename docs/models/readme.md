# Capabilities2 Models

This directory contains the data models used by the Capabilities2 server.

## Models

| Model | Description |
|-------|-------------|
| `provider` | Represents a provider of an interface |
| `interface` | Represents a capability interface |
| `semantic_interface` | Represents a semantic capability interface |

### V1.2 new models (TODO)

| Model | Description |
|-------|-------------|
| `interface_resources` | Represents a remmapable resource such as topic, service, action |
| `provider_remmapings` | Represents remappings for a provider such as topic, service, action remappings |
| `semantic_interface_remmapings` | Represents remappings for a semantic interface such as topic, service, action remappings |

### V2 new models (TODO)

| Model | Description |
|-------|-------------|
| `predicates` | Represents predicates between capabilities |
| `assurances` | Represents assurances for capabilities such as success rate, run time, resource use |

### V3 new models (TODO)(Proposal)

| Model | Description |
|-------|-------------|
| `running` | Represents active runners that are executing capabilities |
| `connections` | Represents connections between active runners |
| `definition` | model text defining function of capability |
| `parameters` | parameters of a capability |

## Traits

The models can implement various traits to provide additional functionality. The available traits are:

| Trait | Description |
|-------|-------------|
| `identifiable` | Allows the model to have a unique identifier |
| `modifiable` | Adds created and modified timestamps |
| `soft_deleteable` | Allows the model to be soft deleted |
| `header` | Adds a header to the model including name, description, and version etc. The header uses id, created, and deleted timestamps |
| `remappable` | Allows the model to have remappings |

### V2 new traits (TODO)

| Trait | Description |
|-------|-------------|
| `predicateable` | Allows the model to use subject-predicate relationships |
| `assurable` | Adds tracking metrics for capability performance |

## Relationships

The models can have relationships with each other. The available relationships are:

| Relationship | Description |
|--------------|-------------|
| `implements` | A provider implements an interface |
| `depends_on` | A provider depends on another provider |
| `redefines` | A semantic interface redefines an interface |

### V2 new relationships (TODO)

| Relationship | Description |
|--------------|-------------|
| `predicate` | A relationship between capabilities defined by a predicate |

## V2 milestones

Each milestone will have a testing check gate before moving to the next milestone. The milestones are:

0. fix sql injection vulnerabilities by changing '+' string concatenation to parameterized queries
    0.1 dont throw away yaml spec not specced yet, (forward compatibility)
    0.2
1. implement traits in table schema
    1.1 redesign ORM classes to work better with traits and relationships
2. fix current 3 model relations including inheritance relations and move join relations to separate tables rather than sorted in db api handler code
    2.1 add new models from model.erd
3. surface parameterised queries to enable selecting just values from specific columns
