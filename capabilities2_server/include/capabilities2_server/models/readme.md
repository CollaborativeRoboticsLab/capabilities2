# Capabilities2 Models

This directory contains the data models used by the Capabilities2 server.

## Models

| Model | Description |
|-------|-------------|
| `provider` | Represents a provider of an interface |
| `interface` | Represents a capability interface |
| `semantic_interface` | Represents a semantic capability interface |

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
| `
