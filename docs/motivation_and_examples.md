## Motivation

The capabilities package was originally implemented using Python. This package is a reimplementation of the capabilities package using CPP. The CPP implementation is more efficient. Secondly, this package extends the capabilities package features.

The main reasons for this are:

1. To allow the capabilities service to provide details of the robot to other robots, a UI or App, or a supervisory AI with a conversational style.
1. To allow packages to register their capabilities at runtime with the capabilities service using a service API or a spawner. Or to modify the capabilities of a robot at runtime.
1. To allow capabilities to be used like actions and not just started and stopped by the capabilities service as in the original capabilities package launch files method.
1. To add state to the capabilities service using a database. This allows the capabilities service or the robot to be restarted without losing state, and hot reloading, and model extension including more complex ontological relationships between capabilities.
1. To allow the capabilities service to be used as a library (as in a ROS2 `component`).

### Example use cases

| Use case | Description |
| --- | --- |
| Generative AI (predominantly language models) Integration | A generic GenAI could derive a plan from the capabilities of a robot, to perform more general multi-step tasks. This could be achieved by using the capabilities as a knowledge base. This may solve problems with GenAI integration, where the GenAI has to be trained on a specific robot, or there is a significant delay in the GenAI communicating with the robot - since the GenAI does not actually execute the task, it is used to suggest a plan and supervise the robot. |
| Inter-Robot Communications | A robot encounters another robot of unknown origin and asks it what it can do. The capabilities could be communicated using M2M communication or speech. This could allow robots from different manufacturers to work together. |
| Enable emergent skills | A robot is asked to do a task it has never done before. This could be achieved by combining capabilities in new ways. or a robot is asked to do a task it has done before but with a different context. This could be achieved by changing the parameters of capabilities. |
| Interoperable Robots | A universal remote control for robots could be created using the capabilities as a standardised interface. This might be useful for robot subsystem integrators. This could also be used to create a robot app store, or standardised sensor and actuator interfaces. |
