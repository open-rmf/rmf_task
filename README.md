# rmf_task

![](https://github.com/open-rmf/rmf_task/workflows/build/badge.svg)
[![codecov](https://codecov.io/gh/open-rmf/rmf_task/branch/main/graph/badge.svg)](https://codecov.io/gh/open-rmf/rmf_task)

The rmf_task repository contains two packages:
* `rmf_task`
  * Provides APIs and base classes for defining and managing Tasks in RMF. A `Task` is defined as an object that generates phases which are a meaningful sequence of steps that results in a desirable outcome. Along with a description, each task should contain a component that allows us to model the state of the robot after completing the task given its initial state and also a component that will command the actual robot to perform the task.
  * Provides `TaskPlanner` API which solves the problem of optimal task allocation. For a given set of tasks and a set of robots belonging to the same fleet (ie, they share physical and kinematic traits), the planner can ascertain which tasks should be performed by which robots and in what order. It does this while also accounting for resource constraints of the robots such as battery level and automatically injects recharging tasks in the robot’s task itinerary when needed.
* `rmf_task_sequence`
  * An out of the box implementation of rmf_task where a `Task` object is defined by a sequence of phases. The phases that such tasks generate will thus match the sequence of phases used to define them.
  * The phases defined in `rmf_task_sequence` are in turn a collection of `Events` which also have components to model the end state and command the robot during the event. Presently the events that are supported are defined [here](https://github.com/open-rmf/rmf_task/tree/main/rmf_task_sequence/include/rmf_task_sequence/events). Users can construct arbitrary definitions of tasks by stringing together a sequence of such phases/events. RMF is then capable of planning and executing such tasks.


## More info

* `rmf_task` and `rmf_task_sequence` are pure C++ libraries without any ROS 2 dependencies.
* rmf_task_sequence contains implementations of the models for the out of the box events and phases. The implementations of how to command the actual robot to perform these events, ie, the Active components, are defined in [rmf_fleet_adapter](https://github.com/open-rmf/rmf_ros2/tree/main/rmf_fleet_adapter/src/rmf_fleet_adapter/events)
These flexible task definitions are conveyed via json payloads whose schemas are defined in [rmf_api_msgs](https://github.com/open-rmf/rmf_api_msgs/blob/main/rmf_api_msgs/schemas/task_request.json). The schemas for the “description” field are provided in rmf_fleet_adapter [here](https://github.com/open-rmf/rmf_ros2/tree/main/rmf_fleet_adapter/schemas). The fleet adapter currently only supports tasks that are defined as a sequence of phases as defined in rmf_task_sequence. It can very easily be extended to support other user implemented task definitions.
* Users can specify how the robot should react when a task is canceled or interrupted at any given phase. Each phase can have its own unique response and can be described [here](https://github.com/open-rmf/rmf_ros2/blob/8440488d5583edc5a5b7226326aa2a8d41dad975/rmf_fleet_adapter/schemas/task_description__compose.json#L23-L27)
See examples below on how to submit tasks to RMF. These tasks can either be directly assigned to a specific robot or have RMF determine the optimal allocation based on its multi-fleet task allocation framework. This is described in greater detail [here](https://osrf.github.io/ros2multirobotbook/task.html)

> Note: When stringing together activities (same as events) in a compose task description (same as flexible task), users need to populate a `category` and `description` for each [activity](https://github.com/open-rmf/rmf_ros2/blob/8440488d5583edc5a5b7226326aa2a8d41dad975/rmf_fleet_adapter/schemas/task_description__compose.json#L40-L45). The schemas for various `descriptions` are provided [here](https://github.com/open-rmf/rmf_ros2/tree/main/rmf_fleet_adapter/schemas). See `event_description__X` where X is the name of the event. Then, care must be taken to ensure the `category` is populated with this same `X`.


## Examples
Examples of constructing and submitting custom tasks is provided in the rmf_demos repository https://github.com/open-rmf/rmf_demos/tree/main/rmf_demos_tasks
