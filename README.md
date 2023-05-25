# rmf_task

![](https://github.com/open-rmf/rmf_task/workflows/build/badge.svg)
[![codecov](https://codecov.io/gh/open-rmf/rmf_task/branch/main/graph/badge.svg)](https://codecov.io/gh/open-rmf/rmf_task)

## Overview
The rmf_task repository contains two packages, `rmf_task` and `rmf_task_sequence`.

### rmf_task
Provides APIs and base classes for defining and managing Tasks in RMF.
* `rmf_task::Task` is a pure abstract interface for an executable Task. What this task represents, how it can be modelled and finally executed are left for the user to implement. To aid such implementations, the following classes are available:
  * [Active](https://docs.ros.org/en/rolling/p/rmf_task/generated/classrmf__task_1_1Task_1_1Active.html): A pure abstract interface to manage the runtime execution of a Task.
  * [Booking](https://docs.ros.org/en/rolling/p/rmf_task/generated/classrmf__task_1_1Task_1_1Booking.html): Stores basic information about the task.
  * [Description](https://docs.ros.org/en/rolling/p/rmf_task/generated/classrmf__task_1_1Task_1_1Description.html): An abstract interface to define the specifics of a task which when implemented help differentiate different tasks from one another.
  * [Model](https://docs.ros.org/en/rolling/p/rmf_task/generated/classrmf__task_1_1Task_1_1Model.html): A pure abstract interface to compute the [Estimate](https://docs.ros.org/en/rolling/p/rmf_task/generated/classrmf__task_1_1Estimate.html#) state of the robot after the task is completed along with an `invariant_duration` which is the time component of the task that does not change regardless of where the robot starts.
  * [Tag](https://docs.ros.org/en/rolling/p/rmf_task/generated/classrmf__task_1_1Task_1_1Tag.html): Static information about the task.
* [rmf_task::TaskPlanner](https://docs.ros.org/en/rolling/p/rmf_task/generated/classrmf__task_1_1TaskPlanner.html): API which solves the problem of optimal allocation of tasks among available robots. For a given collection of tasks and robots belonging to a fleet (ie, they share physical and kinematic traits), the planner determines the best ordering of tasks across robots such that tasks are completed in the shortest durations given their requested start times. It does this while also accounting for resource constraints of the robots such as battery level and automatically injects recharging tasks in the robot’s task itinerary when needed. Here tasks are represented by [Requests](https://docs.ros.org/en/rolling/p/rmf_task/generated/classrmf__task_1_1Request.html) which are made up of `Booking` and `Description` elements respectively.

### rmf_task_sequence
Provides an out-of-the-box implementation of `rmf_task`.
* [rmf_task_sequence::Task](https://docs.ros.org/en/rolling/p/rmf_task_sequence/generated/classrmf__task__sequence_1_1Task.html), which assumes a task to be composed of a sequence of [phases](https://docs.ros.org/en/rolling/p/rmf_task_sequence/generated/classrmf__task__sequence_1_1Phase.html) that need to be executed in-order for the task to be completed. A phase inturn may be composed of a set of [Events](https://docs.ros.org/en/rolling/p/rmf_task_sequence/generated/classrmf__task__sequence_1_1Event.html).
* The following events that are presently available (contributions are welcome):
  * [Bundle](https://docs.ros.org/en/rolling/p/rmf_task_sequence/generated/classrmf__task__sequence_1_1events_1_1Bundle.html#exhale-class-classrmf-task-sequence-1-1events-1-1bundle)
  * [DropOff](https://docs.ros.org/en/rolling/p/rmf_task_sequence/generated/classrmf__task__sequence_1_1events_1_1DropOff.html#exhale-class-classrmf-task-sequence-1-1events-1-1dropoff)
  * [GoToPlace](https://docs.ros.org/en/rolling/p/rmf_task_sequence/generated/classrmf__task__sequence_1_1events_1_1GoToPlace.html#exhale-class-classrmf-task-sequence-1-1events-1-1gotoplace)
  * [PerformAction](https://docs.ros.org/en/rolling/p/rmf_task_sequence/generated/classrmf__task__sequence_1_1events_1_1PerformAction.html#exhale-class-classrmf-task-sequence-1-1events-1-1performaction)
  * [PickUp](https://docs.ros.org/en/rolling/p/rmf_task_sequence/generated/classrmf__task__sequence_1_1events_1_1PickUp.html#exhale-class-classrmf-task-sequence-1-1events-1-1pickup)
  * [Placeholder](https://docs.ros.org/en/rolling/p/rmf_task_sequence/generated/classrmf__task__sequence_1_1events_1_1Placeholder.html#exhale-class-classrmf-task-sequence-1-1events-1-1placeholder)
  * [WaitFor](https://docs.ros.org/en/rolling/p/rmf_task_sequence/generated/classrmf__task__sequence_1_1events_1_1WaitFor.html#exhale-class-classrmf-task-sequence-1-1events-1-1waitfor)
* See [Usage](#usage) for information on how composed tasks are integrated with the rest of Open-RMF.

### Full API documentation:
* [rmf_task](https://docs.ros.org/en/rolling/p/rmf_task)
* [rmf_task_sequence](https://docs.ros.org/en/rolling/p/rmf_task_sequence)

## Usage
* `rmf_task_sequence` contains implementations of the models for the out of the box events and phases. The implementations of how to command the actual robot to perform these events, ie, the Active components, are defined in [rmf_fleet_adapter](https://github.com/open-rmf/rmf_ros2/tree/main/rmf_fleet_adapter/src/rmf_fleet_adapter/events)
These flexible task definitions are conveyed via json payloads whose schemas are defined in [rmf_api_msgs](https://github.com/open-rmf/rmf_api_msgs/blob/main/rmf_api_msgs/schemas/task_request.json). The schemas for the “description” field are provided in rmf_fleet_adapter [here](https://github.com/open-rmf/rmf_ros2/tree/main/rmf_fleet_adapter/schemas). The fleet adapter currently only supports tasks that are defined as a sequence of phases as defined in rmf_task_sequence. It can very easily be extended to support other user implemented task definitions.
* Users can specify how the robot should react when a task is canceled or interrupted at any given phase. Each phase can have its own unique response and can be described [here](https://github.com/open-rmf/rmf_ros2/blob/8440488d5583edc5a5b7226326aa2a8d41dad975/rmf_fleet_adapter/schemas/task_description__compose.json#L23-L27)
See [Examples](#examples) below on how to submit tasks to RMF. These tasks can either be directly assigned to a specific robot or have RMF determine the optimal allocation based on its multi-fleet task allocation framework. This is described in greater detail [here](https://osrf.github.io/ros2multirobotbook/task.html)

> Note: When stringing together activities (same as events) in a compose task description (same as flexible task), users need to populate a `category` and `description` for each [activity](https://github.com/open-rmf/rmf_ros2/blob/8440488d5583edc5a5b7226326aa2a8d41dad975/rmf_fleet_adapter/schemas/task_description__compose.json#L40-L45). The schemas for various `descriptions` are provided [here](https://github.com/open-rmf/rmf_ros2/tree/main/rmf_fleet_adapter/schemas). See `event_description__X` where X is the name of the event. Then, care must be taken to ensure the `category` is populated with this same `X`.


## Examples
Examples of constructing and submitting custom tasks is provided in the rmf_demos repository https://github.com/open-rmf/rmf_demos/tree/main/rmf_demos_tasks

## ROSCon Presentations with more information
* ROSCon2022: How custom tasks are defined, assigned, and executed in Open-RMF
  * [Slides](http://download.ros.org/downloads/roscon/2022/How%20custom%20tasks%20are%20defined,%20assigned,%20and%20executed%20in%20Open-RMF.pdf)
  * [Video](https://vimeo.com/showcase/9954564/video/767157210)
