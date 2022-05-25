## Changelog for package rmf_task_sequence

2.1.0 (2022-05-19)
------------------
*  Allow GoToPlace to know about expected future destinations [#61](https://github.com/open-rmf/rmf_task/pull/61)

2.0.0 (2022-02-14)
------------------
* Support flexible task definitions [#39](https://github.com/open-rmf/rmf_task/pull/39)
  * Provide an implementation for phase-sequence based tasks
  * Arbitrary phases can be implemented downstream
  * A simple event-wrapping phase implementation is provided
  * Events can be composed into a tree-structure of sequential events
  * Downstream users can implement arbitrary events
