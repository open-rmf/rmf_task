## Changelog for package rmf_task_sequence

2.1.x (2022-xx-xx)
------------------
* Fix battery consumption modeling for perform_action tasks [#76](https://github.com/open-rmf/rmf_task/pull/76)
* Contributors: Luca Della Vedova, Yadunund

2.1.1 (2022-10-11)
------------------
* Add find_package/find_dependency for vendored project.
  The vendor package for nlohmann_json_schema_validator was previously
  exporting dependency info for that package, however that is not the
  recommended workflow for vendor packages which are ideally as
  transparent as possible.
* Allow GoToPlace to know about expected future destinations (`#61 <https://github.com/open-rmf/rmf_task/issues/61>`_)
* Contributors: Grey, Marco A. Gutiérrez, Steven! Ragnarök

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
