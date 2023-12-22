^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package rmf_task_sequence
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

2.1.8 (2023-12-22)
------------------
* Add support for multiple destinations to choose from. (`#101 <https://github.com/open-rmf/rmf_task/pull/101>`_)
* Contributors: Arjo Chakravarty, Grey

2.1.7 (2023-12-15)
------------------
* Fix race condition risk for task sequences (`#102 <https://github.com/open-rmf/rmf_task/pull/102>`_)

2.1.6 (2023-08-10)
------------------
* Fix battery drain crash for GoToPlace (`#96 <https://github.com/open-rmf/rmf_task/pull/96>`_)
* Contributors: Yadunund

2.1.5 (2023-06-30)
------------------
* Added ``requester`` and ``request_time`` fields to ``rmf_task::Task::Booking`` (`#81 <https://github.com/open-rmf/rmf_task/pull/81>`_)
* Contributors: Aaron Chong

2.1.4 (2023-06-05)
------------------

2.1.3 (2022-04-17)
------------------
* Fix battery consumption modeling for perform_action tasks (`#76 <https://github.com/open-rmf/rmf_task/pull/76>`_)
* Add missing buildtool_depend on cmake (`#75 <https://github.com/open-rmf/rmf_task/pull/75>`_)
* Contributors: Luca Della Vedova, Scott K Logan, Yadunund

2.1.2 (2022-11-14)
------------------

2.1.1 (2022-10-11)
------------------
* Add find_package/find_dependency for vendored project.
  The vendor package for nlohmann_json_schema_validator was previously
  exporting dependency info for that package, however that is not the
  recommended workflow for vendor packages which are ideally as
  transparent as possible.
* Contributors: Grey, Marco A. Gutiérrez, Steven! Ragnarök

2.1.0 (2022-05-19)
------------------
*  Allow GoToPlace to know about expected future destinations (`#61 <https://github.com/open-rmf/rmf_task/pull/61>`_)
* Contributors: Grey

2.0.0 (2022-02-14)
------------------
* Support flexible task definitions (`#39 <https://github.com/open-rmf/rmf_task/pull/3>`_)
  * Provide an implementation for phase-sequence based tasks
  * Arbitrary phases can be implemented downstream
  * A simple event-wrapping phase implementation is provided
  * Events can be composed into a tree-structure of sequential events
  * Downstream users can implement arbitrary events
* Contributors: Grey, Xiyu, Yadunund, Youliang
