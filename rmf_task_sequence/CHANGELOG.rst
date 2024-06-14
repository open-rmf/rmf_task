^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package rmf_task_sequence
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

2.5.1 (2024-06-15)
------------------

2.5.0 (2024-06-01)
------------------
* Reformat with uncrustify available in noble (`#119 <https://github.com/open-rmf/rmf_task/pull/119>`_)
* Protect phase switch from race conditions (`#111 <https://github.com/open-rmf/rmf_task/pull/111>`_)
* Cancellation phase (`#107 <https://github.com/open-rmf/rmf_task/pull/107>`_)
* Do not segfault when models cannot be generated (`#109 <https://github.com/open-rmf/rmf_task/pull/109>`_)
* Return nullptr is goal set is empty (`#108 <https://github.com/open-rmf/rmf_task/pull/108>`_)
* Contributors: Grey, Yadunund

2.4.0 (2023-12-22)
------------------
* Add support for multiple destinations to choose from. (`#101 <https://github.com/open-rmf/rmf_task/pull/101>`_)
* Contributors: Arjo Chakravartyi, Grey

2.3.3 (2023-12-15)
------------------
* Fix edge case for task sequences (`#102 <https://github.com/open-rmf/rmf_task/pull/102>`_)

2.3.2 (2023-08-10)
------------------
* Fix battery drain crash for GoToPlace (`#94 <https://github.com/open-rmf/rmf_task/pull/94>`_)
* Contributors: Yadunund

2.3.1 (2023-06-30)
------------------
* Added ``rmf_task_sequence`` to workflows and fixed codestyle (`#91 <https://github.com/open-rmf/rmf_task/pull/91>`_)
* Added ``requester`` and ``request_time`` fields to ``rmf_task::Task::Booking`` (`#81 <https://github.com/open-rmf/rmf_task/pull/81>`_)
* Contributors: Aaron Chong

2.3.0 (2023-06-08)
------------------

2.2.0 (2023-06-06)
------------------
* Switch to rst changelogs
* Contributors: Yadunund

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
* Allow GoToPlace to know about expected future destinations (`#61 <https://github.com/open-rmf/rmf_task/pull/61>`_)
* Contributors: Grey, Marco A. Gutiérrez, Steven! Ragnarök

2.1.0 (2022-05-19)
------------------
*  Allow GoToPlace to know about expected future destinations (`#61 <https://github.com/open-rmf/rmf_task/pull/61>`_)

2.0.0 (2022-02-14)
------------------
* Support flexible task definitions (`#39 <https://github.com/open-rmf/rmf_task/pull/39>`_)
  * Provide an implementation for phase-sequence based tasks
  * Arbitrary phases can be implemented downstream
  * A simple event-wrapping phase implementation is provided
  * Events can be composed into a tree-structure of sequential events
  * Downstream users can implement arbitrary events
