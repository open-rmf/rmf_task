^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package rmf_task
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

2.5.1 (2024-06-15)
------------------

2.5.0 (2024-06-01)
------------------
* Reformat with uncrustify available in noble (`#119 <https://github.com/open-rmf/rmf_task/pull/119>`_)
* Add labels to booking (`#110 <https://github.com/open-rmf/rmf_task/pull/110>`_)
* Cancellation phase (`#107 <https://github.com/open-rmf/rmf_task/pull/107>`_)
* Contributors: Aaron Chong, Grey, Yadunund

2.4.0 (2023-12-22)
------------------

2.3.3 (2023-12-15)
------------------
* Allow charging tasks to run indefinitely (`#99 <https://github.com/open-rmf/rmf_task/pull/99>`_, `#100 <https://github.com/open-rmf/rmf_task/pull/100>`_)

2.3.2 (2023-08-10)
------------------

2.3.1 (2023-06-30)
------------------
* Added ``requester`` and ``request_time`` fields to ``rmf_task::Task::Booking`` (`#81 <https://github.com/open-rmf/rmf_task/pull/81>`_)
* Contributors: Aaron Chong

2.3.0 (2023-06-08)
------------------

2.2.0 (2023-06-06)
------------------
* Switch to rst changelogs
* Update github actions and fix style as per uncrustify 0.72 (`#74 <https://github.com/open-rmf/rmf_task/pull/74>`_)
* Fix build with apple clang (`#77 <https://github.com/open-rmf/rmf_task/pull/77>`_)
* Contributors: Esteban Martinena, Yadunund

2.1.3 (2022-04-17)
------------------
* Fixed unit test for checking automatic insertion of ChargeBattery task (`#76 <https://github.com/open-rmf/rmf_task/pull/76>`_)
* Add missing buildtool_depend on cmake (`#75 <https://github.com/open-rmf/rmf_task/pull/75>`_)
* Contributors: Luca Della Vedova, Scott K Logan, Yadunund

2.1.2 (2022-11-14)
------------------
* Removed c++fs option for clang
* Fixing need of  stdc++fs to build RPM package (`#265 <https://github.com/open-rmf/rmf/pull/265>`_)
* Contributors: Esteban Martinena

2.1.1 (2022-10-11)
------------------
* Allow GoToPlace to know about expected future destinations (`#61 <https://github.com/open-rmf/rmf_task/pull/61>`_)
* Contributors: Grey, Marco A. Gutiérrez

2.1.0 (2022-05-19)
------------------
* Fix undefined behavior in log: (`#62 <https://github.com/open-rmf/rmf_task/pull/62>`_)

2.0.0 (2022-02-14)
------------------
* Support flexible task definitions (`#39 <https://github.com/open-rmf/rmf_task/pull/39>`_)
  * Abstract interfaces are used to define tasks
  * The task interfaces can be given arbitrary implementations by downstream users

1.0.0 (2021-09-01)
------------------
* Fix id for ParkRobotFactory: (`#37 <https://github.com/open-rmf/rmf_task/pull/37>`_)
* New TaskPlanner::Options, TaskPlanner::plan(), RequestFactory APIs: (`#28 <https://github.com/open-rmf/rmf_task/pull/28>`_)
* Remove dependency on rmf_dispenser_msgs: (`#36 <https://github.com/open-rmf/rmf_task/pull/36>`_)
* Bug fix when battery_soc of robot is greater than recharge_soc: (`#30 <https://github.com/open-rmf/rmf_task/pull/30>`_)
* Contributors: Yadunund

0.1.0 (2021-06-04)
------------------
* Provides `rmf_task::requests` to describe various task requests which may be used for allocation planning
* Provides `rmf_task::agv::TaskPlanner` object that can generate task allocation plans where a set of `rmf_task::requests` are optimally distributed across a set of agents
* `rmf_task::requests::ChargeBattery` requests are automatically injected into the allocation set where necessary
