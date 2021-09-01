## Changelog for package rmf_task

1.0.0 (2021-09-01)
------------------
* Fix id for ParkRobotFactory: [#37](https://github.com/open-rmf/rmf_task/issues/37)
* New TaskPlanner::Options, TaskPlanner::plan(), RequestFactory APIs: [#28](https://github.com/open-rmf/rmf_task/issues/28>)
* Remove dependency on rmf_dispenser_msgs: [#36](https://github.com/open-rmf/rmf_task/issues/36)
* Bug fix when battery_soc of robot is greater than recharge_soc: [#30](https://github.com/open-rmf/rmf_task/issues/30)
* Contributors: Yadunund
  
0.1.0 (2021-06-04)
------------------
* Provides `rmf_task::requests` to describe various task requests which may be used for allocation planning
* Provides `rmf_task::agv::TaskPlanner` object that can generate task allocation plans where a set of `rmf_task::requests` are optimally distributed across a set of agents
* `rmf_task::requests::ChargeBattery` requests are automatically injected into the allocation set where necessary
