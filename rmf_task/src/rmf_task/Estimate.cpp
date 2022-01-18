/*
 * Copyright (C) 2020 Open Source Robotics Foundation
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *
*/

#include <unordered_map>
#include <mutex>

#include <rmf_task/Estimate.hpp>

namespace rmf_task {

//==============================================================================
class Estimate::Implementation
{
public:

  Implementation(State finish_state, rmf_traffic::Time wait_until)
  : _finish_state(std::move(finish_state)),
    _wait_until(std::move(wait_until))
  {}

  State _finish_state;
  rmf_traffic::Time _wait_until;
};

//==============================================================================
Estimate::Estimate(State finish_state, rmf_traffic::Time wait_until)
: _pimpl(rmf_utils::make_impl<Implementation>(
      std::move(finish_state), std::move(wait_until)))
{
}

//==============================================================================
State Estimate::finish_state() const
{
  return _pimpl->_finish_state;
}

//==============================================================================
Estimate& Estimate::finish_state(State new_finish_state)
{
  _pimpl->_finish_state = std::move(new_finish_state);
  return *this;
}

//==============================================================================
rmf_traffic::Time Estimate::wait_until() const
{
  return _pimpl->_wait_until;
}

//==============================================================================
Estimate& Estimate::wait_until(rmf_traffic::Time new_wait_until)
{
  _pimpl->_wait_until = std::move(new_wait_until);
  return *this;
}

//==============================================================================
class TravelEstimator::Result::Implementation
{
public:

  static Result make(
    rmf_traffic::Duration duration_,
    double change_in_state_of_charge_)
  {
    Result output;
    output._pimpl = rmf_utils::make_impl<Implementation>(
      Implementation{duration_, change_in_state_of_charge_});

    return output;
  }

  rmf_traffic::Duration duration;
  double change_in_charge;
};

//==============================================================================
class TravelEstimator::Implementation
{
public:

  Implementation(const Parameters& parameters)
  : planner(parameters.planner()),
    motion_sink(parameters.motion_sink()),
    ambient_sink(parameters.ambient_sink()),
    cache(make_cache(planner))
  {
    // Do nothing
  }

  std::optional<Result> estimate(
    const rmf_traffic::agv::Plan::Start& start,
    const rmf_traffic::agv::Plan::Goal& goal) const
  {
    std::pair<Cache::iterator, bool> insertion;
    {
      std::unique_lock<std::mutex> lock(mutex, std::defer_lock);
      while (!lock.try_lock()) {}

      Key wps{start.waypoint(), goal.waypoint()};
      insertion = cache.insert(std::make_pair(wps, Value()));
    }

    if (!insertion.second)
      return insertion.first->second;

    auto result = calculate_result(start, goal);
    {
      std::unique_lock<std::mutex> lock(mutex, std::defer_lock);
      while (!lock.try_lock()) {}
      insertion.first->second = result;
    }

    return result;
  }

  std::optional<Result> calculate_result(
    const rmf_traffic::agv::Plan::Start& start,
    const rmf_traffic::agv::Plan::Goal& goal) const
  {
    const auto plan = planner->plan(start, goal);
    if (!plan.success())
      return std::nullopt;

    // We assume we can always compute a plan
    const auto itinerary_start_time = start.time();
    double battery_drain = 0.0;
    for (const auto& itinerary : plan->get_itinerary())
    {
      const auto& trajectory = itinerary.trajectory();
      const auto& finish_time = *trajectory.finish_time();
      const rmf_traffic::Duration itinerary_duration =
        finish_time - itinerary_start_time;

      // Compute battery drain
      const auto dSOC_motion =
        motion_sink->compute_change_in_charge(trajectory);

      const auto dSOC_device = ambient_sink->compute_change_in_charge(
        rmf_traffic::time::to_seconds(itinerary_duration));

      battery_drain += dSOC_motion + dSOC_device;
    }

    auto duration = rmf_traffic::Duration(0);
    if (!plan->get_itinerary().empty())
    {
      duration =
        plan->get_itinerary().back().trajectory().back().time()
        - itinerary_start_time;
    }

    return Result::Implementation::make(duration, battery_drain);
  }

private:
  std::shared_ptr<const rmf_traffic::agv::Planner> planner;
  rmf_battery::ConstMotionPowerSinkPtr motion_sink;
  rmf_battery::ConstDevicePowerSinkPtr ambient_sink;

  struct PairHash
  {
    PairHash(std::size_t N)
    {
      _shift = std::ceil(std::log2(N));
    }

    size_t operator()(const std::pair<size_t, size_t>& p) const
    {
      return p.first + (p.second << _shift);
    }

    std::size_t _shift;
  };

  using Key = std::pair<size_t, size_t>;
  using Value = std::optional<Result>;
  using Cache = std::unordered_map<Key, std::optional<Result>, PairHash>;
  mutable Cache cache;

  static Cache make_cache(
    const std::shared_ptr<const rmf_traffic::agv::Planner>& planner)
  {
    const auto N = planner->get_configuration().graph().num_waypoints();
    return Cache(N, PairHash(N));
  }

  mutable std::mutex mutex;
};

//==============================================================================
TravelEstimator::TravelEstimator(const Parameters& parameters)
: _pimpl(rmf_utils::make_unique_impl<Implementation>(parameters))
{
  // Do nothing
}

//==============================================================================
rmf_traffic::Duration TravelEstimator::Result::duration() const
{
  return _pimpl->duration;
}

//==============================================================================
double TravelEstimator::Result::change_in_charge() const
{
  return _pimpl->change_in_charge;
}

//==============================================================================
TravelEstimator::Result::Result()
{
  // Do nothing
}

//==============================================================================
auto TravelEstimator::estimate(
  const rmf_traffic::agv::Plan::Start& start,
  const rmf_traffic::agv::Plan::Goal& goal) const -> std::optional<Result>
{
  return _pimpl->estimate(start, goal);
}

} // namespace rmf_task
