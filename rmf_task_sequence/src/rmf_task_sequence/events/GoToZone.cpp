#include <rmf_task_sequence/events/GoToZone.hpp>
#include <rmf_task_sequence/events/GoToPlace.hpp>

#include "utils.hpp"

namespace rmf_task_sequence {
namespace events {

namespace {
//==============================================================================
// One GoToPlace model per zone vertex, costed against the nearest.
class ZoneModel : public Activity::Model
{
public:
  ZoneModel(
    std::vector<Activity::ConstModelPtr> candidates)
  : _candidates(std::move(candidates)),
    _nearest(_candidates.front())
  {
    for (const auto& candidate : _candidates)
    {
      if (candidate->invariant_duration() < _nearest->invariant_duration())
        _nearest = candidate;
    }
  }

  std::optional<rmf_task::Estimate> estimate_finish(
    rmf_task::State initial_state,
    rmf_traffic::Time earliest_arrival_time,
    const rmf_task::Constraints& constraints,
    const rmf_task::TravelEstimator& travel_estimator) const override
  {
    std::optional<rmf_task::Estimate> best;
    for (const auto& candidate : _candidates)
    {
      auto estimate = candidate->estimate_finish(
        initial_state, earliest_arrival_time, constraints, travel_estimator);
      if (!estimate.has_value())
        continue;

      if (!best.has_value()
        || *estimate->finish_state().time() < *best->finish_state().time())
      {
        best = std::move(estimate);
      }
    }

    return best;
  }

  rmf_traffic::Duration invariant_duration() const override
  {
    return _nearest->invariant_duration();
  }

  rmf_task::State invariant_finish_state() const override
  {
    return _nearest->invariant_finish_state();
  }

private:
  std::vector<Activity::ConstModelPtr> _candidates;
  Activity::ConstModelPtr _nearest;
};
} // anonymous namespace

//==============================================================================
class GoToZone::Description::Implementation
{
public:
  std::string zone_name;
  std::optional<Modifiers> modifiers;
};

//==============================================================================
auto GoToZone::Description::make(
  std::string zone_name,
  std::optional<Modifiers> modifiers) -> DescriptionPtr
{
  auto desc = std::shared_ptr<Description>(new Description);
  desc->_pimpl = rmf_utils::make_impl<Implementation>(
    Implementation{std::move(zone_name), std::move(modifiers)});

  return desc;
}

//==============================================================================
Activity::ConstModelPtr GoToZone::Description::make_model(
  State invariant_initial_state,
  const Parameters& parameters) const
{
  const auto& graph = parameters.planner()->get_configuration().graph();
  const auto zone_props = graph.find_known_zone(_pimpl->zone_name);
  if (!zone_props)
    return nullptr;

  std::vector<Activity::ConstModelPtr> candidates;
  for (const auto& iv : zone_props->internal_vertices())
  {
    const auto* wp = graph.find_waypoint(iv.name());
    if (!wp)
      continue;

    auto model = GoToPlace::Description::make(wp->index())
      ->make_model(invariant_initial_state, parameters);
    if (model)
      candidates.push_back(std::move(model));
  }

  if (candidates.empty())
    return nullptr;

  return std::make_shared<ZoneModel>(std::move(candidates));
}

//==============================================================================
Header GoToZone::Description::generate_header(
  const State& initial_state,
  const Parameters& parameters) const
{
  const std::string& fail_header = "[GoToZone::Description::generate_header]";
  const auto& graph = parameters.planner()->get_configuration().graph();
  const auto start_wp_opt = initial_state.waypoint();
  if (!start_wp_opt)
    utils::fail(fail_header, "Initial state is missing a waypoint");

  const auto start_name =
    rmf_task::standard_waypoint_name(graph, *start_wp_opt);

  const auto model = make_model(initial_state, parameters);
  const auto duration = model ? model->invariant_duration() :
    rmf_traffic::Duration(0);

  return Header(
    "Go to zone " + _pimpl->zone_name,
    "Moving robot from " + start_name + " to zone " + _pimpl->zone_name,
    duration);
}

//==============================================================================
const std::string& GoToZone::Description::zone_name() const
{
  return _pimpl->zone_name;
}

//==============================================================================
const std::optional<GoToZone::Description::Modifiers>&
GoToZone::Description::modifiers() const
{
  return _pimpl->modifiers;
}

//==============================================================================
GoToZone::Description::Description()
{
  // Do nothing
}

//==============================================================================
class GoToZone::Description::Modifiers::Implementation
{
public:
  std::string group_hint;
  std::optional<double> orientation_hint;
  std::vector<std::string> preferred_waypoints;
};

//==============================================================================
GoToZone::Description::Modifiers::Modifiers()
: _pimpl(rmf_utils::make_impl<Implementation>())
{
  // Do nothing
}

//==============================================================================
const std::string& GoToZone::Description::Modifiers::group_hint() const
{
  return _pimpl->group_hint;
}

//==============================================================================
auto GoToZone::Description::Modifiers::set_group_hint(std::string hint)
-> Modifiers&
{
  _pimpl->group_hint = std::move(hint);
  return *this;
}

//==============================================================================
std::optional<double>
GoToZone::Description::Modifiers::orientation_hint() const
{
  return _pimpl->orientation_hint;
}

//==============================================================================
auto GoToZone::Description::Modifiers::set_orientation_hint(
  std::optional<double> hint) -> Modifiers&
{
  _pimpl->orientation_hint = hint;
  return *this;
}

//==============================================================================
const std::vector<std::string>&
GoToZone::Description::Modifiers::preferred_waypoints() const
{
  return _pimpl->preferred_waypoints;
}

//==============================================================================
auto GoToZone::Description::Modifiers::set_preferred_waypoints(
  std::vector<std::string> waypoints) -> Modifiers&
{
  _pimpl->preferred_waypoints = std::move(waypoints);
  return *this;
}

} // namespace events
} // namespace rmf_task_sequence