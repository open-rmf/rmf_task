#include <rmf_task_sequence/events/GoToZone.hpp>
#include <rmf_task_sequence/events/GoToPlace.hpp>

#include "utils.hpp"

namespace rmf_task_sequence {
namespace events {

namespace {
// Wraps a GoToPlace Activity::Model and rejects robots whose current
// waypoint is inside the target zone. Prevents robots already in the
// zone from winning bids for zone tasks targeting the same zone.
class ZoneGuardModel : public Activity::Model
{
public:
  ZoneGuardModel(
    Activity::ConstModelPtr inner,
    std::vector<std::size_t> zone_waypoints)
  : _inner(std::move(inner)),
    _zone_waypoints(std::move(zone_waypoints))
  {
    // Do nothing
  }

  std::optional<rmf_task::Estimate> estimate_finish(
    rmf_task::State initial_state,
    rmf_traffic::Time earliest_arrival_time,
    const rmf_task::Constraints& constraints,
    const rmf_task::TravelEstimator& travel_estimator) const override
  {
    if (initial_state.waypoint().has_value())
    {
      const auto wp = *initial_state.waypoint();
      for (const auto& zone_wp : _zone_waypoints)
      {
        if (wp == zone_wp)
          return std::nullopt;
      }
    }
    return _inner->estimate_finish(
      std::move(initial_state), earliest_arrival_time,
      constraints, travel_estimator);
  }

  rmf_traffic::Duration invariant_duration() const override
  {
    return _inner->invariant_duration();
  }

  rmf_task::State invariant_finish_state() const override
  {
    return _inner->invariant_finish_state();
  }

private:
  Activity::ConstModelPtr _inner;
  std::vector<std::size_t> _zone_waypoints;
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

  std::vector<rmf_traffic::agv::Plan::Goal> goals;
  std::vector<std::size_t> zone_wp_indices;
  for (const auto& iv : zone_props->internal_vertices())
  {
    const auto* wp = graph.find_waypoint(iv.name());
    if (!wp)
      continue;
    goals.emplace_back(wp->index());
    zone_wp_indices.push_back(wp->index());
  }

  if (goals.empty())
    return nullptr;

  const auto place_desc =
    GoToPlace::Description::make_for_one_of(std::move(goals));
  auto inner_model = place_desc->make_model(
    std::move(invariant_initial_state), parameters);
  if (!inner_model)
    return nullptr;

  // Wrap in ZoneGuardModel so that robots already at a zone vertex
  // get filtered out during bidding (estimate_finish returns nullopt)
  Activity::ConstModelPtr guard = std::make_shared<const ZoneGuardModel>(
    std::move(inner_model), std::move(zone_wp_indices));
  return guard;
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

} // namespace events
} // namespace rmf_task_sequence