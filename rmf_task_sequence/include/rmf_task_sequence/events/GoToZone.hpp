#ifndef RMF_TASK_SEQUENCE__EVENTS__GOTOZONE_HPP
#define RMF_TASK_SEQUENCE__EVENTS__GOTOZONE_HPP

#include <rmf_traffic/agv/Planner.hpp>

#include <rmf_task_sequence/Event.hpp>

namespace rmf_task_sequence {
namespace events {

//==============================================================================
class GoToZone
{
public:
  class Description;
  using DescriptionPtr = std::shared_ptr<Description>;
  using ConstDescriptionPtr = std::shared_ptr<const Description>;
};

//==============================================================================
class GoToZone::Description : public Event::Description
{
public:
  /// Hints for which waypoint in the zone to assign.
  class Modifiers
  {
  public:
    /// Construct with no hints set.
    Modifiers();

    /// Get the waypoint group to prefer. Empty means no preference.
    const std::string& group_hint() const;

    /// Set the waypoint group to prefer.
    Modifiers& set_group_hint(std::string hint);

    /// Get the orientation to prefer at the waypoint, in radians.
    std::optional<double> orientation_hint() const;

    /// Set the orientation to prefer at the waypoint, in radians.
    Modifiers& set_orientation_hint(std::optional<double> hint);

    /// Get the waypoints to prefer, in order.
    const std::vector<std::string>& preferred_waypoints() const;

    /// Set the waypoints to prefer, in order.
    Modifiers& set_preferred_waypoints(std::vector<std::string> waypoints);

    class Implementation;
  private:
    rmf_utils::impl_ptr<Implementation> _pimpl;
  };

  /// Make a GoToZone description using a zone name and optional modifiers.
  static DescriptionPtr make(
    std::string zone_name,
    std::optional<Modifiers> modifiers = std::nullopt);

  /// Get the name of the zone for this description.
  const std::string& zone_name() const;

  /// Get the modifiers for this description.
  const std::optional<Modifiers>& modifiers() const;

  // Documentation inherited
  Activity::ConstModelPtr make_model(
    State invariant_initial_state,
    const Parameters& parameters) const final;

  // Documentation inherited
  Header generate_header(
    const State& initial_state,
    const Parameters& parameters) const final;

  class Implementation;
private:
  Description();
  rmf_utils::impl_ptr<Implementation> _pimpl;
};

} // namespace events
} // namespace rmf_task_sequence

#endif // RMF_TASK_SEQUENCE__EVENTS__GOTOZONE_HPP
