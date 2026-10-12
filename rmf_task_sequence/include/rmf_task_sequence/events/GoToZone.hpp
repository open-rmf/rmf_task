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
  /// One preference for the zone assignment: where to park, and facing
  /// which way. Name a group, or a waypoint, or neither to accept any
  /// waypoint in the zone.
  class Hint
  {
  public:
    /// Construct with nothing named, which accepts any waypoint at any
    /// orientation.
    Hint();

    /// Get the group to prefer. Empty if none.
    const std::string& group() const;

    /// Set the group to prefer.
    Hint& set_group(std::string group);

    /// Get the waypoint to prefer. Empty if none.
    const std::string& waypoint() const;

    /// Set the waypoint to prefer.
    Hint& set_waypoint(std::string waypoint);

    /// Get the orientations the robot may park at, in radians. Empty means
    /// any.
    const std::vector<double>& orientations() const;

    /// Set the orientations the robot may park at, in radians.
    Hint& set_orientations(std::vector<double> orientations);

    class Implementation;
  private:
    rmf_utils::impl_ptr<Implementation> _pimpl;
  };

  /// Make a GoToZone description using a zone name and ordered hints, most
  /// preferred first.
  static DescriptionPtr make(
    std::string zone_name,
    std::vector<Hint> hints = {});

  /// Get the name of the zone for this description.
  const std::string& zone_name() const;

  /// Get the hints for this description, most preferred first.
  const std::vector<Hint>& hints() const;

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
