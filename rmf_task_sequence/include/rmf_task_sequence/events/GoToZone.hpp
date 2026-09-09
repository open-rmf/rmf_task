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
  struct Modifiers
  {
    std::string group_hint;
    std::optional<double> orientation_hint;
    std::vector<std::string> preferred_waypoints;
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
