/*
 * Copyright (C) 2021 Open Source Robotics Foundation
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

#include <rmf_task_sequence/Phase.hpp>

namespace rmf_task_sequence {

//==============================================================================
class Phase::Activator::Implementation
{
public:

  std::unordered_map<std::type_index, Activate<Phase::Description>> activators;

};

//==============================================================================
Phase::ActivePtr Phase::Activator::activate(
  std::function<State()> get_state,
  ConstTagPtr tag,
  const Description& description,
  std::optional<nlohmann::json> backup_state,
  std::function<void(rmf_task::Phase::ConstSnapshotPtr)> update,
  std::function<void(Active::Backup)> checkpoint,
  std::function<void()> finished) const
{
  const auto& type = typeid(description);
  const auto it = _pimpl->activators.find(type);
  if (it == _pimpl->activators.end())
    return nullptr;

  return it->second(
    std::move(get_state),
    std::move(tag),
    description,
    std::move(backup_state),
    std::move(update),
    std::move(checkpoint),
    std::move(finished));
}

//==============================================================================
void Phase::Activator::_add_activator(
  std::type_index type, Activate<Phase::Description> activator)
{
  _pimpl->activators.insert_or_assign(type, std::move(activator));
}

//==============================================================================
class Phase::SequenceModel::Implementation
{
public:
  std::vector<Phase::ConstModelPtr> models;
  rmf_task::State invariant_finish_state;
  rmf_traffic::Duration invariant_duration;
};

//==============================================================================
Phase::ConstModelPtr Phase::SequenceModel::make(
  const std::vector<ConstDescriptionPtr>& descriptions,
  rmf_task::State invariant_initial_state,
  const rmf_task::Parameters& parameters)
{
  std::vector<Phase::ConstModelPtr> models;
  rmf_task::State invariant_finish_state = invariant_initial_state;
  rmf_traffic::Duration invariant_duration = rmf_traffic::Duration(0);
  for (const auto& desc : descriptions)
  {
    auto next_model = desc->make_model(invariant_finish_state, parameters);
    invariant_finish_state = next_model->invariant_finish_state();
    invariant_duration += next_model->invariant_duration();

    models.emplace_back(std::move(next_model));
  }

  auto output = std::shared_ptr<SequenceModel>(new SequenceModel);
  output->_pimpl = rmf_utils::make_unique_impl<Implementation>(
    Implementation{
      std::move(models),
      std::move(invariant_finish_state),
      invariant_duration
    });

  return output;
}

//==============================================================================
std::optional<rmf_task::State> Phase::SequenceModel::estimate_finish(
  rmf_task::State initial_state,
  const rmf_task::Constraints& constraints,
  const rmf_task::TravelEstimator& travel_estimator) const
{
  std::optional<rmf_task::State> finish_state = std::move(initial_state);
  for (const auto& model : _pimpl->models)
  {
    finish_state = model->estimate_finish(
      std::move(*finish_state), constraints, travel_estimator);

    if (!finish_state.has_value())
      return std::nullopt;
  }

  return *finish_state;
}

//==============================================================================
rmf_traffic::Duration Phase::SequenceModel::invariant_duration() const
{
  return _pimpl->invariant_duration;
}

//==============================================================================
rmf_task::State Phase::SequenceModel::invariant_finish_state() const
{
  return _pimpl->invariant_finish_state;
}

//==============================================================================
Phase::SequenceModel::SequenceModel()
{
  // Do nothing
}

} // namespace rmf_task_sequence
