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

#include <rmf_task/sequence/Phase.hpp>

namespace rmf_task {
namespace sequence {

//==============================================================================
class Phase::SequenceModel::Implementation
{
public:
  std::vector<Phase::ConstModelPtr> models;
  State invariant_finish_state;
  rmf_traffic::Duration invariant_duration;
};

//==============================================================================
Phase::ConstModelPtr Phase::SequenceModel::make(
  const std::vector<ConstDescriptionPtr>& descriptions,
  State invariant_initial_state,
  const Parameters& parameters)
{
  std::vector<Phase::ConstModelPtr> models;
  State invariant_finish_state = invariant_initial_state;
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
std::optional<State> Phase::SequenceModel::estimate_finish(
  State initial_state,
  const Constraints& constraints,
  const TravelEstimator& travel_estimator) const
{
  std::optional<State> finish_state = std::move(initial_state);
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
State Phase::SequenceModel::invariant_finish_state() const
{
  return _pimpl->invariant_finish_state;
}

//==============================================================================
Phase::SequenceModel::SequenceModel()
{
  // Do nothing
}

} // namespace sequence
} // namespace rmf_task
