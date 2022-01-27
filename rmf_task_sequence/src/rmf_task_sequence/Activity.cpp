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

#include <rmf_task_sequence/Activity.hpp>

namespace rmf_task_sequence {

//==============================================================================
class Activity::SequenceModel::Implementation
{
public:
  std::vector<Activity::ConstModelPtr> models;
  rmf_task::State invariant_finish_state;
  rmf_traffic::Duration invariant_duration;
};

//==============================================================================
Activity::ConstModelPtr Activity::SequenceModel::make(
  const std::vector<ConstDescriptionPtr>& descriptions,
  rmf_task::State invariant_initial_state,
  const rmf_task::Parameters& parameters)
{
  std::vector<Activity::ConstModelPtr> models;
  rmf_task::State invariant_finish_state = invariant_initial_state;
  rmf_traffic::Duration invariant_duration = rmf_traffic::Duration(0);
  for (const auto& desc : descriptions)
  {
    auto next_model = desc->make_model(invariant_finish_state, parameters);
    if (!next_model)
    {
      // TODO: Should we throw an error here?
      return nullptr;
    }
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
std::optional<Estimate> Activity::SequenceModel::estimate_finish(
  rmf_task::State initial_state,
  rmf_traffic::Time earliest_arrival_time,
  const rmf_task::Constraints& constraints,
  const rmf_task::TravelEstimator& travel_estimator) const
{
  rmf_task::State finish_state = std::move(initial_state);
  std::optional<rmf_traffic::Time> wait_until;
  for (const auto& model : _pimpl->models)
  {
    const auto estimate = model->estimate_finish(
      std::move(finish_state),
      earliest_arrival_time,
      constraints,
      travel_estimator);

    if (!estimate.has_value())
      return std::nullopt;

    finish_state = estimate->finish_state();
    if (!wait_until.has_value())
      wait_until = estimate->wait_until();
  }

  if (!wait_until.has_value())
  {
    // This means that the models are empty, which is probably an error...
    wait_until = earliest_arrival_time;
  }

  return Estimate(std::move(finish_state), *wait_until);
}

//==============================================================================
rmf_traffic::Duration Activity::SequenceModel::invariant_duration() const
{
  return _pimpl->invariant_duration;
}

//==============================================================================
rmf_task::State Activity::SequenceModel::invariant_finish_state() const
{
  return _pimpl->invariant_finish_state;
}

//==============================================================================
Activity::SequenceModel::SequenceModel()
{
  // Do nothing
}


} // namespace rmf_task_sequence
