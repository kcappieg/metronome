//
// Created by kevin on 5/5/20.
//

#ifndef METRONOME_TRIVIALGRDPLANNER_HPP
#define METRONOME_TRIVIALGRDPLANNER_HPP

#include <experiment/Configuration.hpp>
#include "GoalRecognitionDesignPlanner.hpp"

namespace metronome {
  template <typename Domain>
  class TrivialGrdPlanner final : public GoalRecognitionDesignPlanner<Domain> {
  public:
    using State = typename Domain::State;
    using Action = typename Domain::Action;
    using Intervention = typename Domain::Intervention;
    using Cost = typename Domain::Cost;
    using Planner = metronome::Planner<Domain>;
    using InterventionBundle = typename GoalRecognitionDesignPlanner<Domain>::InterventionBundle;

    TrivialGrdPlanner(const Domain&, const Configuration&){}

    std::vector<InterventionBundle> selectInterventions(
            const typename Domain::State&, const Domain& systemState
    ) override {
      GoalRecognitionDesignPlanner<Domain>::beginIteration();

      // Depends on identity intervention function being available
      Intervention inter = systemState.getIdentityIntervention();
      return {{inter, 1}};
    }

    InterventionBundle getIdentityIntervention(const Domain& systemState) const override {
      return {systemState.getIdentityIntervention(), 1};
    }

    State getGoalPrediction(const Domain& systemState,
                                             const State& subjectState) override {
      for (auto goal : systemState.getGoals()) {
        if (systemState.isGoal(subjectState, goal)) return goal;
      }

      // just return the first one
      return systemState.getGoals()[0];
    }
  };

} // namespace metronome

#endif //METRONOME_TRIVIALGRDPLANNER_HPP
