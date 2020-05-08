//
// Created by kevin on 5/5/20.
//

#ifndef METRONOME_DYNAMICPLANNER_HPP
#define METRONOME_DYNAMICPLANNER_HPP

#include <vector>
#include "Planner.hpp"

namespace metronome {
  template <typename Domain>
  class DynamicPlanner: public Planner<Domain> {
  public:
    ~DynamicPlanner() override = default;

    /**
     * Replan in a dynamic context
     * @param currentState The state the agent is currently in
     * @param changedStates Any states whose edge costs have changed since the last call to replan.
     * Will be empty on first call
     * @param domain Reference to the domain whose edge costs have changed
     * @return Full plan to goal
     */
    virtual std::vector<typename Planner<Domain>::ActionBundle> replan(
            const typename Domain::State& currentState,
            const std::vector<typename Domain::State>& changedStates,
            const Domain& domain) = 0;
  };

} // namespace metronome

#endif //METRONOME_DYNAMICPLANNER_HPP
