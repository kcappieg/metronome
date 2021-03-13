//
// Created by kevin on 5/5/20.
//

#ifndef METRONOME_NAIVEDYNAMICASTAR_HPP
#define METRONOME_NAIVEDYNAMICASTAR_HPP

#include <experiment/Configuration.hpp>
#include "DynamicPlanner.hpp"
#include "AStar.hpp"

namespace metronome {
  /**
   * Naive Dynamic algorithm. Invokes A* every time it is asked to replan.
   * @tparam Domain
   */
  template<typename Domain>
  class NaiveDynamicAStar final : public DynamicPlanner<Domain> {
  public:
    typedef typename Planner<Domain>::ActionBundle ActionBundle;
    typedef typename Domain::State State;

    NaiveDynamicAStar(const Domain&, const Configuration& config) : config_{config} {}

    /**
     * Replan by invoking AStar from the current state
     * @param currentState
     * @param changedStates
     * @param domain
     * @return
     */
    std::vector<ActionBundle> replan(
            const State& currentState,
            const std::vector<State>&,
            const Domain& domain) override {
      AStar<Domain> planner(domain, config_);
      std::vector<typename Domain::Action> plan = planner.plan(currentState);

      std::vector<ActionBundle> planWithCost;
      planWithCost.reserve(plan.size());
      for (auto action : plan) {
        planWithCost.emplace_back(action, domain.getActionDuration(action));
      }

      return planWithCost;
    }
  private:
   const Configuration& config_;
  };

} // namespace metronome

#endif //METRONOME_NAIVEDYNAMICASTAR_HPP
