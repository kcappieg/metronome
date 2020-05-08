//
// Created by kevin on 5/5/20.
//

#ifndef METRONOME_GRDEXPERIMENT_HPP
#define METRONOME_GRDEXPERIMENT_HPP

#include "Experiment.hpp"
#include "MetronomeException.hpp"
#include "easylogging++.h"
#include "termination/TimeTerminationChecker.hpp"

#include <cmath>
#include <utility>
#include <random>

namespace metronome{

template <typename Domain, typename GrdPlanner, typename SubjectPlanner>
class GrdExperiment : public Experiment<Domain, GrdPlanner> {
public:
  typedef typename Domain::State State;

  explicit GrdExperiment(Configuration configuration) : configuration(std::move(configuration)) {
    if (configuration.hasMember(TIME_LIMIT)) {
      TimeTerminationChecker experimentTerminationChecker;
      experimentTerminationChecker.resetTo(configuration.getLong(TIME_LIMIT));

      experimentTimeLimit = experimentTerminationChecker;
    }
  }

  Result plan(const Configuration& configuration,
              const Domain& domain,
              GrdPlanner& grdPlanner) override {
    SubjectPlanner subjectPlanner(domain, configuration);
    using namespace std::chrono;
    LOG(INFO) << "Begin GRD planning iterations";

    // Vector of "turns" taken. Note that in each turn the intervention comes first.
    std::vector<std::pair<typename Domain::Intervention, typename Domain::Action>> turns;

    std::int64_t grdPlanningTime = 0;
    State subjectState = domain.getStartState();
    const State subjectGoal = getSubjectGoal(configuration, domain);

    // Implement loop - get intervention, then action. Repeat.
    while (!domain.isGoal(subjectState, subjectGoal)) {
      // clear goal for observer intervention
      domain.clearCurrentGoal();

      // TODO: Measure planning time, execute GRD iteration
      // Store the whole sequence?
      // apply intervention(s), keep changed states

      domain.setCurrentGoal(subjectGoal);
      // TODO: Invoke Subject Planner (dynamic)
      // First check to see if anything has changed. If not, don't invoke planner
      // Store whole sequence for next iteration (in case nothing changed)

      // TODO: Construct turn pair from heads of both lists
      // validate each turn pair as we go
    }

    return Result(configuration, "Not implemented");
  }
private:
  const Configuration configuration;
  std::optional<TimeTerminationChecker> experimentTimeLimit;

  const State getSubjectGoal(const Configuration& configuration, const Domain& domain) {
    const std::vector<State> goalVector = domain.getGoals();

    // select subject goal from goal prior
    int64_t seed = configuration.getLong(SEED, 1);
    std::mt19937 rand(seed);

    uint32_t goalIndex = 0;

    if (configuration.hasMember(GOAL_PRIORS)) {
      std::vector<double> goalPriors = configuration.getDoubles(GOAL_PRIORS);
      verifyPriors(goalPriors, goalVector);

      std::uniform_real_distribution<double> distribution(0.0, 1.0);

      double num = distribution(rand);
      double sum = goalPriors[0];
      while (sum < num) {
        sum += goalPriors[++goalIndex];
      }
    } else {
      std::uniform_int_distribution<uint32_t> distribution(0, goalVector.size());
      goalIndex = distribution(rand);
    }

    return goalVector[goalIndex];
  }

  /**
   * Verifies the vector sums to one (allowing for slight floating point imprecision)
   * @param priors
   */
  void verifyPriors(std::vector<double>& priors, const std::vector<State>& goalVector) {
    if (priors.size() != goalVector.size()) {
      throw MetronomeException("Different number of goal priors than goals");
    }

    double tally = 1.0;
    for (auto prior : priors) {
      tally -= prior;
    }

    if (std::abs(tally) > 0.000001) throw MetronomeException("Goal priors do not sum to 1");
  }
};

} // namespace metronome

#endif //METRONOME_GRDEXPERIMENT_HPP
