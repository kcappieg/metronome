//
// Created by kevin on 5/5/20.
//

#ifndef METRONOME_GRDEXPERIMENT_HPP
#define METRONOME_GRDEXPERIMENT_HPP

#include "Experiment.hpp"
#include "MetronomeException.hpp"
#include "easylogging++.h"
#include "termination/TimeTerminationChecker.hpp"
#include "algorithms/GoalRecognitionDesignPlanner.hpp"

#include <cmath>
#include <utility>
#include <random>
#include <deque>

namespace metronome{

template <typename Domain, typename GrdPlanner, typename SubjectPlanner>
class GrdExperiment : public Experiment<Domain, GrdPlanner> {
public:
  typedef typename Domain::State State;
  typedef typename GoalRecognitionDesignPlanner<Domain>::InterventionBundle InterventionBundle;
  typedef typename Planner<Domain>::ActionBundle ActionBundle;

  explicit GrdExperiment(Configuration configuration) : configuration(std::move(configuration)) {
    if (configuration.hasMember(TIME_LIMIT)) {
      TimeTerminationChecker experimentTerminationChecker;
      experimentTerminationChecker.resetTo(configuration.getLong(TIME_LIMIT));

      experimentTimeLimit = experimentTerminationChecker;
    }
  }

  Result plan(const Configuration&,
              const Domain&,
              GrdPlanner&) override {
    throw MetronomeException("Method signature invalid for GRD");
  }


  Result plan(const Configuration& configuration,
              Domain& domain,
              GrdPlanner& grdPlanner) {
    SubjectPlanner subjectPlanner(domain, configuration);
    using namespace std::chrono;
    LOG(INFO) << "Begin GRD planning iterations";

    // Vector of "turns" taken. Note that in each turn the intervention comes first.
    std::vector<std::pair<typename Domain::Intervention, typename Domain::Action>> turns;

    // Keep a temporary cache of interventions / actions
    // These are used when either the dynamic or GRD algorithm
    // either chooses not to return anything to do or doesn't need to (Dynamic algorithms)
    std::deque<InterventionBundle> cachedInterventions;
    std::deque<ActionBundle> cachedActions;

    std::int64_t grdPlanningTime = 0;
    std::int64_t grdExecutionTime = 0; // cost of interventions
    /** One turn = 1 Observer Intervention + 1 Subject Action */
    std::size_t turnCount = 0;

    State subjectState = domain.getStartState();
    const State subjectGoal = getSubjectGoal(configuration, domain);

    // Implement loop - get intervention, then action. Repeat.
    while (!domain.isGoal(subjectState, subjectGoal)) {
      // clear goal for Observer Intervention
      domain.clearCurrentGoal();
      turnCount++;

      // TODO: Measure planning time, execute GRD iteration
      const auto iterationStartTime = currentNanoTime();
      std::vector<InterventionBundle> interventions = grdPlanner.selectInterventions(subjectState, domain);
      const auto iterationEndTime = currentNanoTime();

      const auto iterationDuration = iterationEndTime - iterationStartTime;
      grdPlanningTime += iterationDuration;

      if (interventions.size() > 1) {
        cachedInterventions = std::deque(interventions.begin(), interventions.end());
      } else if (cachedInterventions.size() == 0) {
        cachedInterventions.push_back(grdPlanner.getIdentityIntervention(domain));
      }

      // apply next intervention, keep changed states
      const auto patch = domain.applyIntervention(cachedInterventions.pop_front());


      // Subject Iteration
      // First check to see if anything has changed. If not, don't invoke planner
      if (patch.affectedStates)
      domain.setCurrentGoal(subjectGoal);
      // TODO: Invoke Subject Planner (dynamic)
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
