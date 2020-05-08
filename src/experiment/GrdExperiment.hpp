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
    std::vector<std::pair<typename Domain::Invervention, typename Domain::Action>> turns;

    std::int64_t grdPlanningTime = 0;
    auto subjectState = domain.getStartState();

    // select subject goal from goal prior
    int64_t seed = configuration.getLong(SEED, 1);
    std::vector<double> goalPriors = configuration.getDoubles(GOAL_PRIORS);
    verifyPriors(goalPriors);

    std::mt19937 rand(seed);
    std::uniform_real_distribution<double> distribution(0.0, 1.0);

    double num = distribution(rand);
    double sum = goalPriors[0];
    uint32_t i = 0;
    while (sum < num) {
      sum += goalPriors[++i];
    }

    auto subjectGoal = domain.getGoals()[i];

    // Implement loop - get first action from subject planner, then get intervention. Repeat
    // validate each turn pair as we go

    return Result(configuration, "Not implemented");
  }
private:
  const Configuration configuration;
  std::optional<TimeTerminationChecker> experimentTimeLimit;

  Domain::State getSubjectGoal(const Configuration& configuration, const Domain& domain) {
    const std::vector<typename Domain::State> goalVector = domain.getGoals();

    // select subject goal from goal prior
    int64_t seed = configuration.getLong(SEED, 1);
    std::vector<double> goalPriors = configuration.getDoubles(GOAL_PRIORS);
    verifyPriors(goalPriors, goalVector);

    std::mt19937 rand(seed);
    std::uniform_real_distribution<double> distribution(0.0, 1.0);

    double num = distribution(rand);
    double sum = goalPriors[0];
    uint32_t i = 0;
    while (sum < num) {
      sum += goalPriors[++i];
    }

    auto subjectGoal = domain.getGoals()[i];
  }

  /**
   * Verifies the vector sums to one (allowing for slight floating point imprecision)
   * @param priors
   */
  void verifyPriors(std::vector<double>& priors, const std::vector<typename Domain::State> goalVector) {
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
