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
#include "utils/File.hpp"
#include "utils/Meta.hpp"

#include <cmath>
#include <utility>
#include <random>
#include <deque>
#include <fstream>

namespace metronome{

// SFINAE Functions - let all domains compile

/** Having fun with macros, don't judge me */
DEFINE_METHOD_CALL_IF_AVAILABLE(visualize, visualize)
DEFINE_METHOD_CALL_IF_AVAILABLE(giveTerminationChecker, setTerminationChecker)


template <typename Domain, typename GrdPlanner, typename SubjectPlanner>
class GrdExperiment : public Experiment<Domain, GrdPlanner> {
public:
  typedef typename Domain::State State;
  typedef typename Domain::Action Action;
  typedef typename Domain::Intervention Intervention;
  typedef typename Planner<Domain>::ActionBundle ActionBundle;

  explicit GrdExperiment(const Configuration& configuration) {
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
    std::vector<std::pair<Intervention, Action>> turns;

    // Keep a temporary cache of interventions / actions
    // These are used when either the dynamic or GRD algorithm
    // either chooses not to return anything to do or doesn't need to (Dynamic algorithms)
    std::deque<InterventionBundle<Domain>> cachedInterventions;
    std::deque<ActionBundle> cachedActions;

    std::int64_t grdPlanningTime = 0;
    std::int64_t grdExecutionCost = 0; // cost of interventions
    std::int64_t subjectExecutionCost = 0;
    /** One turn = 1 Observer Intervention + 1 Subject Action */
    std::size_t turnCount = 0;
    std::size_t goalReportedOnTurn = 0;
    State goalPrediction{};

    State subjectState = domain.getStartState();
    const State subjectGoal = getSubjectGoal(configuration, domain);
    LOG(INFO) << "Subject goal: " << subjectGoal;

#ifdef ENABLE_PLAYBACK
    LOG(INFO) << "Playback enabled. Initializing write stream";
    std::string playbackPath{ "./results/playback/" +
                             (configuration.hasMember(PLAYBACK_FILENAME) ? configuration.getString(PLAYBACK_FILENAME)
                                                                         : "playback.grid-viz")
    };

    std::fstream playbackOutputStream{playbackPath, std::fstream::out};

    visualize(domain, playbackOutputStream);
#endif

    // if planner accepts termination checker, give it
    giveTerminationChecker(grdPlanner, experimentTimeLimit.value());

    try {
      // Implement loop - get intervention, then action. Repeat.
      while (!domain.isGoal(subjectState, subjectGoal)) {
        if (experimentTimeLimit.has_value() &&
            experimentTimeLimit.value().reachedTermination()) {
          throw MetronomeTimeoutException();
        }

        // clear goal for Observer Intervention
        domain.clearCurrentGoal();
        turnCount++;

        // Measure planning time, execute GRD iteration
        const auto iterationStartTime = currentNanoTime();
        std::vector<InterventionBundle<Domain>> interventions = grdPlanner.selectInterventions(subjectState, domain);
        const auto iterationEndTime = currentNanoTime();

        const auto iterationDuration = iterationEndTime - iterationStartTime;
        grdPlanningTime += iterationDuration;

        if (interventions.size() > 0) {
          // recreate deque
          cachedInterventions = std::deque<InterventionBundle<Domain>>(interventions.begin(), interventions.end());
        } else if (cachedInterventions.size() == 0) {
          cachedInterventions.push_back(grdPlanner.getIdentityIntervention(domain));
        }

        // apply next intervention, keep changed states
        const InterventionBundle interventionBundle = cachedInterventions.front();
        const auto patch = validateIntervention(domain, interventionBundle.intervention, subjectState);
        grdExecutionCost += interventionBundle.interventionCost;

#ifdef ENABLE_PLAYBACK
        visualize(domain, playbackOutputStream, subjectState);
#endif


        // Subject Iteration
        // First check to see if anything has changed (or we've run out of actions). If not, don't invoke planner
        if (patch.affectedStates.size() > 0 || cachedActions.size() == 0) {
          domain.setCurrentGoal(subjectGoal);

          // dynamic planner
          // we don't care so much about its timing, so just plan and store
          std::vector<ActionBundle> actions = subjectPlanner.replan(subjectState, patch.affectedStates, domain);

          // cache actions in case GRD planner doesn't affect any states
          cachedActions = std::deque<ActionBundle>(actions.begin(), actions.end());
        }

        // transition subject
        const ActionBundle actionBundle = cachedActions.front();
        subjectState = validateAction(domain, subjectState, actionBundle.action);
        subjectExecutionCost += actionBundle.actionDuration;

#ifdef ENABLE_PLAYBACK
        visualize(domain, playbackOutputStream, subjectState);
#endif

        // construct pair
        turns.emplace_back(interventionBundle.intervention, actionBundle.action);

        cachedActions.pop_front();
        cachedInterventions.pop_front();

        // get goal prediction
        auto optionalGoalPrediction = grdPlanner.getGoalPrediction(domain, subjectState);
        if (optionalGoalPrediction.has_value()) {
          goalPrediction = *optionalGoalPrediction;
          if (goalPrediction == subjectGoal) {
            if (goalReportedOnTurn == 0) {
              goalReportedOnTurn = turnCount;
            }
          } else {
            // reset for incorrect predictions
            goalReportedOnTurn = 0;
          }
        }
      }
    } catch (MetronomeTimeoutException& timeoutEx) {
      // handle timeouts specially so we can report on certain info
      Result result{configuration, "Timeout"};
      result.attributes = grdPlanner.getAttributes();

      return result;
    }


#ifdef ENABLE_PLAYBACK
    // close playback outstream
    playbackOutputStream.close();
#endif

    std::vector<std::string> turnList;
    for (auto& turn : turns) {
      std::ostringstream str;
      str << "Intervention: " << turn.first
        << "; Action: " << turn.second;

      turnList.push_back(str.str());
    }

    Result result(configuration,
                  grdPlanner.getExpandedNodeCount(),
                  grdPlanner.getGeneratedNodeCount(),
                  grdPlanningTime,
                  grdExecutionCost,
                  0,
                  0,
                  0,
                  turns.size(),
                  turnList,
                  turnCount);

    result.attributes = grdPlanner.getAttributes();
    result.attributes.emplace_back("goalReportedIteration", goalReportedOnTurn);

    return result;
  }
private:
  std::optional<TimeTerminationChecker> experimentTimeLimit;

  State getSubjectGoal(const Configuration& configuration, const Domain& domain) {
    const std::vector<State> goalVector = domain.getGoals();

    uint64_t goalIndex = 0;

    if (not configuration.hasMember(GOAL_PRIORS)) {
      throw MetronomeException("No goal priors specified");
    }
    auto goalPriors = configuration.getDoubles(GOAL_PRIORS);
    verifyPriors(goalPriors, goalVector);

    if (configuration.hasMember(SUBJECT_GOAL)) {
      // explicit goal
      goalIndex = configuration.getLong(SUBJECT_GOAL);

    } else {
      // select subject goal from goal prior
      int64_t seed = configuration.getLong(SEED, 1);
      std::mt19937 rand(seed);

      std::uniform_real_distribution<double> distribution(0.0, 1.0);

      double num = distribution(rand);
      double sum = goalPriors[0];
      while (sum < num) {
        sum += goalPriors[++goalIndex];
      }
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

  State validateAction(const Domain& domain,
                       const State& state,
                       const Action& action) {
    auto nextState = domain.transition(state, action);

    if (!nextState.has_value()) {
      LOG(ERROR) << "Invalid action " << action << " from: " << state;
      throw MetronomeException("Invalid action. Subject plan is corrupt.");
    }
    LOG(INFO) << "> action from: " << state << " to " << *nextState;

    return nextState.value();
  }

  typename Domain::Patch validateIntervention(Domain& domain,
                                              const Intervention& intervention,
                                              const State& subjectState) {
    std::optional<typename Domain::Patch> patch = domain.applyIntervention(intervention, subjectState);

    if (!patch.has_value()) {
      LOG(ERROR) << "Invalid intervention " << intervention << " at subject state " << subjectState;
      throw MetronomeException("Invalid intervention. GRD plan is corrupt");
    }
    LOG(INFO) << "> intervention applied: " << intervention;

    return patch.value();
  }
};

} // namespace metronome

#endif //METRONOME_GRDEXPERIMENT_HPP
