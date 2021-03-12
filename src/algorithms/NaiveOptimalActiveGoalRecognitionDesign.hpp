//
// Created by kevin on 5/14/20.
//

#ifndef METRONOME_NAIVEOPTIMALACTIVEGOALRECOGNITIONDESIGN_HPP
#define METRONOME_NAIVEOPTIMALACTIVEGOALRECOGNITIONDESIGN_HPP

#include <algorithm>
#include <cassert>
#include <limits>
#include <sstream>
#include <string>
#include <unordered_map>
#include <unordered_set>
#include <queue>

#include <experiment/Configuration.hpp>
#include "GoalRecognitionDesignPlanner.hpp"
#include "Planner.hpp"
#include "utils/Hash.hpp"
#include "utils/ObjectPool.hpp"
#include <MemoryConfiguration.hpp>
#include <utility>
#include "utils/PriorityQueue.hpp"
#include "planner_tools/Comparators.hpp"
#include "domains/SuccessorBundle.hpp"
#include "utils/TimeMeasurement.hpp"
#include "experiment/termination/TimeTerminationChecker.hpp"

#define NAIVEOPTIMALACTIVEGOALRECOGNITIONDESIGN_DEBUG_TRACE 0
#define NAIVEOPTIMALACTIVEGOALRECOGNITIONDESIGN_LOG_TO_DEPTH 6

namespace metronome {
  // quick and sloppy way to get unique ID. Sue me...
  uint64_t idCounter = 0;

  class RecomputeOptimalException : public MetronomeException {
   public:
    explicit RecomputeOptimalException(std::string msg) noexcept : MetronomeException(std::move(msg)) {}
    RecomputeOptimalException(const RecomputeOptimalException&) noexcept = default;
  };

  template<typename Domain>
  class NaiveOptimalActiveGoalRecognitionDesign final : public GoalRecognitionDesignPlanner<Domain> {
  public:
    using State = typename Domain::State;
    using StateHash = typename metronome::Hash<State>;
    using Action = typename Domain::Action;
    using Intervention [[maybe_unused]] = typename Domain::Intervention;
    using Cost = typename Domain::Cost;
    using Planner = metronome::Planner<Domain>;
    using Base = metronome::GoalRecognitionDesignPlanner<Domain>;
    using Patch = typename Domain::Patch;

    enum class Metric {
      REACTION_TIME,
      FRACTION_OF_OPTIMAL,
      DISTINCTIVENESS
    };

    // TODO make this configurable if we want
    // either min or max
    static constexpr bool optimizeMin{true};
    static constexpr Metric objective{Metric::FRACTION_OF_OPTIMAL};

    NaiveOptimalActiveGoalRecognitionDesign(const Domain& domain, const Configuration& config)
        : openList(Memory::OPEN_LIST_SIZE, &fComparator<Node>),
          maxDepth(config.getLong(MAX_DEPTH, std::numeric_limits<uint32_t>::max())),
          iterativeWidening{config.getBool(GRD_ITERATIVE_WIDENING, false)}
    {
      if (config.hasMember(GOAL_PRIORS)) {
        std::vector<double> goalPriors = config.getDoubles(GOAL_PRIORS);

        size_t index = 0;
        for (const State& goal : domain.getGoals()) {
          goalsToPriors[goal] = goalPriors[index++];
        }
      } else {
        throw MetronomeException("No goal priors specified");
      }

      // init term checker to action duration for iterative deepening
      // TODO Make this more configurable / standard in GRD framework
      //  This is quick + dirty thesis research code. Not good for generalizing
      //  this framework
      if (iterativeWidening){
        terminationChecker.emplace();
        planningIterationTimeLimit = config.getLong(ACTION_DURATION);
      }
    }

    std::vector<InterventionBundle<Domain>> selectInterventions(
            const State& subjectState, const Domain& systemState
    ) override {
      // set term checker for iterative deepening
      if (iterativeWidening) {
        terminationChecker->resetTo(planningIterationTimeLimit);
      }

      const auto iterationStartTime = currentNanoTime();
      GoalRecognitionDesignPlanner<Domain>::beginIteration();
      // copy domain, set accessible pointer to it
      Domain localDomain(systemState);
      domain = &localDomain;
      depth = 0;
      bool storeFirstIterationRuntime = false;
      if (!rootState.has_value()) {
        rootState = domain->getStartState();
        storeFirstIterationRuntime = true;
      }

      if (!potentialOutcomes.empty()) {
        bool transitioned = false;
        for (ActionProbability& actionResult : potentialOutcomes) {
          if (actionResult.successor->state == subjectState) {
            goalsToPriors = actionResult.conditionedGoalPriors;
            transitioned = true;
          }
        }
        if (!transitioned) {
          throw MetronomeException(
              "Unexpected transition! The Subject transitioned to a state the Observer did not expect");
        }
      }

#if NAIVEOPTIMALACTIVEGOALRECOGNITIONDESIGN_DEBUG_TRACE
      std::stringstream sstream{};
      for (auto& entry : goalsToPriors) {
        sstream << entry.first << " - " << entry.second << "; ";
      }
      LOG(DEBUG) << "Goal Priors at iteration " << Base::iterationCount
                 << ": " << sstream.str();
#endif

      uint64_t recomputeOptimalBegin = recomputeOptimalIteration;

      recomputeOptimalInfo(subjectState);
      Node* currentStateNode = nodes[subjectState];
      // Set widening threshold to consider actions whose likelihood equals
      // the most likely action.
      topNActionProbabilty = 1;

      // Store optimal plan lengths, but only on first iteration
      if (recomputeOptimalIteration == 1) {
        size_t index = 0;
        for (auto& entry : goalsToOptimalCost) {
          Base::recordAttribute("Goal_" + std::to_string(index++), entry.second);
        }
      }

      std::optional<InterventionTrialResult> incumbent{};
      if (iterativeWidening) {
        // init the incumbent as the identity action. Call this the first iteration :)
        incumbent.emplace(InterventionTrialResult{
            0.0,
            Base::getIdentityIntervention(localDomain),
            computeActionProbabilities(currentStateNode, goalsToPriors)});
      }

      // TODO: atrocious code. Iterative-deepening should be executed maybe in another function
      //  Refactor if you want to keep using. (Whole file is atrocious at this point...)
      try {
        while (not incumbent or (iterativeWidening and not terminationChecker->reachedTermination()) ) {
          auto trialResult = interventionTrial(currentStateNode, currentStateNode);

          // If the incumbent doesn't exist or the best intervention exists.
          // This check prevents us from overwriting the default Identity intervention
          // when no intervention was identified
          if (not incumbent or trialResult.bestIntervention) {
            incumbent.emplace(std::move(trialResult));
          }

          // this will allow the alg to consider actions whose likelihood is equal to the
          // next highest *likelihood* of actions at each subject node. e.g. if this number
          // is 2 and we have 4 actions with likelihoods A: 0.35, B: 0.3, C: 0.3, D: 0.05,
          // the second highest likelihood is 0.3. Therefore, we will consider
          // actions A, B, and C but not D which does not meet the threshold of 0.3
          ++topNActionProbabilty;
        }
      } catch (const MetronomeTimeoutException& timeoutEx) {
        if (not iterativeWidening) {
          throw timeoutEx;
        }
      }

      LOG(DEBUG) << "Intervention score: " << incumbent->score;

      std::optional<InterventionBundle<Domain>> intervention = incumbent->bestIntervention;
      potentialOutcomes = incumbent->potentialSubjectActionOutcomes;

      domain = nullptr;
      depth = 0;
      Base::recordAttribute("numOptimalRecomputations", recomputeOptimalIteration - recomputeOptimalBegin);

      // instead of averaging all iteration runtimes, we're most concerned with the
      // first iteration for the Naive algorithm.
      // You could imagine caching computation for subsequent iterations...
      auto iterationRuntime = currentNanoTime() - iterationStartTime;
      if (storeFirstIterationRuntime) {
        Base::recordAttribute("firstIterationRuntime", iterationRuntime);
      }
      // also keep average (mostly for iterative deepening)
      Base::recordAttribute("iterationRuntime", iterationRuntime);

      if (not intervention) {
        // The experiment runner will implicitly choose "identity"
        return {};
      }

      // we want to know if this domain actually has something for the observer
      // to do, so store that here
      if (intervention->intervention != domain->getIdentityIntervention()) {
        Base::recordAttribute("observerIsActive", 1);
      }

      return {*intervention};
    }

    std::optional<State> getGoalPrediction(const Domain&,
                            const State& subjectState) override {
      Node* subjectStateNode = nodes[subjectState];
      if (subjectStateNode->goalsToPlanCount.size() == 1) {
        return {subjectStateNode->goalsToPlanCount.begin()->first};
      }

      return {};
    }

    void setExperimentTerminationChecker(TimeTerminationChecker& experimentTerminationChecker) {
      // only set if we haven't already set the termination checker
      if (not terminationChecker) {
        terminationChecker = experimentTerminationChecker;
      }
    }

  private:
    struct Edge;
    // Search Node
    struct Node {
    public:
      Node(State state, Cost g, Cost h, unsigned int iteration)
              : id{idCounter++}, state(std::move(state)), g(g), h(h), iteration{iteration} {
        goalBackupIteration = 0;
      }

      size_t hash() const { return state.hash(); }
      bool operator==(const Node& node) const { return state == node.state; }

      Cost f() const { return g + h; }

      /** Index used by the priority queue */
      mutable unsigned int index{std::numeric_limits<unsigned int>::max()};

      uint64_t id;
      const State state;
      Cost g;
      Cost h;

      /** Init parent set */
      std::vector<Edge> parents{};
      /** Init successor set */
      std::vector<Edge> successors{};
      bool open = false;

      /** Goal Hypothesis - goal state maps to plan count */
      std::unordered_map<State, size_t, StateHash> goalsToPlanCount{};

      unsigned int iteration;
      unsigned int goalBackupIteration;

      std::string toString() const {
        std::ostringstream stream;
        stream << "State: " << state << "; g: " << g << "; Iteration: " << iteration;

        return stream.str();
      }
    };
    // Search Edge
    struct Edge {
    public:
      Edge(Node* from, Node* to, Cost actionCost) : from(from), to(to), actionCost(actionCost) {}

      Node* from;
      Node* to;
      Cost actionCost;
    };

    // Contains probability of action and resulting conditioned goal probability
    struct ActionProbability {
      Node* successor{nullptr};
      double probabilityOfAction{0};
      /** Goal priors conditioned on this action being taken. These values are normalized */
      std::unordered_map<State, double, StateHash> conditionedGoalPriors{};
    };

    struct InterventionTrialResult {
      double score;
      std::optional<InterventionBundle<Domain>> bestIntervention;
      std::vector<ActionProbability> potentialSubjectActionOutcomes;
    };

    struct ActionTrialResult {
      double score;
      std::vector<ActionProbability> potentialSubjectActionOutcomes;
    };

//    // debug methods
//    Node getNode(uint64_t x, uint64_t y) {
//      return *nodes[State(x, y)];
//    }

    /*****************************************
            Algorithm methods
            Methods with formal definitions
    *****************************************/

    /**
     * Enumerate all optimal plans to all goals so that we can configure goal hypotheses and plan counts.
     * Basically A* except we don't stop until we've exhausted the full f-layer of all goals
     *
     * WARNING: This method does not handle adding edges. Adding edges could alter g-values
     * that are not repaired by recomputing from the simulated (current) state.
     * Instead, to repair values in the graph you must recompute from the subject's actual
     * current state (i.e. not the simulated state)
     * @param currentState State from which to start the recomputation
     */
    void recomputeOptimalInfo(const State& currentState) {
      // reset open list, goal info, optimal plan states
      openList.clear();
      openList.reorder(&fComparator<Node>);
      goalsToOptimalCost.clear();
      recomputeOptimalIteration++;

      /* NOTE:
       * The node already has the correct cost-to-come (g) value
       * We will base the search off this existing value
       */
      Node* startNode = getSuccessor(nullptr, currentState, 0, false);

      if (currentState == rootState.value()) { // indicates this is root node on first pass
        startNode->g = 0;
      }

      startNode->iteration = recomputeOptimalIteration;

      openList.push(*startNode);

      // set up local tracking for goals. We will tick them off when we expand them
      std::vector<State> goals = domain->getGoals();
      // init all to true, then mark as false any goals in the goalsToPriors
      std::vector<bool> foundGoals{};
      foundGoals.resize(goals.size(), true);

      // only searching for goals we haven't already eliminated from the
      // search's current state
      size_t goalsLeft = 0;
      for (size_t i = 0; i < goals.size(); i++) {
        auto& goal = goals[i];
        if (startNode->goalsToPlanCount.count(goal) > 0 || startNode->g == 0) {
          goalsLeft++;
          foundGoals[i] = false;
        }
      }

      // keep a map of goal specifications to states that achieve them
      // optimally. Required to support multiple distinct states achieving goal
      std::unordered_map<State, std::vector<Node*>, StateHash> goalSpecToConcrete{};

      Cost finalFLayer = std::numeric_limits<Cost>::max();

      // set up counter for termination checking
      // Since this function could be expensive, we need to term check within it
      size_t localTimeoutCounter = 0;

      // loop through until we've found all paths to all goals
      while (openList.isNotEmpty()) {
        if(terminationChecker.has_value() && ++localTimeoutCounter % 10 == 0) {
          if (terminationChecker->reachedTermination()) {
            throw MetronomeTimeoutException();
          }
        }

        Node* top = openList.pop();
        // we've found every path to every goal
        if (top->f() > finalFLayer) {
          break;
        }

        if (domain->isGoal(top->state)){
          // finding goal from list
          // Searching through list to support underspecified goals
          size_t goalIdx = goals.size();
          for (size_t i = 0; i < goals.size(); i++) {
            if (domain->isGoal(top->state, goals[i])) {
              goalIdx = i;
            }
          }

          const State& goal = goals[goalIdx];

          // if we haven't already found this goal,
          // set it to found, set its optimal cost
          if (!foundGoals[goalIdx]) {
            foundGoals[goalIdx] = true;
            assert(goalsLeft > 0);
            goalsLeft--;

            goalSpecToConcrete[goal] = {};
            goalsToOptimalCost.insert({goal, top->g});
            if (top->g == 0) {
              throw MetronomeException("Invalid AGRD problem: A possible goal is the start state.");
            }
          }

          // it is possible we are seeing this goal again later, i.e. with
          // higher g cost. Do not add it to list of optimal states achieving
          // goal
          // Additionally check that this goal is relevant right now. (Might
          // not be since we could have already pruned it from hypothesis at this branch)
          if (goalsToOptimalCost.count(goal) > 0 &&
              top->g == goalsToOptimalCost.at(goal)) {
            auto& concreteNodes = goalSpecToConcrete[goal];
            concreteNodes.push_back(top);
          }

          // If this is the last goal, set the final F-layer
          if (goalsLeft == 0) {
            finalFLayer = top->f();
          }
        }
        top->open = false;

        // Even if it is a goal, must continue to expand.
        // Path to another goal may exclusively pass through this goal
        expandNode(top);
      }

      if (goalsLeft != 0) {
        throw RecomputeOptimalException("Not all goals discovered from subject's current state. Dead end! (follow-up)");
      }

      openList.clear();
      openList.reorder(&gMaxComparator<Node>);

      // seed reverse open list with all found goals
      for (auto& concreteGoalEntry : goalSpecToConcrete) {
        for (Node* concreteGoal : concreteGoalEntry.second) {
          concreteGoal->goalBackupIteration = recomputeOptimalIteration;
          // seed the goalsToPlans map with this goal and count 1
          concreteGoal->goalsToPlanCount = {{concreteGoalEntry.first, 1}};
          openList.push(*concreteGoal);
          concreteGoal->open = true;
        }
      }

      // Loop until open is fully empty
      int64_t backupCount = 0;
      while (openList.isNotEmpty()) {
        if(terminationChecker.has_value() && ++localTimeoutCounter % 10 == 0) {
          if (terminationChecker->reachedTermination()) {
            throw MetronomeTimeoutException();
          }
        }

        Node* top = openList.pop();

        backupGoalHypothesis(top);
        top->open = false;
        backupCount++;
      }
      Base::recordAttribute("backups", backupCount);

      // clear open list - we no longer need it, so ensure no bugs slip in for later uses of it
      openList.clear();
    }

    /**
     * If node is a leaf, get its value. Otherwise return empty optional.
     * Note that for iterative deepening, a node is a leaf if only 1 goal
     * meets the threshold
     * @param simulatedStateNode
     * @return
     */
    std::optional<double> getValueIfLeaf(Node* simulatedStateNode) {
      std::vector<typename std::unordered_map<State, double, StateHash>::value_type> possibleGoals{};
      // Filter out map entries to just those with > 0 probability
      // Iterative deepening filters to just those that have > threshold probability
      std::copy_if(goalsToPriors.cbegin(),
                   goalsToPriors.cend(),
                   std::back_inserter(possibleGoals),
                   [](const auto& mapVal) {
                     return mapVal.second > 0.0;
                   });

      if (possibleGoals.size() == 1) {
        auto goalState = possibleGoals[0].first;
        auto g = static_cast<double>(simulatedStateNode->g);
        if (goalsToOptimalCost.count(goalState) == 0) {
          throw MetronomeException("Bug in goal plan count (follow-up)");
        }
        auto cStar = static_cast<double>(goalsToOptimalCost.at(goalState));

        switch(objective) {
          case Metric::REACTION_TIME:
            return {cStar - g};
          case Metric::FRACTION_OF_OPTIMAL:
            return {g / cStar};
          case Metric::DISTINCTIVENESS:
            return {g};
        }
      } else if (possibleGoals.size() == 0) {
        throw MetronomeException("Bug in search - goals to plan count had no goals with plans (follow-up)");
      }

      return {};
    }

    // Recursion - all going on the stack. Might become stack overflow - watch for this

    /**
     *
     * @param simulatedStateNode
     * @param baseNode Recompute optimal plan costs from this node
     * when reversing interventions. Required to ensure successors from subject
     * actions are appropriately repaired. Should be the parent of the
     * simulatedStateNode
     * @return TODO Should return optional
     */
    InterventionTrialResult interventionTrial(
            Node* simulatedStateNode,
        Node* baseNode) {
      const auto nodeValue = getValueIfLeaf(simulatedStateNode);
      if (nodeValue) {
        return {nodeValue.value(), {}, {}};
      }

      // only check every 10 nodes of the DFS
      if(terminationChecker.has_value() && ++timeoutCounter % 10 == 0) {
        if (terminationChecker->reachedTermination()) {
          throw MetronomeTimeoutException();
        }
      }

      ++depth;
      if (depth > maxDepth) {
        throw MetronomeException("Depth limit reached! Aborting search (follow-up)");
      }

      InterventionTrialResult trialResult{
          (optimizeMin ? 1.0 : -1.0) * std::numeric_limits<double>::infinity(), {}, {}
      };

      // Caching information so that we can restore it at the end of each trial

      // This is set every time we call recompute optimal info, but if we
      // don't invoke that method then it could still be out of sync
      auto goalsToOptimalCostRestore = goalsToOptimalCost;
      // cache g and goals so that we can restore it after each trial
      Cost baseGRestore = baseNode->g;
      auto baseCurrentGoalsToPlanCountRestore = baseNode->goalsToPlanCount;

      std::vector<InterventionBundle<Domain>> interventions;
      if (identityTrial) {
        // for identity trials, we are only simulating subject actions
        interventions.push_back(Base::getIdentityIntervention(*domain));
      } else {
        // get states relevant to the subject state
        std::unordered_set<State, StateHash> optimalPlanStates {simulatedStateNode->state};
        std::queue<Node*> queue {{simulatedStateNode}};
        while (!queue.empty()) {
          Node* next = queue.front();
          for (Edge& edge : next->successors) {
            // grab successor node from edge
            Node* succ = edge.to;
            if (succ->goalBackupIteration == next->goalBackupIteration
                && succ->g > next->g
                && optimalPlanStates.insert(succ->state).second) {
              queue.push(succ);
            }
          }

          queue.pop();
        }

        interventions = domain->interventions(
            simulatedStateNode->state,
            {optimalPlanStates.begin(), optimalPlanStates.end()});
      }

      for (InterventionBundle interventionBundle : interventions) {
        // Set identity trial flag
        bool identityTrialStart = false;
        if (interventionBundle.intervention == domain->getIdentityIntervention()) {
//          identityTrial = true;
//          identityTrialStart = true;
        }

        std::optional<Patch> optionalDomainPatch =
                domain->applyIntervention(interventionBundle.intervention, simulatedStateNode->state);
        // invalid intervention - we will not consider it
        if (!optionalDomainPatch.has_value()) continue;
        // grab patch
        const Patch& domainPatch = optionalDomainPatch.value();


        // repair optimal info, but only if no subject states were actually affected
        // This could happen w/ identity intervention or interventions that only
        // affect observer state
        if (domainPatch.affectedStates.size() > 0) {
          // copy goals to optimal cost map so we can verify they don't change
          std::unordered_map<State, Cost, StateHash> goalsToOptimalCostCopy(goalsToOptimalCost);
          std::unordered_map<State, size_t , StateHash> goalHypothesisCopy(simulatedStateNode->goalsToPlanCount);
          bool invalidIntervention = false;
          try {
            recomputeOptimalInfo(simulatedStateNode->state);
          } catch (RecomputeOptimalException& ex) {
            // made a goal unreachable
            invalidIntervention = true;
          }

          // detect change in optimal cost for any goal in the goal hypothesis
          for (auto& entry : goalsToOptimalCostCopy) {
            Cost originalCost = entry.second;
            Cost newCost = goalsToOptimalCost[entry.first];

            if (goalHypothesisCopy.count(entry.first) > 0 && originalCost != newCost) {
              invalidIntervention = true;
              break;
            }
          }
          // detect if the simulated state is no longer on an optimal path
          // or if goals have been removed from simulated state.
          // If either, this intervention is no good in this trial
          if (simulatedStateNode->goalBackupIteration != recomputeOptimalIteration
              || goalHypothesisCopy.size() != simulatedStateNode->goalsToPlanCount.size()) {
            invalidIntervention = true;
          }

          // when intervention deemed invalid, reverse and continue
          if (invalidIntervention) {
            domain->reversePatch(domainPatch, simulatedStateNode->state);
            // reset g and goals to plan count for recomputing optimal plans
            baseNode->g = baseGRestore;
            baseNode->goalsToPlanCount =
                baseCurrentGoalsToPlanCountRestore;
            try {
              recomputeOptimalInfo(baseNode->state);
            } catch (RecomputeOptimalException& rex) {
              // giving me a breakpoint for debug
              throw rex;
            }
            // restore in case it is different
            goalsToOptimalCost = goalsToOptimalCostRestore;
            continue;
          }
        }


        // descend
        ActionTrialResult actionTrialResult = actionTrial(
            simulatedStateNode);

#if NAIVEOPTIMALACTIVEGOALRECOGNITIONDESIGN_DEBUG_TRACE
        if (depth <= NAIVEOPTIMALACTIVEGOALRECOGNITIONDESIGN_LOG_TO_DEPTH) {
          LOG(DEBUG) << pad(depth) << "IT "
                     << interventionBundle.intervention
                     << ": " << actionTrialResult.score;
//          // condition here so getNode compiles
//          if (depth == 1000000) {
//            LOG(INFO) << getNode(0, 0).toString();
//          }
        }
#endif

        // different predicate depending on if we're minimizing or maximizing
        constexpr auto trialIsBetter = optimizeMin ? [](const double branchScore, const double currentBest) -> bool {
          return branchScore < currentBest;
        } : [](const double branchScore, const double currentBest) -> bool {
          return branchScore > currentBest;
        };

        if (trialIsBetter(actionTrialResult.score, trialResult.score) or
            // prefer identity interventions when there is a tie
            (trialResult.score - actionTrialResult.score < std::numeric_limits<double>::epsilon()
             and interventionBundle.intervention == domain->getIdentityIntervention())) {
          trialResult.score = actionTrialResult.score;
          trialResult.bestIntervention.emplace(interventionBundle);
          trialResult.potentialSubjectActionOutcomes =
              std::move(actionTrialResult.potentialSubjectActionOutcomes);
        }

        // reverse patch, repair optimal info
        domain->reversePatch(domainPatch, simulatedStateNode->state);
        // reset g and goals to plan count for simulated state.
        // could be affected by searches deeper in the tree, so this needs
        // to be reset after every trial
        baseNode->g = baseGRestore;
        baseNode->goalsToPlanCount =
            baseCurrentGoalsToPlanCountRestore;

        // always recompute :(
        // doesn't work otherwise, there are too many edge cases
        try {
          recomputeOptimalInfo(baseNode->state);
        } catch (RecomputeOptimalException& rex) {
          // breakpoint for debug
          throw rex;
        }
        goalsToOptimalCost = goalsToOptimalCostRestore;
        if (identityTrialStart) {
          identityTrial = false;
        }
      }

      --depth;
      return trialResult;
    }

    ActionTrialResult actionTrial(Node* simulatedStateNode) {
      // only check every 10 nodes of the DFS
      if(terminationChecker.has_value() && ++timeoutCounter % 10 == 0) {
        if (terminationChecker->reachedTermination()) {
          throw MetronomeTimeoutException();
        }
      }

      depth++;

      ActionTrialResult trialResult {
          0.0,
          computeActionProbabilities(simulatedStateNode, goalsToPriors)
      };

      // copy goals to priors before to modifying
      std::unordered_map<State, double, StateHash> goalsToPriorsBak(goalsToPriors);

      for (const ActionProbability& actionResults : trialResult.potentialSubjectActionOutcomes) {
        Node* successor = actionResults.successor;
        double probability = actionResults.probabilityOfAction;
        // can happen with iterative widening algorithm
        if (probability == 0.0) {
          continue;
        }

        double actionScore = 0.0;
        [[maybe_unused]] bool noOp = false;
        // this means that the state is a goal state
        if (successor->state == simulatedStateNode->state) {
          noOp = true;
        } else {
          // copy-assign
          goalsToPriors = actionResults.conditionedGoalPriors;
          actionScore = interventionTrial(successor, simulatedStateNode).score;
        }

#if NAIVEOPTIMALACTIVEGOALRECOGNITIONDESIGN_DEBUG_TRACE
        if (depth <= NAIVEOPTIMALACTIVEGOALRECOGNITIONDESIGN_LOG_TO_DEPTH) {
          LOG(DEBUG) << pad(depth) << "AT "
                     << (noOp ? "NO-OP - subject reached goal " : "")
                     << successor->state
                     << ": " << actionScore
                     << " (Prob " << probability
                     << "; Action Score: " << actionScore
                     << ")";
#if NAIVEOPTIMALACTIVEGOALRECOGNITIONDESIGN_DEBUG_TRACE > 1
          std::stringstream sstream{};
          for (auto& entry : actionResults.conditionedGoalPriors) {
            sstream << entry.first << " - " << entry.second << "; ";
          }

          LOG(DEBUG) << "_" << pad(depth) << "Goal Priors at this level: " << sstream.str();
#endif
        }
#endif
        trialResult.score += probability * actionScore;
      }

      // restore previous goals-to-priors
      goalsToPriors = std::move(goalsToPriorsBak);

      depth--;
      return trialResult;
    }

    /**
     * Computes probability of every valid successor of the simulated state node.
     * Returns map of successor state to its probability
     * @param simulatedStateNode
     * @param conditionedGoalsToPriors Map of goals to probabilities. Probability
     * is expected to be conditioned on the sequence of actions the subject has
     * taken thus far (in the simulation)
     * @return Each element represents an action an optimal subject could take
     * represented by its successor. Also included are bayesian updated goal posteriors
     * assuming that action is taken and the probability of the action.
     * When iterativeWidening is true, this probability could be returned as 0.0.
     * We do not want to prune it from the vector, however, because we need the
     * goal posterior if the agent does take that action.
     */
    std::vector<ActionProbability> computeActionProbabilities(
        Node* simulatedStateNode,
        std::unordered_map<State, double, StateHash>& conditionedGoalsToPriors) {
      std::vector<ActionProbability> actionResults{};

      // special check for if this state is a goal - we say the subject
      // will stay put and we assign the goal's probability to it
      // This might happen if plans to other goals could pass through it
      if (domain->isGoal(simulatedStateNode->state)) {
        ActionProbability actionResult{};
        actionResult.successor = simulatedStateNode;

        // need this found check in case of underspecified goals.
        // If a large number of states match goal condition, but we've already
        // eliminated that goal, this would cause problems
        bool found = false;
        for (auto& entry : conditionedGoalsToPriors) {
          // Sum probability of all goals this state satisfies
          // That sum is the probability of the "action" (i.e. stay put)
          if (domain->isGoal(simulatedStateNode->state, entry.first)
              and entry.second > 0.0) {
            found = true;

            actionResult.conditionedGoalPriors[entry.first] = entry.second;
            actionResult.probabilityOfAction += entry.second;
          } else {
            actionResult.conditionedGoalPriors[entry.first] = 0.0;
          }
        }

        if (found) {
          assert(actionResult.probabilityOfAction > 0.0);
          for (auto& entry : conditionedGoalsToPriors) {
            actionResult.conditionedGoalPriors[entry.first] /= actionResult.probabilityOfAction;
          }
          actionResults.push_back(std::move(actionResult));
        }
      }

      // for each action, but we only care about the successors
      for (const auto& successorEdge : simulatedStateNode->successors) {
        Node* successorNode = successorEdge.to;

        // if g does not increase, the subject would not
        // transition from the simulated state to this successor via an optimal plan
        if (successorNode->g <= simulatedStateNode->g) continue;

        // this means the successor is not on any optimal plan, so we can skip
        if (simulatedStateNode->goalBackupIteration !=
            successorNode->goalBackupIteration) {
          continue;
        }

        actionResults.emplace_back();
        ActionProbability& actionResult = actionResults.back();
        actionResult.successor = successorNode;
        // initialize conditioned goal priors to 0.0
        for (auto& entry : conditionedGoalsToPriors) {
          actionResult.conditionedGoalPriors[entry.first] = 0.0;
        }

        actionResult.probabilityOfAction = 0.0;
        // cycle through goal hypothesis of successor
        for (auto& hypothesisEntry : actionResult.successor->goalsToPlanCount) {
          // sanity checks
          assert(actionResult.conditionedGoalPriors.count(hypothesisEntry.first) == 1);
          assert(conditionedGoalsToPriors.count(hypothesisEntry.first) == 1);

          if (simulatedStateNode->goalsToPlanCount[hypothesisEntry.first] == 0) {
            continue; // should be pruned
          }

          /** calculates the fraction of plans to this goal that pass through this action */
          double fractionOfPlansForGoal =
              (double)hypothesisEntry.second /
              (double)simulatedStateNode->goalsToPlanCount[hypothesisEntry.first];

          // probability of the considered goal conditioned on the agent taking this action
          double conditionedProbability =
              fractionOfPlansForGoal * conditionedGoalsToPriors[hypothesisEntry.first];

          actionResult.conditionedGoalPriors[hypothesisEntry.first] = conditionedProbability;
          actionResult.probabilityOfAction += conditionedProbability;
        }

        // Finally, normalize goal priors based on on the sum of conditioned
        // probabilities
        if (actionResult.probabilityOfAction > 0.0) {
          for (auto& entry : conditionedGoalsToPriors) {
            actionResult.conditionedGoalPriors[entry.first] /= actionResult.probabilityOfAction;
          }
        }
      }
      // only does anything if iterative widening is turned on
      setActionProbabilitiesIterativeWidening(actionResults);

      return actionResults;
    }

    /**
     * In iterative widening, re-sets action probabilities based on topNProbabilities
     * @param actionResults
     */
    void setActionProbabilitiesIterativeWidening(std::vector<ActionProbability>& actionResults) {
      // can return if topNActionProbabilty is greater than vector because we
      // will not change probabilities in that case
      if (not iterativeWidening or topNActionProbabilty >= actionResults.size()) return;

      std::vector<double> uniqueProbabilities{};
      uniqueProbabilities.reserve(actionResults.size());
      std::transform(actionResults.begin(), actionResults.end(),
                     std::back_inserter(uniqueProbabilities),
                     [](const auto& actionProb) { return actionProb.probabilityOfAction; });
      std::sort(uniqueProbabilities.begin(), uniqueProbabilities.end());
      auto last = std::unique(uniqueProbabilities.begin(), uniqueProbabilities.end());
      uniqueProbabilities.erase(last, uniqueProbabilities.end());

      // again, nothing we can do - the lowest likelihood will be the threshold
      if (topNActionProbabilty > uniqueProbabilities.size()) return;

      double threshold = uniqueProbabilities[topNActionProbabilty - 1];
      double normalizingSum = 0.0;
      std::for_each(actionResults.begin(), actionResults.end(),
                    [&](ActionProbability& actionProbability) {
                      if (actionProbability.probabilityOfAction < threshold) {
                        actionProbability.probabilityOfAction = 0.0;
                      } else {
                        normalizingSum += actionProbability.probabilityOfAction;
                      }
                    });
      std::for_each(actionResults.begin, actionResults.end(),
                    [&](ActionProbability& actionProbability) {
                      actionProbability.probabilityOfAction /= normalizingSum;
                    });
    }

    /*****************************************
            Utility / Helper methods
    *****************************************/



    /**
     * Creates a node if it doesn't already exist. Updates parent / successor sets.
     * @param domain
     * @param parent
     * @param successorState
     * @return
     */
    Node* getSuccessor(Node* parent, const State& successorState,
                       Cost actionCost = 0, bool resetG = true) {
      Node* tempNode = nodes[successorState];

      if (tempNode == nullptr) {
        Planner::incrementGeneratedNodeCount();
        tempNode = nodePool.construct(
                successorState,
                std::numeric_limits<Cost>::max(),
                getMinHeuristic(successorState),
                recomputeOptimalIteration);

        nodes[successorState] = tempNode;
      }

      // Node is outdated
      // must do this here so we can reset successors / predecessors
      if (tempNode->iteration != recomputeOptimalIteration) {
        tempNode->iteration = recomputeOptimalIteration;
        if (resetG) {
          tempNode->g = std::numeric_limits<Cost>::max();
        }
        tempNode->h = getMinHeuristic(successorState);
        tempNode->open = false;
        tempNode->parents.clear();
        tempNode->successors.clear();
      }

      // if not a root we set edge info
      if (parent != nullptr) {
        Edge edge{parent, tempNode, actionCost};

        tempNode->parents.push_back(edge);
        parent->successors.push_back(edge);
      }

      return tempNode;
    }

    /**
     * Of all possible goals gets the minimum heuristic for a given state
     * @param state
     * @param domain
     * @return
     */
    Cost getMinHeuristic(const State& state) {
      Cost minH = std::numeric_limits<Cost>::max();
      for (const auto& goal : domain->getGoals()) {
        minH = std::min(minH, domain->heuristic(state, goal));
      }

      return minH;
    }

    /**
     * Basically a standard A* node expansion
     *
     * @param source
     */
    void expandNode(Node* source) {
      Planner::incrementExpandedNodeCount();

      for (SuccessorBundle<Domain> successor : domain->successors(source->state)) {
        // might record new parent, if applicable
        Node* successorNode = getSuccessor(source, successor.state, successor.actionCost);

        Cost successorPotentialG = source->g + successor.actionCost;
        if (successorNode->g > successorPotentialG) {
          successorNode->g = successorPotentialG;

          if (!successorNode->open) {
            successorNode->open = true;
            openList.push(*successorNode);
          } else {
            openList.update(*successorNode);
          }
        }
      }
    }

    /**
     * Backing up goal hypothesis to predecessors along with plan counts
     * @param source
     */
    void backupGoalHypothesis(Node* source) {
      for (Edge predecessorEdge : source->parents) {
        Node* predecessor = predecessorEdge.from;
        // skip if g is not monotonically decreasing
        // Also, if we did not touch predecessor this iteration we'll skip it
        if (predecessor->g >= source->g || predecessor->iteration != source->iteration) continue;

        // If we haven't seen this node this backup iteration, reset its goal hypothesis
        if (predecessor->goalBackupIteration != source->goalBackupIteration) {
          predecessor->goalBackupIteration = source->goalBackupIteration;
          predecessor->goalsToPlanCount = {};
          predecessor->open = false;
        }

        assert(source->goalsToPlanCount.size() > 0);
        for (const auto& entry : source->goalsToPlanCount) {
          // Try emplace does nothing if key already exists
          predecessor->goalsToPlanCount.try_emplace(entry.first, 0);
          predecessor->goalsToPlanCount[entry.first] += entry.second;
        }

        if (!predecessor->open) {
          openList.push(*predecessor);
          predecessor->open = true;
        }
      }
    }

    /*****************************************
            Log Helpers
    *****************************************/
    std::string pad(uint64_t currentDepth) {
      std::ostringstream str;
      for (uint64_t i = 0; i < currentDepth; i++) {
        str << "  ";
      }
      str << "d" << currentDepth << " ";
      return str.str();
    }

    /*****************************************
            Properties
    *****************************************/

    PriorityQueue<Node> openList;
    std::unordered_map<State, Node*, StateHash> nodes;
    ObjectPool<Node, Memory::NODE_LIMIT> nodePool;
    std::unordered_map<State, double, StateHash> goalsToPriors;
    std::optional<State> rootState;
    uint32_t maxDepth{0};
    // check for timeouts
    std::optional<TimeTerminationChecker> terminationChecker{};
    uint64_t timeoutCounter{0};

    // Fields that reset after each iteration
    std::unordered_map<State, Cost, StateHash> goalsToOptimalCost{};
    uint64_t recomputeOptimalIteration{0};
    uint32_t depth{0};

    /**
     * If true (controlled by config), algorithm uses an iterative widening strategy.
     * Branches are ignored if probabilities of actions do not meet a minimum threshold
     * based on top n likelihood values.
     * In subsequent iterations, the threshold is reduced by increasing n.
     * We continue to iterate until time runs out. Iteration time is controlled
     * by "action duration" config
     */
    bool iterativeWidening;
    size_t topNActionProbabilty{1};
    int64_t planningIterationTimeLimit{0};
    /**
     * Vector holds predicted outcomes given an intervention.
     * We will use this to update goal priors based on the action the subject
     * took
     */
    std::vector<ActionProbability> potentialOutcomes{};
    /**
     * Flag is set when an identity action is trialed.
     * Signals that a trial is seeing what happens if no more interventions are executed
     */
    bool identityTrial = false;

    Domain* domain = nullptr;
  };
} // namespace metronome

#endif //METRONOME_NAIVEOPTIMALACTIVEGOALRECOGNITIONDESIGN_HPP
