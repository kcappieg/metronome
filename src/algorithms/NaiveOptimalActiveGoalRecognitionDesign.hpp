//
// Created by kevin on 5/14/20.
//

#ifndef METRONOME_NAIVEOPTIMALACTIVEGOALRECOGNITIONDESIGN_HPP
#define METRONOME_NAIVEOPTIMALACTIVEGOALRECOGNITIONDESIGN_HPP

#include <cassert>
#include <limits>
#include <sstream>
#include <string>
#include <unordered_map>

#include <experiment/Configuration.hpp>
#include "GoalRecognitionDesignPlanner.hpp"
#include "Planner.hpp"
#include "utils/Hash.hpp"
#include "utils/ObjectPool.hpp"
#include <MemoryConfiguration.hpp>
#include <unordered_set>
#include <queue>
#include "utils/PriorityQueue.hpp"
#include "planner_tools/Comparators.hpp"
#include "domains/SuccessorBundle.hpp"

#define NAIVEOPTIMALACTIVEGOALRECOGNITIONDESIGN_DEBUG_TRACE 1
#define NAIVEOPTIMALACTIVEGOALRECOGNITIONDESIGN_LOG_TO_DEPTH 3

namespace metronome {
  template<typename Domain>
  class NaiveOptimalActiveGoalRecognitionDesign final : public GoalRecognitionDesignPlanner<Domain> {
  public:
    using State = typename Domain::State;
    using StateHash = typename metronome::Hash<State>;
    using Action = typename Domain::Action;
    using Intervention = typename Domain::Intervention;
    using Cost = typename Domain::Cost;
    using Planner = metronome::Planner<Domain>;
    using Base = metronome::GoalRecognitionDesignPlanner<Domain>;
    using Patch = typename Domain::Patch;

    NaiveOptimalActiveGoalRecognitionDesign(const Domain& domain, const Configuration& config)
        : openList(Memory::OPEN_LIST_SIZE, &fComparator<Node>),
          maxDepth(config.getLong(MAX_DEPTH, std::numeric_limits<uint32_t>::max())) {
      if (config.hasMember(GOAL_PRIORS)) {
        std::vector<double> goalPriors = config.getDoubles(GOAL_PRIORS);

        size_t index = 0;
        for (const State goal : domain.getGoals()) {
          goalsToPriors[goal] = goalPriors[index++];
        }
      } else {
        throw MetronomeException("No goal priors specified");
      }
    }

    std::vector<InterventionBundle<Domain>> selectInterventions(
            const State& subjectState, const Domain& systemState
    ) override {
      GoalRecognitionDesignPlanner<Domain>::beginIteration();
      // copy domain, set accessible pointer to it
      Domain localDomain(systemState);
      domain = &localDomain;
      depth = 0;
      if (!rootState.has_value()) {
        *rootState = domain->getStartState();
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

      recomputeOptimalInfo(*rootState);

      InterventionTrialResult trialResult = interventionTrial(
          nodes[subjectState], *rootState);
      std::optional<InterventionBundle<Domain>> intervention = trialResult.bestIntervention;
      potentialOutcomes = trialResult.potentialSubjectActionOutcomes;

      LOG(DEBUG) << "Intervention score: " << trialResult.score;

      domain = nullptr;

      depth = 0;
      Base::recordAttribute("numOptimalRecomputations", recomputeOptimalIteration - recomputeOptimalBegin);

      if (intervention.has_value()) return {*intervention};
      return {};
    }

    std::optional<State> getGoalPrediction(const Domain&,
                            const State& subjectState) override {
      Node* subjectStateNode = nodes[subjectState];
      if (subjectStateNode->goalsToPlanCount.size() == 1) {
        return {subjectStateNode->goalsToPlanCount.begin()->first};
      }

      return {};
    }

  private:
    struct Edge;
    // Search Node
    struct Node {
    public:
      Node(const State& state, Cost g, Cost h, unsigned int iteration)
              : state(state), g(g), h(h), iteration{iteration} {
        goalBackupIteration = 0;
      }

      size_t hash() const { return state.hash(); }
      bool operator==(const Node& node) const { return state == node.state; }

      Cost f() const { return g + h; }

      /** Index used by the priority queue */
      mutable unsigned int index{std::numeric_limits<unsigned int>::max()};

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

      std::string toString() {
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
      Node* successor;
      double probabilityOfAction;
      /** Goal priors conditioned on this action being taken */
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

    // debug methods
    Node getNode(uint64_t x, uint64_t y) {
      return *nodes[State(x, y)];
    }

    /*****************************************
            Algorithm methods
            Methods with formal definitions
    *****************************************/

    /**
     * Enumerate all optimal plans to all goals so that we can configure goal hypotheses and plan counts.
     * Basically A* except we don't stop until we've exhausted the full f-layer of all goals
     *
     * WARNING: This method does not work for underspecified goals! It depends on the goal state reached
     * being a fully qualified and unique state.
     * @param root
     */
    void recomputeOptimalInfo(const State& root) {
      // reset open list, goal info, optimal plan states
      openList.clear();
      openList.reorder(&fComparator<Node>);
      goalsToOptimalCost.clear();
      recomputeOptimalIteration++;

      Node* rootNode = nodes[root];
      if (rootNode == nullptr) {
        rootNode = createSuccessor(nullptr, root);
      }

      rootNode->g = 0;
      rootNode->iteration = recomputeOptimalIteration;

      openList.push(*rootNode);

      // set up local set for goals. We will tick them off when we expand them
      std::unordered_set<Node*> foundGoals{};
      size_t numGoals = goalsToPriors.size();

      Cost finalFLayer = std::numeric_limits<Cost>::max();

      // loop through until we've found all paths to all goals
      while (openList.isNotEmpty()) {
        Node* top = openList.pop();
        // we've found every path to every goal
        if (top->f() > finalFLayer) {
          break;
        }

        // If this is the last goal, set the final F-layer var.
        if (domain->isGoal(top->state)
            && foundGoals.insert(top).second) {
          if (foundGoals.size() == numGoals) {
            finalFLayer = top->f();
          }

          goalsToOptimalCost.insert({top->state, top->g});
        }
        top->open = false;

        // Even if it is a goal, must continue to expand.
        // Path to another goal may exclusively pass through this goal
        expandNode(top);
      }

      if (foundGoals.size() == 0) {
        throw MetronomeException("No goals discovered from subject's current state. Dead end!");
      } else if (foundGoals.size() > numGoals) {
        throw MetronomeException("Underspecified goals not implemented in recompute optimal function");
      }

      openList.clear();
      openList.reorder(&gMaxComparator<Node>);

      // seed reverse open list with all found goals
      for (Node* goal : foundGoals) {
        goal->goalBackupIteration = recomputeOptimalIteration;
        // seed the goalsToPlans map with this goal and count 1
        goal->goalsToPlanCount = {{goal->state, 1}};
        openList.push(*goal);
        goal->open = true;
      }

      // Loop until open is fully empty
      int64_t backupCount = 0;
      while (openList.isNotEmpty()) {
        Node* top = openList.pop();

        backupGoalHypothesis(top);
        top->open = false;
        backupCount++;
      }
      Base::recordAttribute("backups", backupCount);

      // clear open list - we no longer need it, so ensure no bugs slip in for later uses of it
      openList.clear();
    }

    // Recursion - all going on the stack. Might become stack overflow - watch for this

    InterventionTrialResult interventionTrial(
            Node* simulatedStateNode, const State& rootState) {
      // sigma
      if (simulatedStateNode->goalsToPlanCount.size() == 1) {
        auto goalState = simulatedStateNode->goalsToPlanCount.cbegin()->first;
        double g = static_cast<double>(simulatedStateNode->g);
        double cStar = static_cast<double>(goalsToOptimalCost[goalState]);
        return {g / cStar, {}, {}};
      }

      InterventionTrialResult trialResult{1.0, {}, {}};

      bool depthLimitReached = false;
      if (!identityTrial && (depth / 2) + 1 > maxDepth) {
        depthLimitReached = true;
        identityTrial = true;
      }
      depth++;

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

        interventions = domain->interventions({optimalPlanStates.begin(), optimalPlanStates.end()});
      }

      for (InterventionBundle interventionBundle : interventions) {
        // Set identity trial flag
        bool identityTrialStart = false;
        if (interventionBundle.intervention == domain->getIdentityIntervention()) {
          identityTrial = true;
          identityTrialStart = true;
        }

        std::optional<Patch> optionalDomainPatch =
                domain->applyIntervention(interventionBundle.intervention, simulatedStateNode->state);
        // invalid intervention - we will not consider it
        if (!optionalDomainPatch.has_value()) continue;
        // grab patch
        const Patch& domainPatch = optionalDomainPatch.value();

        // repair optimal info, but only if it is not an identity intervention
        if (!identityTrial) {
          // copy goals to optimal cost map so we can verify they don't change
          std::unordered_map<State, Cost, StateHash> goalsToOptimalCostCopy(goalsToOptimalCost);
          std::unordered_map<State, size_t , StateHash> goalHypothesisCopy(simulatedStateNode->goalsToPlanCount);
          recomputeOptimalInfo(rootState);

          // detect change in optimal cost for any goal
          bool invalidIntervention = false;
          for (auto& entry : goalsToOptimalCostCopy) {
            Cost originalCost = entry.second;
            Cost newCost = goalsToOptimalCost[entry.first];

            if (originalCost != newCost) {
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
            recomputeOptimalInfo(rootState);
            continue;
          }
        }


        // descend
        ActionTrialResult actionTrialResult = actionTrial(simulatedStateNode, rootState);

#if NAIVEOPTIMALACTIVEGOALRECOGNITIONDESIGN_DEBUG_TRACE
        if (depth <= NAIVEOPTIMALACTIVEGOALRECOGNITIONDESIGN_LOG_TO_DEPTH) {
          LOG(DEBUG) << pad(depth) << "IT "
                     << interventionBundle.intervention
                     << ": " << actionTrialResult.score;
          // condition here so getNode compiles
          if (depth == 1000000) {
            LOG(INFO) << getNode(0, 0).toString();
          }
        }
#endif

        if (actionTrialResult.score < trialResult.score) {
          trialResult.score = actionTrialResult.score;
          trialResult.bestIntervention.emplace(interventionBundle);
          trialResult.potentialSubjectActionOutcomes =
              std::move(actionTrialResult.potentialSubjectActionOutcomes);
        }

        // reverse patch, repair optimal info
        domain->reversePatch(domainPatch, simulatedStateNode->state);
        if (!identityTrial) {
          recomputeOptimalInfo(rootState);
        }
        if (identityTrialStart) {
          identityTrial = false;
        }
      }

      // reset identity trial flag
      if (depthLimitReached) {
        identityTrial = false;
      }
      depth--;
      return trialResult;
    }

    ActionTrialResult actionTrial(Node* simulatedStateNode, const State& rootState) {
      // sigma
      if (simulatedStateNode->goalsToPlanCount.size() == 1) {
        auto goalState = simulatedStateNode->goalsToPlanCount.cbegin()->first;
        double g = static_cast<double>(simulatedStateNode->g);
        double cStar = static_cast<double>(goalsToOptimalCost[goalState]);
        return {g / cStar, {}};
      }
      depth++;

      ActionTrialResult trialResult {
          0.0,
          computeActionProbabilities(simulatedStateNode, goalsToPriors)
      };

      // copy goals to priors prior to modifying
      std::unordered_map<State, double, StateHash> goalsToPriorsBak(goalsToPriors);

      for (const ActionProbability& actionResults : trialResult.potentialSubjectActionOutcomes) {
        Node* successor = actionResults.successor;
        double probability = actionResults.probabilityOfAction;

        double actionScore = 1.0;
        bool noOp = false;
        // this means that the state is a goal state
        if (successor->state == simulatedStateNode->state) {
          noOp = true;
        } else {
          // copy-assign
          goalsToPriors = actionResults.conditionedGoalPriors;
          actionScore = interventionTrial(successor, rootState).score;
        }

#if NAIVEOPTIMALACTIVEGOALRECOGNITIONDESIGN_DEBUG_TRACE
        if (depth <= NAIVEOPTIMALACTIVEGOALRECOGNITIONDESIGN_LOG_TO_DEPTH) {
          LOG(DEBUG) << pad(depth) << "AT "
                     << (noOp ? "NO-OP - subject reached goal " : "")
                     << successor->state
                     << ": " << actionScore
                     << " (Prob " << probability << ")";
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
     * @return
     */
    std::vector<ActionProbability> computeActionProbabilities(
        Node* simulatedStateNode,
        std::unordered_map<State, double, StateHash>& conditionedGoalsToPriors) {

      std::vector<ActionProbability> actionResults{};

      // special check for if this state is a goal - we say the subject
      // will stay put and we assign the goals probability to it
      // This might happen if plans to other goals could pass through it
      if (domain->isGoal(simulatedStateNode->state)) {
        actionResults.emplace_back();
        ActionProbability& actionResult = actionResults.back();
        actionResult.successor = simulatedStateNode;

        for (auto& entry : conditionedGoalsToPriors) {
          // if this is the goal, it has probability one. The action has
          // probability equal to the prior
          if (domain->isGoal(simulatedStateNode->state, entry.first)) {
            actionResult.conditionedGoalPriors[entry.first] = 1.0;
            actionResult.probabilityOfAction = entry.second;
          } else {
            actionResult.conditionedGoalPriors[entry.first] = 0.0;
          }
        }
      }

      // for each action, but we only care about the successors
      for (Edge& successorEdge : simulatedStateNode->successors) {
        // if g does not increase, the subject would not
        // transition from the simulated state to this successor via an optimal plan
        if (successorEdge.to->g <= simulatedStateNode->g) continue;

        // this means the successor is not on any optimal plan, so we can skip
        if (simulatedStateNode->goalBackupIteration !=
            successorEdge.to->goalBackupIteration) {
          continue;
        }

        // this means the state is invalid (possibly due to prior intervention)
        if (!domain->isValidState(successorEdge.to->state)) continue;

        actionResults.emplace_back();
        ActionProbability& actionResult = actionResults.back();
        actionResult.successor = successorEdge.to;
        // initialize conditioned goal priors to 0.0
        for (auto& entry : conditionedGoalsToPriors) {
          actionResult.conditionedGoalPriors[entry.first] = 0.0;
        }

        actionResult.probabilityOfAction = 0.0;

        double adjustedSum = 0.0;
        // cycle through goal hypothesis of successor
        for (auto& hypothesisEntry : actionResult.successor->goalsToPlanCount) {
          // sanity checks
          assert(actionResult.conditionedGoalPriors.count(hypothesisEntry.first) == 1);
          assert(conditionedGoalsToPriors.count(hypothesisEntry.first) == 1);

          /** calculates the fraction of plans to this goal that pass through this action */
          double fractionOfPlansForGoal =
              (double)hypothesisEntry.second /
              (double)simulatedStateNode->goalsToPlanCount[hypothesisEntry.first];

          // Add to the probability of this action:
            // conditioned goal prior * fraction of plans from simulated
            // node to the goal that pass through successor
          actionResult.probabilityOfAction +=
              conditionedGoalsToPriors[hypothesisEntry.first] * fractionOfPlansForGoal;

          // probability of the considered goal conditioned on the agent taking this action
          double conditionedProbability =
              fractionOfPlansForGoal * conditionedGoalsToPriors[hypothesisEntry.first];

          actionResult.conditionedGoalPriors[hypothesisEntry.first] = conditionedProbability;
          adjustedSum += conditionedProbability;
        }

        // Finally, adjust goal priors based on on the sum of conditioned
        // probabilities
        if (adjustedSum > 0.0) {
          for (auto& entry : conditionedGoalsToPriors) {
            actionResult.conditionedGoalPriors[entry.first] /= adjustedSum;
          }
        }
      }

      return actionResults;
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
    Node* createSuccessor(Node* parent, const State& successorState, Cost actionCost = 0) {
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

      // if not a root, we set edge info
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
      for (const auto& entry : goalsToPriors) {
        minH = std::min(minH, domain->heuristic(state, entry.first));
      }

      return minH;
    }

    /**
     * Basically a standard A* node expansion
     * Note: Only supports removing edges, not adding edges
     * @param source
     */
    void expandNode(Node* source) {
      Planner::incrementExpandedNodeCount();

      if (source->successors.size() == 0) {
        for (SuccessorBundle<Domain> successor : domain->successors(source->state)) {
          createSuccessor(source, successor.state, successor.actionCost);
        }
      }

      for (Edge edge : source->successors) {
        if (!domain->isValidState(edge.to->state)) continue;

        Node* successorNode = edge.to;

        // Node is outdated
        if (successorNode->iteration != recomputeOptimalIteration) {
          successorNode->iteration = recomputeOptimalIteration;
          successorNode->g = std::numeric_limits<Cost>::max();
          successorNode->open = false;
        }

        Cost successorPotentialG = source->g + edge.actionCost;
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
    std::string pad(uint16_t depth) {
      std::ostringstream str;
      for (uint64_t i = 0; i < depth; i++) {
        str << "  ";
      }
      str << "d" << depth << " ";
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
    uint32_t maxDepth = 0;

    // Fields that reset after each iteration
    std::unordered_map<State, Cost, StateHash> goalsToOptimalCost{};
    uint64_t recomputeOptimalIteration = 0;
    uint32_t depth = 0;
    /**
     * Vector holds predicted outcomes given an intervention.
     * We will use this to update goal priors based on the action the subject
     * took
     */
    std::vector<ActionProbability> potentialOutcomes;
    /**
     * Flag is set when an identity action is trialed.
     * Signals that a trial is seeing what happens if no more interventions are executed
     */
    bool identityTrial = false;

    Domain* domain = nullptr;
  };
} // namespace metronome

#endif //METRONOME_NAIVEOPTIMALACTIVEGOALRECOGNITIONDESIGN_HPP
