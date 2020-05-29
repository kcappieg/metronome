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

      uint64_t recomputeOptimalBegin = recomputeOptimalIteration;

      recomputeOptimalInfo(*rootState);

      auto intervention = interventionTrial(nodes[subjectState], *rootState).second;
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

    std::pair<double, std::optional<InterventionBundle<Domain>>> interventionTrial(
            Node* simulatedStateNode, const State& rootState) {
      // sigma
      if (simulatedStateNode->goalsToPlanCount.size() == 1) {
        auto goalState = simulatedStateNode->goalsToPlanCount.cbegin()->first;
        double g = static_cast<double>(simulatedStateNode->g);
        double cStar = static_cast<double>(goalsToOptimalCost[goalState]);
        return {g / cStar, {}};
      }

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

      double score = 1.0;
      std::optional<InterventionBundle<Domain>> argminIntervention{};
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

          // Detecting if the intervention had any effect on the current simulated node.
          // If not, we can ignore this intervention as it does not affect this branch
          bool anyChange = false;
          for (auto& entry : goalHypothesisCopy) {
            if (entry.second != simulatedStateNode->goalsToPlanCount[entry.first]) {
              anyChange = true;
            }
          }
          if (!anyChange) invalidIntervention = true;

          // when intervention deemed invalid, reverse and continue
          if (invalidIntervention) {
            domain->reversePatch(domainPatch, simulatedStateNode->state);
            recomputeOptimalInfo(rootState);
            continue;
          }
        }


        // descend
        double trialScore = actionTrial(simulatedStateNode, rootState);

        #if NAIVEOPTIMALACTIVEGOALRECOGNITIONDESIGN_DEBUG_TRACE
        LOG(DEBUG) << pad(depth) << "IT " << interventionBundle.intervention << ": " << trialScore;
        // condition here so getNode compiles
        if (depth == 1000000) {
          LOG(INFO) << getNode(0, 0).toString();
        }
        #endif

        if (trialScore < score) {
          score = trialScore;
          argminIntervention.emplace(interventionBundle);
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
      return {score, argminIntervention};
    }

    double actionTrial(Node* simulatedStateNode, const State& rootState) {
      // sigma
      if (simulatedStateNode->goalsToPlanCount.size() == 1) {
        auto goalState = simulatedStateNode->goalsToPlanCount.cbegin()->first;
        return static_cast<double>(simulatedStateNode->g) / static_cast<double>(goalsToOptimalCost[goalState]);
      }
      depth++;

      std::unordered_map<Node*, double, metronome::Hash<Node>> actionProbabilities = computeActionProbabilities(simulatedStateNode);
      double score = 0.0;

      for (auto& entry : actionProbabilities) {
        Node* successor = entry.first;
        double probability = entry.second;

        double trial = interventionTrial(successor, rootState).first;
        #if NAIVEOPTIMALACTIVEGOALRECOGNITIONDESIGN_DEBUG_TRACE
          LOG(DEBUG) << pad(depth) << "AT " << entry.first->state << ": " << trial << " (Prob " << entry.second << ")";
        #endif
        score += probability * trial;
      }

      depth--;
      return score;
    }

    /**
     * Computes probability of every valid successor of the simulated state node.
     * Returns map of successor state to its probability
     * @param simulatedStateNode
     * @return
     */
    std::unordered_map<Node*, double, metronome::Hash<Node>> computeActionProbabilities(Node* simulatedStateNode) {
      std::unordered_map<State, double, StateHash> adjustedGoalPriors{};
      double reachablePriorSum = 0.0;
      for (auto& entry : simulatedStateNode->goalsToPlanCount) {
        const State& goal = entry.first;

        adjustedGoalPriors[goal] = goalsToPriors[goal];
        reachablePriorSum += goalsToPriors[goal];
      }
      for (auto& entry : adjustedGoalPriors) {
        adjustedGoalPriors[entry.first] = entry.second / reachablePriorSum;
      }

      std::unordered_map<Node*, double, metronome::Hash<Node>> actionProbabilities{};
      // for each action
      for (Edge& successorEdge : simulatedStateNode->successors) {
        // if g does not increase, the subject would not
        // transition from the simulated state to this successor via an optimal plan
        if (successorEdge.to->g <= simulatedStateNode->g) continue;

        // this means the successor is not on any optimal plan, so we can skip
        if (simulatedStateNode->goalBackupIteration != successorEdge.to->goalBackupIteration) continue;

        // this means the state is invalid (possibly due to prior intervention)
        if (!domain->isValidState(successorEdge.to->state)) continue;

        actionProbabilities[successorEdge.to] = 0.0;

        // cycle through goal hypothesis of successor
        for (auto& entry : successorEdge.to->goalsToPlanCount) {
          // Add to the probability of this action:
            // adjusted goal prior * fraction of plans from simulated node to the goal that pass through successor
          actionProbabilities[successorEdge.to] +=
                  adjustedGoalPriors[entry.first] *
                  ((double)entry.second / (double)simulatedStateNode->goalsToPlanCount[entry.first]);
        }
      }

      return actionProbabilities;
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
     * Flag is set when an identity action is trialed.
     * Signals that a trial is seeing what happens if no more interventions are executed
     */
    bool identityTrial = false;

    Domain* domain = nullptr;
  };
} // namespace metronome

#endif //METRONOME_NAIVEOPTIMALACTIVEGOALRECOGNITIONDESIGN_HPP
