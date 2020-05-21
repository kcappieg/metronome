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
        : openList(Memory::OPEN_LIST_SIZE, &fComparator<Node>){
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

      uint64_t recomputeOptimalBegin = recomputeOptimalIteration;

      recomputeOptimalInfo(subjectState);

      auto intervention = interventionTrial(nodes[subjectState], subjectState).second;
      domain = nullptr;

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
      bool open = true;

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
      bool isActive{true};
    };

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
      }

      // Loop until open is fully empty
      int64_t backupCount = 0;
      while (openList.isNotEmpty()) {
        Node* top = openList.pop();

        backupGoalHypothesis(top);
        backupCount++;
      }
      Base::recordAttribute("backups", backupCount);

      // clear open list - we no longer need it, so ensure no bugs slip in for later uses of it
      openList.clear();
    }

    // Recursion - all going on the stack. Might become stack overflow - watch for this

    std::pair<double, std::optional<InterventionBundle<Domain>>> interventionTrial(
            Node* simulatedStateNode, const State& subjectState) {
      // sigma
      if (simulatedStateNode->goalsToPlanCount.size() == 1) {
        auto goalState = simulatedStateNode->goalsToPlanCount.cbegin()->first;
        return {simulatedStateNode->g / goalsToOptimalCost[goalState], {}};
      }

      // get states relevant to the subject state
      std::vector<State> optimalPlanStates {simulatedStateNode->state};
      std::queue<Node*> queue {{simulatedStateNode}};
      while (!queue.empty()) {
        Node* next = queue.front();
        for (Edge& edge : next->successors) {
          // grab successor node from edge
          Node* succ = edge.to;
          if (succ->goalBackupIteration == next->goalBackupIteration) {
            optimalPlanStates.emplace_back(succ->state);
            queue.push(succ);
          }
        }
      }

      double score = 1.0;
      std::optional<InterventionBundle<Domain>> argminIntervention{};
      for (InterventionBundle interventionBundle : domain->interventions(optimalPlanStates)) {
        std::optional<Patch> optionalDomainPatch =
                domain->applyIntervention(interventionBundle.intervention, simulatedStateNode->state);
        // invalid intervention - we will not consider it
        if (!optionalDomainPatch.has_value()) continue;

        // grab patch, repair optimal info
        const Patch& domainPatch = optionalDomainPatch.value();
        // copy goals to optimal cost map so we can verify they don't change
        std::unordered_map<State, Cost, StateHash> goalsToOptimalCostCopy(goalsToOptimalCost);
        recomputeOptimalInfo(subjectState);

        // detect change in optimal cost(s)
        bool invalidIntervention = false;
        for (auto& entry : goalsToOptimalCostCopy) {
          Cost originalCost = entry.second;
          Cost newCost = goalsToOptimalCost[entry.first];

          if (originalCost != newCost) {
            invalidIntervention = true;
            break;
          }
        }
        if (invalidIntervention) {
          // intervention changed the optimal cost to some goal. Reverse and continue
          domain->reversePatch(domainPatch, simulatedStateNode->state);
          recomputeOptimalInfo(subjectState);
          continue;
        }

        // descend
        double trialScore = actionTrial(simulatedStateNode);
        if (trialScore < score) {
          score = trialScore;
          argminIntervention.emplace(interventionBundle);
        }

        // reverse patch, repair optimal info
        domain->reversePatch(domainPatch, simulatedStateNode->state);
        recomputeOptimalInfo(subjectState);
      }

      return {score, argminIntervention};
    }

    double actionTrial(Node* simulatedStateNode, const State& subjectState) {
      // sigma
      if (simulatedStateNode->goalsToPlanCount.size() == 1) {
        auto goalState = simulatedStateNode->goalsToPlanCount.cbegin()->first;
        return simulatedStateNode->g / goalsToOptimalCost[goalState];
      }

      std::vector<double> actionProbabilities = computeActionProbabilities(simulatedStateNode);
      // TODO
    }

// Scrapped this. Too fancy for a "Naive Optimal." Complexity was ballooning and correctness was hard to
// reason about, so instead just recompute optimal plans from scratch every time
//    bool repairOptimalInfo(const std::vector<State>& changedStates) {
//      // TODO
//      // START HERE
//      // you've refactored to use edges for search nodes
//      // mark edges inactive when applicable, use that to identify when changes need to be made
//
//      for (const State& state : changedStates) {
//        Node* changedNode = nodes[state];
//        if (changedNode == nullptr || !domain->isValidState(state)) {
//          // We will deal with this when we find its successor/predecessor
//          continue;
//        }
//
//        std::unordered_set<State, StateHash> possibleStates{};
//        for (auto& entry : changedNode->successors) {
//          // max cost means the edge doesn't exist right now, so we ignore it
//          if (entry.second.actionCost < std::numeric_limits<Cost>::max()) {
//            possibleStates.insert(entry.first);
//          }
//        }
//
//        // check for change in g-value: match successors with existing edges, detect edge cost changes
//        for (SuccessorBundle<Domain> successor : domain->successors(state)) {
//          possibleStates.erase(successor.state);
//          Edge& edge = changedNode->successors[successor.state];
//
//          if (edge == nullptr) {
//            // technically reduced cost from infinity - successor goes on open
//            Node* successorNode = createSuccessor(changedNode, successor.state, successor.actionCost);
//          } else if (edge.actionCost != successor.actionCost) {
//            if (edge.actionCost > successor.actionCost) {
//              // reduced cost - successor goes on open
//
//            } else {
//              // increased cost - back up change in goal hypothesis
//
//            }
//
//            edge.actionCost = successor.actionCost;
//          }
//        }
//
//        // any states leftover were unreachable
//        for (State& unreachableState : possibleStates) {
//          Edge& edge = changedNode->successors[unreachableState];
//          if (edge.actionCost != std::numeric_limits<Cost>::max()) {
//            edge.actionCost = std::numeric_limits<Cost>::max();
//            // increased cost - back up change in goal hypothesis
//          }
//        }
//      }
//
//      // cycle through states, see what changed
//      // back up changes if edges removed
//      // advance forward info if edges added
//    }

    /**
     * Computes probability of every valid successor of the simulated state node.
     * Returns map of successor state to its probability
     * @param simulatedStateNode
     * @return
     */
    std::unordered_map<State, double, StateHash> computeActionProbabilities(Node* simulatedStateNode) {
      std::unordered_map<State, double, StateHash> adjustedGoalPriors{};
      double reachablePriorSum = 0.0;
      for (auto& entry : simulatedStateNode->goalsToPlanCount) {
        State& goal = entry.first;

        adjustedGoalPriors[goal] = goalsToPriors[goal];
        reachablePriorSum += goalsToPriors[goal];
      }
      for (auto& entry : adjustedGoalPriors) {
        adjustedGoalPriors[entry.first] = entry.second / reachablePriorSum;
      }

      std::unordered_map<State, double, StateHash> actionProbabilities{};
      // for each action
      for (size_t i = 0; i < simulatedStateNode->successors.size(); i++) {
        Edge& successorEdge = simulatedStateNode->successors[i];

        // this means the successor is not on any optimal plan, so we can skip
        if (simulatedStateNode->goalBackupIteration != successorEdge.to->goalBackupIteration) continue;

        State& successorState = successorEdge.to->state;
        actionProbabilities[successorState] = 0.0;

        // cycle through goal hypothesis of successor
        for (auto& entry : successorEdge.to->goalsToPlanCount) {
          // Add to the probability of this action:
            // adjusted goal prior * fraction of plans from simulated node to the goal that pass through successor
          actionProbabilities[successorState] +=
                  adjustedGoalPriors[entry.first] * (entry.second / simulatedStateNode->goalsToPlanCount[entry.first]);
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

      if (nodes[successorState] == nullptr) {
        Planner::incrementGeneratedNodeCount();
        Node* tempNode = nodePool.construct(
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
      for (Node* predecessor : source->parents) {
        // skip if g is not monotonically decreasing
        // Also, if we did not touch predecessor this iteration we'll skip it
        if (predecessor->g >= source->g || predecessor->iteration != source->iteration) continue;

        // If we haven't seen this node this backup iteration, reset its goal hypothesis
        if (predecessor->goalBackupIteration != source->goalBackupIteration) {
          predecessor->goalBackupIteration = source->goalBackupIteration;
          predecessor->goalsToPlanCount = {};
        }

        assert(source->goalsToPlanCount.size() > 0);
        for (const auto& entry : source->goalsToPlanCount) {
          // Try emplace does nothing if key already exists
          predecessor->goalsToPlanCount.try_emplace(entry.first, 0);
          predecessor->goalsToPlanCount[entry.first] += entry.second;
        }

        openList.push(*predecessor);
      }
    }

    /*****************************************
            Properties
    *****************************************/

    PriorityQueue<Node> openList;
    std::unordered_map<State, Node*, StateHash> nodes;
    ObjectPool<Node, Memory::NODE_LIMIT> nodePool;
    std::unordered_map<State, double, StateHash> goalsToPriors;

    // Fields that reset after each iteration
    std::unordered_map<State, Cost, StateHash> goalsToOptimalCost{};
    uint64_t recomputeOptimalIteration = 0;

    Domain* domain = nullptr;
  };
} // namespace metronome

#endif //METRONOME_NAIVEOPTIMALACTIVEGOALRECOGNITIONDESIGN_HPP
