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
#include "utils/PriorityQueue.hpp"
#include "planner_tools/Comparators.hpp"
#include "domains/SuccessorBundle.hpp"

namespace metronome {
  template<typename Domain>
  class NaiveOptimalActiveGoalRecognitionDesign final : public GoalRecognitionDesignPlanner<Domain> {
  public:
    using State = typename Domain::State;
    using Action = typename Domain::Action;
    using Intervention = typename Domain::Intervention;
    using Cost = typename Domain::Cost;
    using Planner = metronome::Planner<Domain>;
    using Base = metronome::GoalRecognitionDesignPlanner<Domain>;
    using InterventionBundle = typename Base::InterventionBundle;

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

    std::vector<InterventionBundle> selectInterventions(
            const typename Domain::State& subjectState, const Domain& systemState
    ) override {
      GoalRecognitionDesignPlanner<Domain>::beginIteration();
      // copy domain, set accessible pointer to it
      Domain localDomain(systemState);
      domain = &localDomain;

      recomputeOptimalInfo(subjectState);

      // TODO - Init the DFS

      domain = nullptr;

      // TODO - return something
    }

    State getGoalPrediction(const Domain& systemState,
                            const State& subjectState) override {
      // TODO
    }

  private:
    // Search Node
    struct Node {
    public:
      Node(Node* parent, const State& state, Cost g, Cost h, unsigned int iteration)
              : state(state), g(g), h(h), iteration{iteration} {
        if (parent != nullptr) {
          parents.emplace_back(parent);
        }
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
      std::unordered_set<Node*> parents{};
      /** Init successor set */
      std::unordered_set<Node*> successors{};
      bool open = true;

      /** Goal Hypothesis - goal state maps to plan count */
      std::unordered_map<State, size_t> goalsToPlanCount{};

      unsigned int iteration;
      unsigned int goalBackupIteration;

      std::string toString() {
        std::ostringstream stream;
        stream << "State: " << state << "; g: " << g << "; Iteration: " << iteration;

        return stream.str();
      }
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
      // reset open list comparator
      openList.clear();
      openList.reorder(&fComparator<Node>);

      Node* rootNode = getOrCreateSuccessor(nullptr, root);
      rootNode->g = 0;
      rootNode->iteration = Base::iterationCount;

      openList.push(rootNode);

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
            && foundGoals.insert(top).second
            && foundGoals.size() == numGoals) {
          finalFLayer = top->f();
        }
        top->open = false;

        // Even if it is a goal, must continue to expand
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
        goal->goalBackupIteration = Base::iterationCount;
        // seed the goals to plans map with this goal and count 1
        goal->goalsToPlanCount = {{goal->state, 1}};
        openList.push(goal);
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

    double actionTrial() {
      // TODO
    }

    double interventionTrial() {
      // TODO
    }

    /**
     * Might want to scrap this one - could probably efficiently implement within recomputeOptimalInfo.
     * On second thought, this one requires us to check affected states' predecessors. Might be just different enough?
     */
    void repairOptimalInfo() {
      // TODO
    }

    std::vector<double> computeActionProbabilities() {
      // TODO
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
    Node* getOrCreateSuccessor(const Node* parent, const State successorState) {
      Node* tempNode = nodes[successorState];

      if (tempNode == nullptr) {
        Planner::incrementGeneratedNodeCount();
        tempNode = nodePool.construct(
                parent,
                successorState,
                std::numeric_limits<Cost>::max(),
                getMinHeuristic(successorState, domain),
                Base::iterationCount);

        nodes[successorState] = tempNode;
      } else if (parent != nullptr) {
        tempNode->parents.insert(parent);
      }

      parent->successors.insert(tempNode);

      return tempNode;
    }

    /**
     * Of all possible goals gets the minimum heuristic for a given state
     * @param state
     * @param domain
     * @return
     */
    Cost getMinHeuristic(const State& state, const Domain& domain) {
      Cost minH = std::numeric_limits<Cost>::max();
      for (const auto& entry : goalsToPriors) {
        minH = std::min(minH, domain.heuristic(state, entry.first));
      }

      return minH;
    }

    /**
     * Basically a standard A* node expansion
     * @param source
     */
    void expandNode(Node* source) {
      Planner::incrementExpandedNodeCount();

      for (SuccessorBundle<Domain> successor : domain->successors(source->state)) {
        // handles adding to parent/successor lists
        Node* successorNode = getOrCreateSuccessor(source, successor.state);

        // Node is outdated
        if (successorNode->iteration != Base::iterationCount) {
          successorNode->iteration = Base::iterationCount;
          successorNode->g = std::numeric_limits<Cost>::max();
          successorNode->open = false;
        }

        Cost successorPotentialG = source->g + successor.actionCost;
        if (successorNode->g > successorPotentialG) {
          successorNode->g = successorPotentialG;

          if (!successorNode->open) {
            successorNode->open = true;
            openList.push(successorNode);
          } else {
            openList.update(successorNode);
          }
        }
      }
    }

    /**
     * Backing up goal hypothesis to predecessors along with plan counts
     * @param source
     */
    void backupGoalHypothesis(Node* source) {
      bool isGoal = domain->isGoal(source->state);

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

        openList.push(predecessor);
      }
    }

    /*****************************************
            Properties
    *****************************************/

    PriorityQueue<Node> openList;
    std::unordered_map<State, Node*, typename metronome::Hash<State>> nodes;
    ObjectPool<Node, Memory::NODE_LIMIT> nodePool;
    std::unordered_map<State, double> goalsToPriors;

    Domain* domain = nullptr;
  };
} // namespace metronome

#endif //METRONOME_NAIVEOPTIMALACTIVEGOALRECOGNITIONDESIGN_HPP
