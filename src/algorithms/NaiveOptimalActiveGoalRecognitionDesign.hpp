//
// Created by kevin on 5/14/20.
//

#ifndef METRONOME_NAIVEOPTIMALACTIVEGOALRECOGNITIONDESIGN_HPP
#define METRONOME_NAIVEOPTIMALACTIVEGOALRECOGNITIONDESIGN_HPP

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
        : openList(Memory::OPEN_LIST_SIZE, fComparator<Node>()){
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
      for (auto goal : systemState.getGoals()) {
        if (systemState.isGoal(subjectState, goal)) return goal;
      }

      // just return the first one
      return systemState.getGoals()[0];
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

      std::string toString() {
        std::ostringstream stream;
        stream << "State: " << state << "; g: " << g << "; Iteration: " << iteration;

        return stream.str();
      }
    };

    /*****************************************
            Algorithm methods
    *****************************************/

    /**
     * Enumerate all optimal plans to all goals so that we can configure goal hypotheses and plan counts.
     * Basically A* except we don't stop until we've exhausted the full f-layer of all goals
     * @param root
     */
    void recomputeOptimalInfo(const State& root) {
      // init open list
      Node* rootNode = getOrCreateSuccessor(nullptr, root);
      rootNode->g = 0;
      rootNode->iteration = Base::iterationCount;

      openList.push(rootNode);

      // set up local set for goals. We will tick them off when we expand them
      std::unordered_set<State> unfoundGoals{};
      for (auto& entry : goalsToPriors)  unfoundGoals.insert(entry.first);

      Cost finalFLayer = std::numeric_limits<Cost>::max();

      // while open isn't empty or goal count > 0
      // expand as A* - add to successors / parents

      // If a goal is expanded, check if it's already been expanded
      // mark it off, record its f-layer value
      // once all goals have been found, set the "final" f layer value. Once any node above that f-layer is expanded, break loop

      // From each goal, trace back to all monotonically decreasing (in g) parents
      // carry backward the number of paths through this node as we go, and add the goal to the goal hypothesis
        // To accomplish this, probably need to use the open list. Perhaps a max-queue on g seeded with each goal?
        // Do all goal backtracking in 1 by tracking which goal(s) the expanded node can reach optimally

      // clear open list - we may need it again, so ensure it's clear
      openList.clear();

      // TODO
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
     * Creates a node if it doesn't already exist. Updates parent / successor sets
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
