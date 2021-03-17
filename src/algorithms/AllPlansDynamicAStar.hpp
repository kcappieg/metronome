#pragma once

#include <easylogging++.h>
#include <MemoryConfiguration.hpp>
#include "DynamicPlanner.hpp"
#include <utils/PriorityQueue.hpp>
#include "OfflinePlanner.hpp"
#include "Planner.hpp"
#include "experiment/Configuration.hpp"
#include "utils/Hash.hpp"
#include "utils/ObjectPool.hpp"
#include "visualization/Visualizer.hpp"
#include <domains/SuccessorBundle.hpp>
#include <algorithms/planner_tools/Comparators.hpp>

#include <unordered_map>
#include <unordered_set>
#include <random>
#include <vector>

namespace metronome {

/**
 * Dynamic Planner that uses an A-Star like search to find all plans to the
 * goal. The agent's plan is selected randomly by walking back through
 * parent pointers from the goal and randomly choosing from among optimal predecessors
 */
template <typename Domain>
class AllPlansDynamicAStar final : public DynamicPlanner<Domain> {
  using State = typename Domain::State;
  using Action = typename Domain::Action;
  using Cost = typename Domain::Cost;
  using Planner = metronome::Planner<Domain>;
  using ActionBundle = typename Planner::ActionBundle;
  
 public:
  explicit AllPlansDynamicAStar(const Domain&, const Configuration& config = {})
      : openList(Memory::OPEN_LIST_SIZE, fComparator<Node>)
        , randEngine{static_cast<uint64_t>(config.getLong(SEED, 0))}
  {
    // Initialize hash table
    nodes.max_load_factor(1);
    nodes.reserve(Memory::NODE_LIMIT);
  }

  std::vector<ActionBundle> replan(
      const State& currentState,
      const std::vector<State>&,
      const Domain& domain) override {
    ++iterationCounter;
    openList.clear();
    openList.reorder(fComparator<Node>);
    Node* root = getSuccessor(domain, currentState);
    root->g = 0;

    openList.push(*root);
    Cost finalFLayer = std::numeric_limits<Cost>::max(); ///< Explore this entire F-layer (when we find it)
    std::unordered_set<Node*, typename metronome::Hash<Node>> foundGoals;
    while (openList.isNotEmpty()) {
      Planner::incrementExpandedNodeCount();
      Node* currentNode = openList.pop();

      if (currentNode->f() > finalFLayer) {
        // this means there are no more optimal goal paths
        break;
      }

      if (domain.isGoal(currentNode->state)) {
        foundGoals.emplace(currentNode); // add to found goals if not a repeat

        // set final F: this is the endgame!
        finalFLayer = currentNode->g;
        continue; // no need to expand successors since they would have higher f-values
      }

      for (auto successor : domain.successors(currentNode->state)) {
        if (successor.state == currentNode->state) {
          continue;  // Skip parent
        }

        auto successorNode = getSuccessor(
            domain, successor.state, successor.action, successor.actionCost, currentNode);
        auto newCost = successor.actionCost + currentNode->g;
        if (successorNode->g > newCost) {
          successorNode->g = newCost;
          openList.insertOrUpdate(*successorNode);
        } // else the new path is not better than the existing
      }
    }

    if (foundGoals.empty()) {
      throw MetronomeException("Goal not found");
    }

    openList.clear();
    openList.reorder(gMaxComparator<Node>);

    // seed reverse open list with found goals
    // Each goal has planCount = 1 to seed the plan count backup
    for (auto& goalNode : foundGoals) {
      goalNode->planCount = 1;
      openList.push(*goalNode);
    }

    // loop until we have traced back to root
    while (*openList.top() != *root){
      // expand all monotonically decreasing g-value predecessors,
      // accumulating plan counts
      Node* currentNode = openList.pop();
      for (const BackEdge& parentEdge : currentNode->parents) {
        if (parentEdge.parent->g < currentNode->g) {
          Node* parent = parentEdge.parent;

          parent->planCount += currentNode->planCount; ///< Magic! Accumulating plan counts
          // add successor edge to the parent
          parent->optimalSuccessors.push_back(ForwardEdge{
              currentNode,
              parentEdge.action,
              parentEdge.actionCost
          });

          openList.insertOrUpdate(*parent);
        }
        // ignore nodes that don't monotonically decrease g
      }
    }

    // Now extract a path by walking forward and sampling successors weighted by plan count
    std::vector<ActionBundle> plan{};
    Node* currentNode = root;
    while (not domain.isGoal(currentNode->state)) {
      if (currentNode->optimalSuccessors.empty()) {
        throw MetronomeException("Unable to extract path to goal");
      }

      std::vector<double> weights{};
      weights.reserve(currentNode->optimalSuccessors.size());
      std::transform(currentNode->optimalSuccessors.begin(), currentNode->optimalSuccessors.end(),
                     std::back_inserter(weights),
                     [](const ForwardEdge& edge) {
                       return static_cast<double>(edge.successor->planCount);
                     });

      std::discrete_distribution<size_t> weightedDist{weights.begin(), weights.end()};
      auto successorIdx = weightedDist(randEngine);

      // record action in the plan
      const ForwardEdge& edge = currentNode->optimalSuccessors[successorIdx];
      plan.emplace_back(edge.action, edge.actionCost);
      currentNode = edge.successor;
    }

    return plan;
  }

 private:
  class Node;

  // Simple edges for forward and backward
  struct BackEdge {
    Node* parent;
    Action action;
    Cost actionCost;
  };

  // Yup, same as above. I'm tired and this makes it hard to fuck up
  // what goes where
  struct ForwardEdge {
    Node* successor;
    Action action;
    Cost actionCost;
  };

  class Node {
   public:
    Node(const State state, Cost h, size_t iteration)
        : state{state}
          , g{std::numeric_limits<Cost>::max()}, h{h}
          , iteration{iteration}
    {}

    unsigned long hash() const { return state.hash(); }

    std::string toString() const {
      std::ostringstream stream;
      stream << "s: " << state << " g: " << g << " h: " << h << " f: " << f()
             << "it: " << iteration << " parents: ";

      std::for_each(parents.begin(), parents.end(), [&stream](const BackEdge& parentEdge) {
        stream << "{ s:" << parentEdge.parent->state
               << ", a: " << parentEdge.action
               << ", cost: "<< parentEdge.actionCost << " }; ";
      });

      return stream.str();
    }

    Cost f() const { return g + h; }

    /** Reset the node by wiping any info that may have carried over from last iteration */
    void reset(size_t newIteration, Cost heuristic) {
      iteration = newIteration;
      h = heuristic;
      g = std::numeric_limits<Cost>::max();

      parents.clear();
      optimalSuccessors.clear();
      planCount = 0;

      // reset index in priority queue
      index = std::numeric_limits<unsigned int>::max();
    }

    bool operator==(const Node& node) const { return state == node.state; }
    bool operator!=(const Node& node) const { return state != node.state; }

    mutable unsigned int index{std::numeric_limits<unsigned int>::max()};
    const State state;
    Cost g;
    Cost h;
    size_t iteration;

    std::vector<BackEdge> parents{};
    /** Exclusively for storing successors that are in optimal plans */
    std::vector<ForwardEdge> optimalSuccessors{};
    size_t planCount{0}; ///< Count of plans that pass through this node
  };

  /**
   * Generates a successor, or gets one if it already exists.
   * Resets info if this is a new iteration
   */
  Node* getSuccessor(const Domain& domain,
                     const State& successorState,
                     const Action& action = {}, const Cost actionCost = 0,
                     Node* const parent = nullptr) {
    Node* successorNode = nodes[successorState];

    if (successorNode == nullptr) {
      Planner::incrementGeneratedNodeCount();
      successorNode = nodePool.construct(
          successorState,
          domain.heuristic(successorState),
          iterationCounter);

      nodes[successorState] = successorNode;
    }

    // Node is outdated
    // Reset predecessors and g value
    if (successorNode->iteration != iterationCounter) {
      // Recompute heuristic. Could have changed if the world changed
      successorNode->reset(iterationCounter, domain.heuristic(successorState));
    }

    // if not a root we set parent info
    if (parent != nullptr) {
      successorNode->parents.push_back(BackEdge{
          parent,
          action,
          actionCost
      });
    }

    return successorNode;
  }

  PriorityQueue<Node> openList;
  std::unordered_map<State, Node*, typename metronome::Hash<State>> nodes;
  ObjectPool<Node, Memory::NODE_LIMIT> nodePool;
  std::mt19937_64 randEngine;

  size_t iterationCounter{0};
};

}  // namespace metronome
