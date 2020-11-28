//
// Created by kevin on 11/19/20.
//

#pragma once

#include <experiment/Configuration.hpp>
#include <forward_list>
#include <limits>
#include <ostream>
#include <string>
#include <sstream>
#include <vector>

#include "GridWorld.hpp"
#include "SuccessorBundle.hpp"

namespace metronome {
/**
 * Implementation of IPC Easy-Grid domain.
 * Agent is navigating grid world with locked doors.
 * They need to find the keys to the doors that lead to their
 * goal. Unlocking doors is an additional action.
 *
 * For GRD domains, the observer can move through locked doors.
 * Interventions are picking up keys, thus making them unavailable
 * for the agent.
 *
 * Wrapper around GridWorld domain with KeyGrid-specific actions,
 * interventions, etc on top. Delegates much to GridWorld
 */
class KeyGrid {
 public:
  using Cost = GridWorld::Cost;

  /**
   * Additional actions are:
   * - P: Pick Up
   * - U: Unlock Door
   */
  class Action {
   public:
    std::optional<GridWorld::Action> gridAction;

    Action() : gridAction{GridWorld::Action()}, label{'~'} {}
    explicit Action(char label) : gridAction{}, label{label} {}
    explicit Action(GridWorld::Action gridAction)
        : gridAction{gridAction}, label{gridAction.toChar()} {}
    Action(const Action&) = default;
    Action(Action&&) = default;
    Action& operator=(const Action&) = default;
    ~Action() = default;


    [[nodiscard]] std::string toString() const {
      if (gridAction.has_value()) {
        return gridAction->toString();
      }

      return std::string(1,label);
    }

    friend std::ostream& operator<<(std::ostream& os, const Action& action) {
      return os << action.toString();
    }

   private:
    char label;
  };

  class State {
   public:
    bool operator==(const State& state) const {
      return gridState == state.gridState &&
        hasKey == state.hasKey &&
        hasKeyShape == state.hasKeyShape &&
        doorUnlocked == state.doorUnlocked;
    }

    bool operator!=(const State& state) const {
      return !(*this == state);
    }

    [[nodiscard]] std::size_t hash() const {
      size_t seed = 17;
      seed = (seed * 31) ^ gridState.hash();
      seed = (seed * 31) ^ std::hash<std::vector<bool>>{}(hasKey);
      seed = (seed * 31) ^ std::hash<std::vector<bool>>{}(hasKeyShape);
      seed = (seed * 31) ^ std::hash<std::vector<bool>>{}(doorUnlocked);

      return seed;
    }

    [[nodiscard]] std::string toString() const {
      std::stringstream stateStr{};
      stateStr << gridState << "; "
        << "HasKey:" << boolVecToString(hasKey) << "; "
        << "HasShape:" << boolVecToString(hasKeyShape) << "; "
        << "DoorsUnlocked" << boolVecToString(doorUnlocked);

      return stateStr.str();
    }

    [[nodiscard]] const GridWorld::State& getGridState() const {
      return gridState;
    }

    friend std::ostream& operator<<(std::ostream& os, const State& state) {
      return os << state.toString();
    }
   private:
    GridWorld::State gridState;

    // bool vectors for key and door state
    std::vector<bool> hasKey;
    std::vector<bool> hasKeyShape;
    std::vector<bool> doorUnlocked;

    static std::string boolVecToString(std::vector<bool> vec) {
      std::ostringstream vecStr{};
      vecStr << '<';
      for (size_t i = 0; i < vec.size(); i++) {
        if (vec[i]) {
          vecStr << "True";
        } else {
          vecStr << "False";
        }

        if (i < vec.size() - 1) {
          vecStr << ", ";
        }
      }
      vecStr << '>';

      return vecStr.str();
    }
  };

  /** Possible interventions for a Goal Recognition Design problem */
  class Intervention {
   public:
    [[nodiscard]] std::string toString() const {
      // TODO
    }
  };

  /**
   * If domain is implemented for Goal Recognition Design, Patch provides way to store interventions (modifications)
   */
  class Patch {
   public:
    [[nodiscard]] std::string toString() const {
      // TODO
    }

    // Hash? ==?

    /**
     * Dynamic algorithms sometimes receive information on which states were affected.
     * This property provides that information for a given intervention.
     * Note that these states could be "underspecified" according to the domain semantics
     */
    const std::vector<State> affectedStates;
  };

  KeyGrid(const Configuration& configuration,
          std::istream& mapInput, std::istream& sceneInput)
      : gridWorld(configuration, mapInput) {
    // TODO Read scene and build the rest of the scenario
    // Populate goalVector
  }

  std::optional<const State> transition(const State& state, const Action& action) const {
    // TODO
  }

  bool isGoal(const State& location) const {
    return gridWorld.isGoal(location.getGridState());
  }

  Cost heuristic(const State& state) const {
    return gridWorld.heuristic(state.getGridState());
  }

  Cost heuristic(const State& state, const State& other) const {
    return gridWorld.heuristic(state.getGridState(), other.getGridState());
  }

  std::vector<SuccessorBundle<KeyGrid>> successors(const State& state) const {
    // TODO
  }

  State getStartState() const {
    // TODO
  }

  Cost getActionDuration() const {
    return gridWorld.getActionDuration();
  }

  Cost getActionDuration(const Action&) const {
    return gridWorld.getActionDuration();
  }

  static Action getIdentityAction() {
    return Action();
  }

  // GRD-supporting methods

  /**
   * In multiple goal situations, implement this function to check if a state is
   * a specific goal
   * @param location
   * @param goalLocation
   * @return
   */
  bool isGoal(const State& location, const State& goalLocation) const {
    return metronome::GridWorld::isGoal(location.getGridState(), goalLocation.getGridState());
  }

  /**
   * Note: could be underspecified if domain semantics permit
   * @return
   */
  std::vector<State> getGoals() const {
    return goalVector;
  }

  /**
   * Set the goal to be used for subject planners
   * @param goal
   */
  void setCurrentGoal(const State& goal) {
    gridWorld.setCurrentGoal(goal.getGridState());
  }

  /**
   * Clear the subject goal so that the isGoal function can work for multiple goals
   */
  void clearCurrentGoal() {
    gridWorld.clearCurrentGoal();
  }

  /**
   * Allowing interventions method to take a vector of states so that we can retrieve
   * many interventions at once
   * @param currentState The current state being considered
   * @param lookaheadStates Algorithms may have some lookahead. This param allows
   * them to pass the lookahead. The domain can then return only interventions relevant
   * to these states, if semantically applicable
   * @return
   */
  std::vector<InterventionBundle<KeyGrid>> interventions(
      const State& currentState, const std::vector<State>& lookaheadStates) const {
    // TODO
  }

  Intervention getIdentityIntervention() const {
    // TODO
  }

  /**
   * This method will mutate the domain
   * @param intervention
   * @param subjectState Current state of subject (checks legal interventions)
   * @return The patch that was applied. Will not have value on failure
   */
  std::optional<Patch> applyIntervention(const Intervention& intervention, const State& subjectState) {
    // TODO
  }

  /**
   * This method will mutate the domain
   */
  void reversePatch(const Patch&, const State& subjectState) {
    // TODO
  }

 private:
  GridWorld gridWorld;
  std::vector<State> goalVector;
};

}  // namespace metronome
