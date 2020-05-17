#pragma once

#include <experiment/Configuration.hpp>
#include <limits>
#include <string>
#include <vector>
#include "SuccessorBundle.hpp"

namespace metronome {

class Domain {
 public:
  typedef long long int Cost;

  class Action {
   public:

    std::string toString() const {
      // TODO
    }
  };

  class State {
   public:
    bool operator==(const State& state) const {
      // TODO
    }

    std::size_t hash() const {
      // TODO
    }

    std::string toString() const {
      // TODO
    }
  };

  /** Possible interventions for a Goal Recognition Design problem */
  class Intervention {
   public:
    std::string toString() const {
      // TODO
    }
  };

  /**
   * If domain is implemented for Goal Recognition Design, Patch provides way to store interventions (modifications)
   */
  class Patch {
   public:
    std::string toString() const {
      // TODO
    }

    // Hash? ==?

    /**
     * Dynamic algorithms sometimes receive information on which states were affected.
     * This property provides that information for a given intervention
     */
    const std::vector<State> affectedStates;
  };

  Domain(const Configuration& configuration, std::istream& input) {
    // TODO
  }

  std::optional<const State> transition(const State& state, const Action& action) const {
    // TODO
  }

  /**
   * Validates that the agent can visit the state
   * @param state
   * @return
   */
  bool isValidState(const State& state) const {
    // TODO
  }


  bool isGoal(const State& location) const {
    // TODO
  }

  /**
   * In multiple goal situations, implement this function to check if a state is
   * a specific goal
   * @param location
   * @param goalLocation
   * @return
   */
  bool isGoal(const State& location, const State& goalLocation) const {
    // TODO
  }

  const std::vector<State> getGoals() const {
    // TODO
  }

  Cost distance(const State& state) const {
    // TODO
  }

  Cost heuristic(const State& state) const {
    // TODO
  }

  std::vector<SuccessorBundle<Domain>> successors(const State& state) const {
    // TODO
  }

  const State getStartState() const {
    // TODO
  }

  Cost getActionDuration() const {
    // TODO
  }

  Cost getActionDuration(const Action& action) const {
    // TODO
  }

  Action getIdentityAction() const {
    // TODO
  }

  bool safetyPredicate(const State& state) const {
    // TODO
  }

  // GRD-supporting methods

  /**
 * Set the goal to be used for subject planners
 * @param goal
 */
  void setCurrentGoal(State goal) {
    // TODO
  }

  /**
   * Clear the subject goal so that the isGoal function can work for multiple goals
   */
  void clearCurrentGoal() {
    // TODO
  }

  /**
   * Allowing interventions method to take a vector of states so that we can retrieve
   * many interventions at once
   * @param states
   * @return
   */
  std::vector<InterventionBundle<Domain>> interventions(const std::vector<State>& states) const {
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
};

}  // namespace metronome
