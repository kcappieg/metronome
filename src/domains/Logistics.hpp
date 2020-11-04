#pragma once

#include <experiment/Configuration.hpp>
#include <limits>
#include <ostream>
#include <sstream>
#include <string>
#include <vector>
#include "SuccessorBundle.hpp"

namespace metronome {

/**
 * Variant of IPC Logistics domain. Trucks are dispatched to
 * pick up packages and deliver them to locations.
 * This variant adds road networks between locations, and excludes
 * airports / airplanes as modes of transportation.
 * We arbitrarily assign a capacity to the trucks of 5
 *
 * Goals specifications are subsets of packages being delivered to specific
 * locations
 *
 * GRD-supporting methods add a set of observer agents. They follow the
 * same movement rules as the trucks, but instead of picking up packages
 * they can block roads from location to location
 *
 * Implemented heuristic is relatively simple, not meant to be terribly strong:
 *
 */
class Logistics {
 public:
  typedef long long int Cost;

  /**
   * Agent actions. Load, unload, drive.
   * All actions store some reference to the object or location being
   * interacted with as well as the truck
   */
  class Action {
   public:
    enum Type {
      LOAD,
      UNLOAD,
      DRIVE,
      IDENTITY
    };

    Action() : type{IDENTITY} {}
    // LOAD / UNLOAD action constructor
    Action(size_t truckId, size_t pkgId, Type type)
        : truckId{truckId}, pkgId{pkgId}, type{type} {}
    // DRIVE action constructor
    Action(size_t truckId, size_t locationId)
        : truckId{truckId}, locationId{locationId}, type{DRIVE} {}

    [[nodiscard]] std::string toString() const {
      std::ostringstream actionStr;
      switch(type) {
        case LOAD:
          actionStr << "L pkg" << pkgId << " trk" << truckId;
          break;
        case UNLOAD:
          actionStr << "U pkg" << pkgId << " trk" << truckId;
          break;
        case DRIVE:
          actionStr << "D trk" << truckId << " loc" << locationId;
          break;
        case IDENTITY:
          actionStr << "Identity";
          break;
      }

      return actionStr.str();
    }

    friend std::ostream& operator<<(std::ostream& os, const Action& action) {
      return os << action.toString();
    }

    const size_t truckId = 0;
    const size_t pkgId = 0;
    const size_t locationId = 0;
    const Type type;
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
     * This property provides that information for a given intervention.
     * Note that these states could be "underspecified" according to the domain semantics
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
   * In multiple goal situations, implement this function to check if a state is
   * a specific goal
   * @param location
   * @param goalLocation
   * @return
   */
  bool isGoal(const State& location, const State& goalLocation) const {
    // TODO
  }

  /**
   * Note: could be underspecified if domain semantics permit
   * @return
   */
  const std::vector<State> getGoals() const {
    // TODO
  }

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
  // Internal state of domain
  // Note: we are 1-indexing domain items so that
  // 0 can be reserved as "NULL"

  // trucks
  size_t numTrucks;
  std::vector<size_t> truckLocations{};


  size_t numPkgs;
  size_t numLocations;
};

}  // namespace metronome
