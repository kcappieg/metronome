#pragma once

#include <experiment/Configuration.hpp>
#include <algorithm>
#include <array>
#include <forward_list>
#include <limits>
#include <iostream>
#include <iterator>
#include <ostream>
#include <sstream>
#include <string>
#include <string_view>
#include <utility>
#include <vector>
#include "SuccessorBundle.hpp"

namespace metronome {

// reserving max index as the "null ID" when that needs to be expressed
static constexpr size_t NULL_ID = std::numeric_limits<size_t>::max();

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
 * Implemented heuristic is relatively simple, not heavily engineered.
 * Basically accumulates packages not at their goal locations
 *
 */
class Logistics {
  // forward declare
 public:
  typedef long long int Cost;
 private:
  /** Represents an edge between 2 locations */
  struct Edge {
    const size_t fromLoc;
    const size_t toLoc;
    const Cost edgeCost;
  };
 public:

  /**
   * Agent actions. Load, unload, drive.
   * All actions store some reference to the object or location being
   * interacted with as well as the truck.
   * To express non-unit cost for driving between locations, the
   * successor of drive actions will be followed by more drive actions
   * until all cost has been expressed as time steps in the domain
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
      std::ostringstream actionStr{};
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

    size_t truckId = NULL_ID;
    size_t pkgId = NULL_ID;
    size_t locationId = NULL_ID;
    Type type;
  };

  class State {
   public:
    // Typedef-ing so I can change capacity by editing this one line
    typedef std::array<size_t, 5> Load;
    static constexpr Load emptyLoad = {NULL_ID, NULL_ID, NULL_ID, NULL_ID, NULL_ID};

    bool operator==(const State& state) const {
      return truckLocations == state.truckLocations &&
             pkgLocations == state.pkgLocations &&
             truckLoads == state.truckLoads;
    }

    bool operator!=(const State& state) const {
      return !(*this == state);
    }

    [[nodiscard]] std::size_t hash() const {
      size_t seed = 17;

      for (const size_t id : truckLocations) {
        seed = (seed * 31) ^ std::hash<size_t>{}(id);
      }
      for (const size_t id : pkgLocations) {
        seed = (seed * 31) ^ std::hash<size_t>{}(id);
      }
      for (const auto& load : truckLoads) {
        for (const size_t id : load) {
          seed = (seed * 31) ^ std::hash<size_t>{}(id);
        }
      }

      return seed;
    }

    [[nodiscard]] std::string toString() const {
      std::ostringstream stateStr{};
      stateStr << "Truck locs: " << vecToString(truckLocations)
        << "; " << "Object locs: " << vecToString(pkgLocations)
        << "; " << "Truck loads[" << vecToString(truckLoads) << "];";

      return stateStr.str();
    }

    /** location id of truck. Index is truck ID */
    std::vector<size_t> truckLocations{};
    /** location id of object. Index is object ID. If on truck, location will be NULL_ID */
    std::vector<size_t> pkgLocations{};
    /**
     * Tracks the load of every truck. Indexed by truck ID.
     * array entries are object IDs. NULL_ID if none
     */
    std::vector<Load>truckLoads{};

    friend std::ostream& operator<<(std::ostream& os, const State& state) {
      return os << state.toString();
    }
   private:

    [[nodiscard]] static std::string vecToString(const std::vector<size_t>& vec) {
      std::ostringstream vecStr{};
      vecStr << '<';

      for (size_t i = 0; i < vec.size(); i++) {
        if (vec[i] == NULL_ID) {
          vecStr << "NULL";
        } else {
          vecStr << vec[i];
        }

        if (i < vec.size() - 1) {
          vecStr << ',';
        }
      }
      vecStr << '>';

      return vecStr.str();
    }

    [[nodiscard]] static std::string vecToString(const std::vector<Load>& vec) {
      std::ostringstream vecStr{};

      for (size_t i = 0; i < vec.size(); i++) {
        const auto& load = vec[i];
        vecStr << "Trk" << i << ": <";

        for (size_t j = 0; j < load.size(); j++) {
          if (load[j] == NULL_ID) {
            vecStr << "NULL";
          } else {
            vecStr << load[j];
          }

          if (j < load.size() - 1) {
            vecStr << ',';
          }
        }
        vecStr << '>' << (i < vec.size() - 1 ? "," : "");
      }

      return vecStr.str();
    }
  };

  /** Possible interventions for a Goal Recognition Design problem */
  class Intervention {
   public:
    enum Type {
      DRIVE,
      BLOCK,
      IDENTITY
    };

    Intervention() : locationId{NULL_ID}, type{IDENTITY} {};
    Intervention(size_t locationId, Type type)
        : locationId{locationId}, type{type} {}

    bool operator==(const Intervention& intervention) const {
      return type == intervention.type && locationId == intervention.locationId;
    }
    bool operator!=(const Intervention& intervention) const {
      return !(*this == intervention);
    }

    [[nodiscard]] std::string toString() const {
      std::ostringstream interventionStr{};
      switch (type) {
        case BLOCK:
          interventionStr << "B loc" << locationId;
          break;
        case DRIVE:
          interventionStr << "D loc" << locationId;
          break;
        case IDENTITY:
          interventionStr << "Identity";
          break;
      }
      return interventionStr.str();
    }

    friend std::ostream& operator<<(std::ostream& os, const Intervention& intervention) {
      return os << intervention.toString();
    }

    size_t locationId;
    Type type;
  };

  /**
   * If domain is implemented for Goal Recognition Design, Patch provides way to store interventions (modifications)
   */
  class Patch {
   private:
    Patch(
        Intervention intervention,
        size_t originLocId,
        const Edge& blockedEdge,
        std::vector<State> affectedStates)
        : intervention{intervention},
          originLocId{originLocId}, blockedEdge{blockedEdge},
          affectedStates{std::move(affectedStates)} {}
   public:
    // Identity patch
    explicit Patch(Intervention intervention)
        : Patch(intervention, NULL_ID, {}, {}) {}

    // Drive patch
    Patch(Intervention intervention, size_t originLocId)
        : Patch(intervention, originLocId, {}, {}) {}

    // Block patch
    Patch(Intervention intervention, const Edge blockedEdge, const State& subjectState)
        : Patch(intervention, NULL_ID, blockedEdge, {subjectState}) {}

    const Intervention intervention;
    const size_t originLocId;
    const Edge blockedEdge;
    /**
     * Dynamic algorithms sometimes receive information on which states were affected.
     * This property includes the "underspecified" states that are affected.
     * For this domain, this should be states where a truck is in one or the other
     * of the locations whose edge was removed
     */
    const std::vector<State> affectedStates;
  };

  /**
   * Construct logistics domain from input. See input template in resources/inputs/logistics
   * Note that any object or truck locations not specified default to location 0
   * @param configuration
   * @param input
   */
  Logistics(const Configuration& configuration, std::istream& input)
    : actionDuration(configuration.getLong(ACTION_DURATION, 1)),
      interventionCost(configuration.getLong(INTERVENTION_COST, 1)),
      heuristicMultiplier(configuration.getDouble(HEURISTIC_MULTIPLIER, 1)) {
    std::string line;
    bool notEof;

//    {
//      State testState{};
//      testState.pkgLocations.push_back(NULL_ID);
//      std::cout << testState;
//    }

    // get first line, which should correspond to locations
    const std::string locPrefix = "Locations:";
    getNextLine(line, input);
    if (line.rfind(locPrefix,0) != 0) {
      throw MetronomeException("No location count specified");
    }

    line = line.substr(locPrefix.size());
    this->numLocations = std::stoul(line);
    // default construct edge vectors
    this->edgeLookup.resize(numLocations, {});

    // extract all edges
    const std::string truckPrefix = "Trucks:"; // break condition
    notEof = getNextLine(line, input);
    while(notEof) {
      if (line.rfind(truckPrefix, 0) == 0) {
        // move on to next segment
        break;
      }
      std::vector<std::string> splitStr = split(line, ' ');
      if (splitStr.size() != 3) {
        throw MetronomeException("Invalid location edge");
      }

      size_t locA = std::stoul(splitStr[0]),
             locB = std::stoul(splitStr[1]);
      Cost cost = std::stoll(splitStr[2]);

      // if locA or B is out of range, runtime error
      edgeLookup[locA].push_front({locA, locB, cost});
      edgeLookup[locB].push_front({locB, locA, cost});

      notEof = getNextLine(line, input);
    }

    if (!notEof) {
      throw MetronomeException("No trucks specified");
    }

    line = line.substr(truckPrefix.size());
    this->numTrucks = std::stoul(line);
    startState.truckLocations.resize(numTrucks, 0);
    startState.truckLoads.resize(numTrucks, State::emptyLoad);

    // extract truck start locations
    const std::string pkgPrefix = "Packages:"; // break condition
    notEof = getNextLine(line, input);
    while(notEof) {
      if (line.rfind(pkgPrefix, 0) == 0) {
        // move to next segment
        break;
      }

      std::vector<std::string> splitStr = split(line, ' ');
      if (splitStr.size() != 2) {
        throw MetronomeException("Invalid truck location");
      }
      // note: not detecting if truck already assigned location. User-input issue
      startState.truckLocations[std::stoul(splitStr[0])] = std::stoul(splitStr[1]);

      notEof = getNextLine(line, input);
    }

    if (!notEof) {
      throw MetronomeException("No packages specified");
    }

    line = line.substr(pkgPrefix.size());
    this->numPkgs = std::stoul(line);
    startState.pkgLocations.resize(numPkgs, 0);

    // extract truck start locations
    const std::string goalPrefix = "Goals:"; // break condition
    notEof = getNextLine(line, input);
    while(notEof) {
      if (line.rfind(goalPrefix, 0) == 0) {
        // move to next segment
        break;
      }

      std::vector<std::string> splitStr = split(line, ' ');
      if (splitStr.size() != 2) {
        throw MetronomeException("Invalid package location");
      }
      // note: not detecting if pkg already assigned location. User-input issue
      startState.pkgLocations[std::stoul(splitStr[0])] = std::stoul(splitStr[1]);

      notEof = getNextLine(line, input);
    }

    if (!notEof) {
      throw MetronomeException("No goals specified");
    }

    // extract possible goals
    const std::string observerPrefix = "Observer:"; // break condition
    notEof = getNextLine(line, input);
    while(notEof) {
      if (line.rfind(observerPrefix, 0) == 0) {
        // move to next segment
        break;
      }
      // init new goal with null values for all state variables
      goals.push_back({
          std::vector<size_t>(numTrucks, NULL_ID),
          std::vector<size_t>(numPkgs, NULL_ID),
          std::vector<State::Load>(numTrucks, State::emptyLoad)
      });
      State &goalState = goals.back();

      std::vector<std::string> goalConditions = split(line, ',');
      for (auto& condition : goalConditions) {
        // remove parens
        condition.erase(0, 1).erase(condition.size() - 1, 1);

        std::vector<std::string> splitCondition = split(condition, ' ');
        // not checking for duplicate conditions, just overwriting
        goalState.pkgLocations[std::stoul(splitCondition[0])] = std::stoul(splitCondition[1]);
      }

      notEof = getNextLine(line, input);
    }

    // if we haven't reached end of file, extract observer info
    if (notEof) {
      line = line.substr(observerPrefix.size());
      this->observerLoc = std::stoul(line);
    }

    if (goals.size() == 1) {
      useSingleGoal = true;
      singleGoal = goals[0];
    } else if (goals.empty()) {
      throw MetronomeException("No goals specified");
    }
  }

  [[nodiscard]] std::optional<const State> transition(
      const State& state, const Action& action) const {
    switch(action.type){
      case Action::LOAD:
        return loadPkg(state, action);
      case Action::UNLOAD:
        return unloadPkg(state, action);
      case Action::DRIVE:
        return drive(state, action);
      case Action::IDENTITY:
        break;
    }
    // identity action
    return {};
  }

  [[nodiscard]] bool isGoal(const State& location) const {
    if (useSingleGoal) {
      return isGoal(location, singleGoal);
    } else {
      auto isGoalPred = [this, &location](const State& goal) {
        return isGoal(location, goal);
      };
      return std::any_of(goals.cbegin(), goals.cend(), isGoalPred);
    }

  }

  /**
   * Heuristic to the current goal, if applicable
   * @param state
   * @return
   */
  [[nodiscard]] Cost heuristic(const State& state) const {
    if (useSingleGoal) {
      return goalDistance(state, singleGoal) * actionDuration;
    } else {
      throw MetronomeException("Goal ambiguous - Distance function invoked with implicit goal, but goal set is greater than one");
    }
  }

  /**
   * Heuristic to other state. State is assumed to be underspecified
   * @param state
   * @param underspecifiedState
   * @return
   */
  [[nodiscard]] Cost heuristic(const State& state, const State& underspecifiedState) const {
    return goalDistance(state, underspecifiedState) * actionDuration;
  }

  /**
   * Return successors.
   * If traveling, returns another driving action. Otherwise, returns all
   * possible load/unload or drive truck actions
   */
  [[nodiscard]] std::vector<SuccessorBundle<Logistics>> successors(const State& state) const {
    std::vector<SuccessorBundle<Logistics>> successors{};

    // loop through all trucks, looking for drive, load, and unload actions
    for (size_t truckId = 0; truckId < numTrucks; truckId++) {
      size_t truckLocId = state.truckLocations[truckId];

      // load for all possible packages
      for (size_t pkgId = 0; pkgId < numPkgs; pkgId++) {
        if (truckLocId != state.pkgLocations[pkgId]) continue; // can't load what's not there

        addValidSuccessor(successors, state, Action(truckId, pkgId, Action::Type::LOAD));
      }
      // unload for all pkgs in truck's load
      for (size_t pkgId : state.truckLoads[truckId]) {
        if (pkgId == NULL_ID) continue; //empty slot

        addValidSuccessor(successors, state, Action(truckId, pkgId, Action::Type::UNLOAD));
      }

      // drive for all edges
      for (const Edge& edge : edgeLookup[truckLocId]){
        addValidSuccessor(successors, state, Action(truckId, edge.toLoc));
      }
    }

    return successors;
  }

  [[nodiscard]] State getStartState() const {
    return startState;
  }

  /** Returns duration of identity action */
  [[nodiscard]] Cost getActionDuration() const {
    return actionDuration;
  }

  [[nodiscard]] Cost getActionDuration(const Action&) const {
    return actionDuration;
  }

  [[nodiscard]] static Action getIdentityAction() {
    return Action();
  }

  // GRD-supporting methods

  /**
   * Compares all package locations in the goal vs the location. If all match,
   * it is a goal!
   * @param location
   * @param goalLocation expected to be underspecified
   * @return
   */
  [[nodiscard]] bool isGoal(const State& location, const State& goalLocation) const {
    for (size_t i = 0; i < numPkgs; i++) {
      size_t goalPkgLoc = goalLocation.pkgLocations[i];
      // we don't care when the goal does not specify a pkg location
      if (goalPkgLoc != NULL_ID && goalPkgLoc != location.pkgLocations[i]) {
        return false;
      }
    }
    return true;
  }

  /**
   * Note: could be underspecified if domain semantics permit
   * @return
   */
  [[nodiscard]] std::vector<State> getGoals() const {
    return goals;
  }

  /**
   * Set the goal to be used for subject planners
   * @param goal
   */
  void setCurrentGoal(const State& goal) {
    useSingleGoal = true;
    singleGoal = goal; // copied
  }

  /**
   * Clear the subject goal so that the isGoal function can work for multiple goals
   */
  void clearCurrentGoal() {
    useSingleGoal = false;
  }

  /**
   * Possible intervention actions are for the observer to move or
   * block edges between locations.
   * Modeling only a single observer agent
   * @param currentState
   * @param states
   * @return
   */
  [[nodiscard]] std::vector<InterventionBundle<Logistics>> interventions(
      [[maybe_unused]] const State& currentState,
      [[maybe_unused]] const std::vector<State>& states) const {
    std::vector<InterventionBundle<Logistics>> interventions {
        // Identity
        InterventionBundle<Logistics>(getIdentityIntervention(), interventionCost)
    };

    // loop over all edges to find somewhere to travel to or block
    for (const Edge& edge : edgeLookup[observerLoc]) {
      // travel
      interventions.emplace_back(
          Intervention(edge.toLoc, Intervention::DRIVE), interventionCost);
      // block
      interventions.emplace_back(
          Intervention(edge.toLoc, Intervention::BLOCK), interventionCost);
    }

    return interventions;
  }

  [[nodiscard]] static Intervention getIdentityIntervention() {
    return Intervention();
  }

  /**
   * This method will mutate the domain
   * @param intervention
   * @param subjectState Current state of subject (checks legal interventions)
   * @return The patch that was applied. Will not have value on failure
   * Note that the only intervention that will return a patch with non-empty affectedStates
   * is BLOCK. In this case, the subject's state will be populated in that vector
   * instead of enumerating all possible states that could be affected by it
   */
  std::optional<Patch> applyIntervention(const Intervention& intervention, const State& subjectState) {
    // ignore identity
    if (intervention.type == Intervention::IDENTITY) {
      return {Patch(intervention)};
    }

    // Drive
    if (intervention.type == Intervention::DRIVE) {
      for (const Edge& edge : edgeLookup[observerLoc]) {
        if (edge.toLoc == intervention.locationId) {
          auto patch = Patch(intervention, observerLoc);
          observerLoc = edge.toLoc;
          return {patch};
        }
      }
    }

    // Block
    if (intervention.type == Intervention::BLOCK) {
      auto& edgeList = edgeLookup[observerLoc];
      for (auto edgeIt = edgeList.before_begin();
           std::next(edgeIt) != edgeList.end();
           edgeIt = std::next(edgeIt)) {
        Edge& nextEdge = *std::next(edgeIt);
        if (nextEdge.toLoc == intervention.locationId) {
          // remove edge from lookup
          edgeList.erase_after(edgeIt);
          return {Patch(intervention, nextEdge, subjectState)};
        }
      }
    }

    // unknown intervention or one of above failed
    return {};
  }

  /**
   * This method will mutate the domain
   */
  void reversePatch(const Patch& patch, [[maybe_unused]] const State& subjectState) {
    if (patch.intervention.type == Intervention::DRIVE) {
      observerLoc = patch.originLocId;
    } else if (patch.intervention.type == Intervention::BLOCK) {
      edgeLookup[observerLoc].push_front(patch.blockedEdge);
    }
    // ignore identity
  }

 private:
  // Private utility functions
  /** Returns false if EOF, true if success */
  static bool getNextLine(std::string& line, std::istream& input) {
    do {
      if(!getline(input, line)) {
        return false;
      }
    } while (line.empty());
    return true;
  }

  [[nodiscard]] static std::vector<std::string> split(const std::string& str, const char delim) {
    std::vector<std::string> out;
    size_t start = 0, end = 0;
    while ((start = str.find_first_not_of(delim, end)) != std::string::npos) {
      end = str.find(delim, start);
      out.push_back(str.substr(start, end - start));
    }

    return out;
  }

  /**
   * Computes a lower bound distance in time steps between 2 states, including
   * one that is underspecified.
   * - +2 for each goal pkg not at its goal location (load/unload)
   *   - +min edge cost from pkg loc for each goal pkg not at goal loc (drive)
   * - +1 for each goal pkg in a truck (mutually exclusive w/ others)
   *
   * Note: I don't think this is a great heuristic, but it'll get the job done
   * Also note, would be unwise to use this if there are a lot of packages in the domain
   * @param state
   * @param underspecifiedState
   * @return
   */
  [[nodiscard]] Cost goalDistance(const State& state, const State& underspecifiedState) const {
    Cost minDistance = 0;
    for (size_t i = 0; i < numPkgs; i++) {
      size_t goalPkgLoc = underspecifiedState.pkgLocations[i];
      if (goalPkgLoc == NULL_ID) {
        continue; // not part of goal spec, so we ignore measures
      }

      // Package is in transit
      if (state.pkgLocations[i] == NULL_ID) {
        minDistance++;
        continue;
      }

      const auto& edgeList = edgeLookup[state.pkgLocations[i]];
      // if edge list is empty, it means this goal cannot be achieved
      if (edgeList.empty()) {
        // divide by action duration so that it doesn't overflow
        return std::numeric_limits<Cost>::max() / actionDuration;
      }
      // Package not in transit. Must at least LOAD/UNLOAD and travel
      minDistance += 3;
    }
    return minDistance;
  }


  void addValidSuccessor(std::vector<SuccessorBundle<Logistics>>& successors,
                        const State& state, Action action) const {
    std::optional<State> successor = transition(state, action);
    if (successor.has_value()) {
      successors.emplace_back(successor.value(), action, actionDuration);
    }
  }

  // Action functions
  /**
   * Loads package onto truck
   * @param state
   * @param action
   * @return
   */
  [[nodiscard]] static std::optional<const State> loadPkg(const State& state, const Action& action) {
    State successor{state};
    if (successor.pkgLocations[action.pkgId] != successor.truckLocations[action.truckId]) {
      return {}; // pkg not at location
    }
    successor.pkgLocations[action.pkgId] = NULL_ID;
    for (size_t& pkgSlot : successor.truckLoads[action.truckId]) {
      if (pkgSlot == NULL_ID) {
        pkgSlot = action.pkgId;

        return successor;
      }
    }
    return {}; // no capacity
  }

  /**
   * Unloads a package from a truck
   * @param state
   * @param action
   * @return
   */
  [[nodiscard]] static std::optional<const State> unloadPkg(const State& state, const Action& action) {
    State successor{state};
    for (size_t& pkgSlot : successor.truckLoads[action.truckId]) {
      if (pkgSlot == action.pkgId) {
        pkgSlot = NULL_ID;
        successor.pkgLocations[action.pkgId] = successor.truckLocations[action.truckId];

        return successor;
      }
    }
    return {}; // truck does not have package
  }

  [[nodiscard]] std::optional<const State> drive(const State& state, const Action& action) const {
    State successor{state};

    size_t truckLocation = successor.truckLocations[action.truckId];
    for (const Edge& edge : edgeLookup[truckLocation]) {
      if (edge.toLoc == action.locationId) {
        successor.truckLocations[action.truckId] = edge.toLoc;

        return successor;
      }
    }
    return {}; // no edge to successor exists
  }

  // Internal state of domain

  // Basic state
  size_t numTrucks;
  size_t numPkgs;
  size_t numLocations;

  /** lookup for edges available from a location. Index is location ID */
  std::vector<std::forward_list<Edge>> edgeLookup{};

  // Goal state

  /** Possible goals (initial hypothesis) */
  std::vector<State> goals{};
  /** Actual goal */
  State singleGoal;
  bool useSingleGoal = false;

  State startState{};

  // Observer
  size_t observerLoc = NULL_ID;

  // Configuration
  const Cost actionDuration;
  /** Cost of executing intervention */
  const Cost interventionCost;
  const double heuristicMultiplier;
};

}  // namespace metronome
