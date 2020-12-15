#pragma once

#include <algorithm>
#include <cstdlib>
#include <experiment/Configuration.hpp>
#include <functional>
#include <limits>
#include <optional>
#include <ostream>
#include <sstream>
#include <unordered_set>
#include <utils/Hash.hpp>
#include <utility>
#include <vector>
#include "MetronomeException.hpp"
#include "SuccessorBundle.hpp"

namespace metronome {
class GridWorld {
 public:
  typedef long long int Cost;

  class Action {
   public:
    Action() : label{'~'} {};
    explicit Action(char label) : label{label} {}
    Action(const Action&) = default;
    Action(Action&&) = default;
    Action& operator=(const Action&) = default;
    ~Action() = default;

    static std::vector<Action>& getActions() {
      static std::vector<Action> actions{
          Action('N'), Action('S'), Action('W'), Action('E')};
      return actions;
    }

    [[nodiscard]] int relativeX() const {
      if (label == 'N') return 0;
      if (label == 'S') return 0;
      if (label == 'W') return -1;
      if (label == 'E') return 1;
      if (label == '0') return 0;
      return 0;
    }

    [[nodiscard]] int relativeY() const {
      if (label == 'N') return -1;
      if (label == 'S') return 1;
      if (label == 'W') return 0;
      if (label == 'E') return 0;
      if (label == '0') return 0;
      return 0;
    }

    [[nodiscard]] Action inverse() const {
      if (label == 'N') return Action('S');
      if (label == 'S') return Action('N');
      if (label == 'W') return Action('E');
      if (label == 'E') return Action('W');
      if (label == '0') return Action('0');

      throw MetronomeException("Unknown action to invert: " +
                               std::to_string(label));
    }

    bool operator==(const Action& rhs) const { return label == rhs.label; }

    bool operator!=(const Action& rhs) const { return !(rhs == *this); }

    [[nodiscard]] char toChar() const { return label; }

    std::string toString() const {
      std::ostringstream actionStr{};
      actionStr << label << " (dx: " << relativeX()
                << " dy: " << relativeY() << ")";

      return actionStr.str();
    }


    friend std::ostream& operator<<(std::ostream& os, const Action& action) {
      return os << action.toString();
    }

   private:
    char label;
  };

  class State {
   public:
    State() : x(0), y(0) {}
    State(unsigned int x, unsigned int y) : x{x}, y{y} {}
    /*Standard getters for the State(x,y)*/
    [[nodiscard]] unsigned int getX() const { return x; }
    [[nodiscard]] unsigned int getY() const { return y; }
    [[nodiscard]] std::size_t hash() const { return x ^ y << 16 ^ y >> 16; }

    bool operator==(const State& state) const {
      return x == state.x && y == state.y;
    }

    bool operator!=(const State& state) const {
      return x != state.x || y != state.y;
    }

    State& operator=(State toCopy) {
      swap(*this, toCopy);
      return *this;
    }

    [[nodiscard]] std::string toString() const {
      std::ostringstream stateStr{};
      stateStr << "x: " << getX() << " y: " << getY();

      return stateStr.str();
    }

    friend std::ostream& operator<<(std::ostream& stream,
                                    const GridWorld::State& state) {
      return stream << state.toString();
    }

   private:
    /*State(x,y) representation*/
    unsigned int x;
    unsigned int y;
    /*Function facilitating operator==*/
    friend void swap(State& first, State& second) {
      using std::swap;
      swap(first.x, second.x);
      swap(first.y, second.y);
    }
  };

  class Intervention {
   public:
    enum Type {
      ADD,
      REMOVE,
      MOVE,
      IDENTITY
    };

    Intervention(const State& state, Type interventionType)
        : obstacle(state), action{}, interventionType(interventionType) {}
    explicit Intervention(const Action& action)
        : obstacle{}, action(action), interventionType(MOVE) {}
    Intervention(): obstacle{}, action{}, interventionType(IDENTITY) {}

    const State obstacle;
    const Action action;
    const Type interventionType;

    [[nodiscard]] std::size_t hash() const {
      size_t seed = 37;

      seed = (seed * 17) ^ obstacle.hash();
      seed = (seed * 17) ^ std::hash<int>{}(interventionType);
      return seed;
    }

    bool operator==(const Intervention& other) const {
      return other.interventionType == interventionType && other.obstacle == obstacle;
    }

    friend std::ostream& operator<<(std::ostream& stream, const GridWorld::Intervention& intervention) {
      std::ostringstream interventionStr;
      switch(intervention.interventionType) {
        case ADD:
          interventionStr << "Add obstacle " << intervention.obstacle;
          break;
        case REMOVE:
          interventionStr << "Remove obstacle " << intervention.obstacle;
          break;
        case IDENTITY:
          interventionStr << "Identity";
          break;
        case MOVE:
          interventionStr << "Move " << intervention.action;
      }

      stream << interventionStr.str();
      return stream;
    }
  };

  /**
   * Represents an atomic "patch" of one or more interventions
   */
  class Patch {
   public:
    explicit Patch(Intervention intervention = {}, std::vector<State> affectedStates = {})
      : intervention(std::move(intervention)), affectedStates(std::move(affectedStates)) {}

    const Intervention intervention;
    const std::vector<State> affectedStates;
  };

  /**
   * Reads grid map from input stream
   * @param configuration
   * @param input
   * @param lazyScene If specified, allows for lazy instantiation of the scene:
   * goals and agent start position can be specified later
   */
  GridWorld(const Configuration& configuration, std::istream& input,
            bool lazyScene = false)
      : actionDuration(configuration.getLong(ACTION_DURATION, 1)),
        interventionCost(configuration.getLong(INTERVENTION_COST, 1)),
        heuristicMultiplier(configuration.getDouble(HEURISTIC_MULTIPLIER, 1)) {
    unsigned int currentHeight = 0;
    unsigned int currentWidth = 0;
    std::string line;
    char* end;
    getline(input, line);  // get the width

    std::stringstream convertWidth(line);
    if (std::strtol(line.c_str(), &end, 10) == 0) {
      throw MetronomeException("GridWorld first line must be a number.");
    }

    convertWidth >> width;
    getline(input, line);  // get the height
    if (std::strtol(line.c_str(), &end, 10) == 0) {
      throw MetronomeException("GridWorld second line must be a number.");
    }

    std::stringstream convertHeight(line);
    convertHeight >> height;

    std::optional<State> tempStartState{};

    while (getline(input, line) && currentHeight < height) {
      for (char it : line) {
        switch (it) {
          case '@': // find the start location
            tempStartState = State(currentWidth, currentHeight);
            break;
          case '*': // found a goal location
            goalLocations.emplace(currentWidth, currentHeight);
            goalVector.emplace_back(currentWidth, currentHeight);
            break;
          case '#': // store the objects
            obstacles.insert({currentWidth, currentHeight});
            break;
          case 'A': // State where an obstacle can be added
            possibleInterventions.insert(std::pair<State, Intervention>{
              State(currentWidth, currentHeight),
              Intervention(State(currentWidth, currentHeight), Intervention::Type::ADD)
            });
            break;
          case 'O':
          case 'o':
            observerLocation = State(currentWidth, currentHeight);
            break;
          default: // its an open cell nothing needs to be done
            break;
        }
        if (currentWidth == width) break;

        ++currentWidth;  // at the end of the character parse move along
      }
      if (currentWidth < width) {
        throw MetronomeException(
            "GridWorld is not complete. Width doesn't match input "
            "configuration.");
      }

      currentWidth = 0;  // restart character parse at beginning of line
      ++currentHeight;   // move down one line in character parse
    }

    if (currentHeight != height) {
      throw MetronomeException(
          "GridWorld is not complete. Height doesn't match input "
          "configuration.");
    }

    if (!lazyScene &&
        (!tempStartState.has_value() || goalLocations.empty())) {
      throw MetronomeException(
          "GridWorld unknown start or goal location. Start or goal location is "
          "not defined.");
    } else {
      if (tempStartState.has_value()) {
        startLocation = tempStartState.value();
      }
      // If only 1 goal, set the singleGoal property
      if (goalLocations.size() == 1) {
        useSingleGoal = true;
        singleGoal = *goalLocations.cbegin();
      }
    }
  }

  /**
   * Transition to a new state from the given state applying the given action.
   *
   * @return the original state if the transition is not possible
   */
  std::optional<State> transition(const State& sourceState,
                                  const Action& action) const {
    State targetState(sourceState.getX() + action.relativeX(),
                      sourceState.getY() + action.relativeY());

    if (isValidState(targetState)) {
      return targetState;
    }

    return {};
  }

  /*Validating a goal state*/
  bool isGoal(const State& location) const {
    if (useSingleGoal) {
      return singleGoal == location;
    } else {
      return goalLocations.count(location) > 0;
    }
  }

  static bool isGoal(const State& location, const State& goalLocation) {
    return location == goalLocation;
  }

  /**
   *
   * @return copy of goal vector
   */
  std::vector<State> getGoals() const {
    return goalVector;
  }

  /**
   * Allows for lazy-instantiation of goals
   * @param newGoalLocations
   */
  void addGoals(std::vector<State> newGoalLocations) {
    goalLocations.insert(newGoalLocations.begin(), newGoalLocations.end());
    goalVector.insert(goalVector.end(), newGoalLocations.begin(), newGoalLocations.end());

    if (goalLocations.size() == 1) {
      useSingleGoal = true;
      singleGoal = *goalLocations.cbegin();
    }
  }

  /*Validating an obstacle state*/
  bool isObstacle(const State& location) const {
    return obstacles.find(location) != obstacles.end();
  }

  /*Validating the agent can visit the state*/
  bool isValidState(const State& state) const {
    return state.getX() < width && state.getY() < height
           && !isObstacle(state);
  }

  /*Standard getters for the (width,height) of the domain*/
  unsigned int getWidth() const { return width; }
  unsigned int getHeight() const { return height; }

  /*Adding an obstacle to the domain*/
  bool addObstacle(const State& toAdd) {
    if (isValidState(toAdd)) {
      obstacles.insert(toAdd);
      return true;
    }
    return false;
  }

  std::vector<State>::size_type getNumberObstacles() {
    return obstacles.size();
  }

  State getStartState() const { return startLocation; }

  /**
   * For lazy scene initialization
   * @param startState
   */
  void setStartState(const State& startState) {
    startLocation = startState;
  }

  [[maybe_unused]] bool isStart(const State& state) const {
    return state.getX() == startLocation.getX() &&
           state.getY() == startLocation.getY();
  }

  static Cost distance(const State& state, const State& other) {
    unsigned int verticalDistance = std::max(other.getY(), state.getY()) -
                                    std::min(other.getY(), state.getY());
    unsigned int horizontalDistance = std::max(other.getX(), state.getX()) -
                                      std::min(other.getX(), state.getX());
    unsigned int totalDistance = verticalDistance + horizontalDistance;
    Cost manhattanDistance = static_cast<Cost>(totalDistance);
    return manhattanDistance;
  }

  Cost distance(const State& state) const {
    if (useSingleGoal) {
      return distance(state, singleGoal);
    } else {
      throw MetronomeException("Goal ambiguous - Distance function invoked with implicit goal, but goal set is greater than one.");
    }
  }

  Cost heuristic(const State& state) const {
    return distance(state) * actionDuration;
  }

  Cost heuristic(const State& state, const State& other) const {
    return distance(state, other) * actionDuration;
  }

  [[maybe_unused]] static bool safetyPredicate(const State&) { return true; }

  std::vector<SuccessorBundle<GridWorld>> successors(const State& state) const {
    std::vector<SuccessorBundle<GridWorld>> successors;
    successors.reserve(4);

    for (auto& action : Action::getActions()) {
      addValidSuccessor(
          successors, state, action.relativeX(), action.relativeY(), action);
    }

    return successors;
  }

  // GRD-supporting Methods

  /**
   * Set the goal to be used for subject planners
   * @param goal
   */
  void setCurrentGoal(const State& goal) {
    singleGoal = goal;
    useSingleGoal = true;
  }

  /**
   * Clear the subject goal so that the isGoal function can work for multiple goals
   */
  void clearCurrentGoal() {
    useSingleGoal = false;
  }

  /**
   * For lazy-initialization
   * @param interventionStates
   */
  void addPossibleInterventions(const std::vector<State>& interventionStates) {
    for (auto& state : interventionStates) {
      possibleInterventions.insert({
        state, Intervention(state, Intervention::Type::ADD)
      });
    }
  }

  /**
   * For lazy initialization. Do not use in GRD algorithm!
   */
  void setObserverLocation(const State& observerLoc) {
    observerLocation = observerLoc;
  }

  /**
   * Takes a vector of states and returns all interventions applicable to those states.
   * Determines applicable actions based on the observer's current location.
   * NOTE: Only supports removing edges (adding obstacles) as of right now. Consider supporting add edges as well
   * @param states
   * @return
   */
  std::vector<InterventionBundle<GridWorld>> interventions(
      [[maybe_unused]] const State& subjectState,
      const std::vector<State>& lookaheadStates) const {
    std::vector<InterventionBundle<GridWorld>> interventions;
    std::unordered_set<State, metronome::Hash<State>> stateInterventions;

    if (!observerLocation.has_value()) {
      throw MetronomeException("No observer location detected in AGRD setting");
    }

    for (auto& state : lookaheadStates) {
      if (isValidState(state) && possibleInterventions.count(state)) {
        stateInterventions.insert(state);
      }
    }

    // TODO: this basically requires the lookahead states. Should create branch where it is not required
    // Add all possible movement to intervention vector
    for (SuccessorBundle<GridWorld>& successor : successors(*observerLocation)) {
      // If a successor is also a valid obstacle location, add that to interventions
      if (stateInterventions.count(successor.state)) {
        interventions.emplace_back(
            possibleInterventions.at(successor.state),
            interventionCost);
      }

      interventions.emplace_back(
          Intervention(successor.action), successor.actionCost);
    }

    // Identify intervention - does nothing
    interventions.emplace_back(getIdentityIntervention(), interventionCost);
    return interventions;
  }

  /**
   * Add / remove obstacles to the domain given the passed intervention.
   * The patch includes all states adjacent to the obstacle as well as the obstacle
   * state itself in the affectedStates property
   * @param intervention
   * @param subjectState Current state of subject (checks legal interventions)
   * @return Optional Patch. If blank, the operation was invalid
   */
  [[maybe_unused]] std::optional<Patch> applyIntervention(
      const Intervention& intervention, const State& subjectState) {
    std::vector<State> affected;

    // ignore identity
    if (intervention.interventionType == Intervention::Type::IDENTITY) {
      return std::optional{Patch()};
    }

    if (intervention.interventionType == Intervention::Type::MOVE) {
      auto newState = transition(*observerLocation, intervention.action);
      if (newState.has_value()) {
        observerLocation = newState;
        // no states were affected to report to the subject
        return std::optional{Patch(intervention, {})};
      } else {
        // Move not valid. Shouldn't happen, but ignore anyway
        return {};
      }
    }

    // Adding and removing obstacles
    if (intervention.interventionType == Intervention::Type::ADD) {
      // invalid intervention - no value
      if (subjectState == intervention.obstacle) return {};

      // If no obstacles added, that means it was invalid
      if (!addObstacle(intervention.obstacle)) return {};

      affected.emplace_back(intervention.obstacle);
    } else if (intervention.interventionType == Intervention::Type::REMOVE) {
      // invalid intervention - no value
      if (!isObstacle(intervention.obstacle)) return {};

      obstacles.erase(intervention.obstacle);
    }

    for (auto& bundle : successors(intervention.obstacle)) {
      affected.emplace_back(bundle.state);
    }

    return std::optional{Patch(intervention, affected)};
  }

  /**
   * Reverses a patch by re-adding or removing obstacles
   * @param patch
   * @param subjectState Checking validity of reversal - can't re-add an obstacle on top of a subject
   */
   [[maybe_unused]] void reversePatch(const Patch& patch, const State& subjectState) {
      if (patch.intervention.interventionType == Intervention::Type::REMOVE) {
        if (subjectState == patch.intervention.obstacle) {
          throw MetronomeException("Attempted to add obstacle in the subject's state");
        }

        obstacles.insert(patch.intervention.obstacle);
      } else if (patch.intervention.interventionType == Intervention::Type::ADD) {
        obstacles.erase(patch.intervention.obstacle);
      } else if (patch.intervention.interventionType == Intervention::Type::MOVE) {
        Action reverseAction = patch.intervention.action.inverse();
        auto newState = transition(*observerLocation, reverseAction);
        if (newState.has_value()) {
          observerLocation = newState;
        } else {
          throw MetronomeException("Attempted to backtrack the observer agent into an obstacle");
        }
      }
      // ignore identity
  }

  void visualize(std::ostream& display, std::optional<State> state = {}) const {
    auto agentState = state.has_value() ? state.value() : startLocation;
    for (unsigned int i = 0; i < height; ++i) {
      for (unsigned int j = 0; j < width; ++j) {
        if (agentState.getX() == j && agentState.getY() == i) {
          display << '@';
        } else if (observerLocation.has_value() and observerLocation->getX() == j && observerLocation->getY() == i) {
          display << 'o';
        } else if (goalLocations.count({j, i}) > 0) {
          display << '*';
        } else if (isObstacle(State(j, i))) {
          display << '#';
        } else {
          display << '_';
        }
      }
      display << "\n";
    }
    display << "\n";
  }

  Cost getActionDuration() const { return actionDuration; }
  Cost getActionDuration(const Action&) const { return actionDuration; }

  Action getIdentityAction() const { return Action('0'); }

  static Intervention getIdentityIntervention() {
    return Intervention();
  }

 private:
  void addValidSuccessor(std::vector<SuccessorBundle<GridWorld>>& successors,
                         const State& sourceState,
                         const int relativeX,
                         const int relativeY,
                         Action& action) const {
    auto successor = getSuccessor(sourceState, relativeX, relativeY);
    if (successor.has_value()) {
      successors.emplace_back(successor.value(), action, actionDuration);
    }
  }

  std::optional<State> getSuccessor(const State& sourceState,
                                    int relativeX,
                                    int relativeY) const {
    auto newX = static_cast<unsigned int>(static_cast<int>(sourceState.getX()) +
                                          relativeX);
    auto newY = static_cast<unsigned int>(static_cast<int>(sourceState.getY()) +
                                          relativeY);

    State newState = State(newX, newY);

    if (isValidState(newState)) {
      return newState;
    }

    return {};
  }

  /*
   * maxActions <- maximum number of actions
   * width/height <- internal size representation of world
   * obstacles <- stores locations of the objects in world
   * dirtyCells <- stores locations of dirt in the world
   * startLocation <- where the agent begins
   * goalLocation <- where the agent needs to end up
   * initalAmountDirty <- how many cells are dirty
   * initialCost <- constant cost value
   * obstacles <- stores references to obstacles
   */
  unsigned int width;
  unsigned int height;
  std::unordered_set<State, typename metronome::Hash<State>> obstacles;
  /** Possible interventions in this domain */
  std::unordered_map<State, Intervention, typename metronome::Hash<State>> possibleInterventions;
  State startLocation{};
  /** Current position of the observer. Part of system state for AGRD setting */
  std::optional<State> observerLocation{};
  std::unordered_set<State, typename metronome::Hash<State>> goalLocations;
  /** vector separate from goalLocations to guarantee deterministic ordering when returned in instance method */
  std::vector<State> goalVector;
  /** When true, uses a single goal (specified by singleGoal) even if there are multiple goals registered */
  bool useSingleGoal = false;
  State singleGoal{};
  const Cost actionDuration;
  const Cost interventionCost;
  const double heuristicMultiplier;
};

}  // namespace metronome
