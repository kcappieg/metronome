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

    int relativeX() const {
      if (label == 'N') return 0;
      if (label == 'S') return 0;
      if (label == 'W') return -1;
      if (label == 'E') return 1;
      if (label == '0') return 0;
      return 0;
    }

    int relativeY() const {
      if (label == 'N') return -1;
      if (label == 'S') return 1;
      if (label == 'W') return 0;
      if (label == 'E') return 0;
      if (label == '0') return 0;
      return 0;
    }

    Action inverse() const {
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

    char toChar() const { return label; }

    friend std::ostream& operator<<(std::ostream& os, const Action& action) {
      os << action.label << " (dx: " << action.relativeX()
         << " dy: " << action.relativeY() << ")";
      return os;
    }

   private:
    char label;
  };

  class State {
   public:
    State() : x(0), y(0) {}
    State(unsigned int x, unsigned int y) : x{x}, y{y} {}
    /*Standard getters for the State(x,y)*/
    unsigned int getX() const { return x; }
    unsigned int getY() const { return y; }
    std::size_t hash() const { return x ^ y << 16 ^ y >> 16; }

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

    friend std::ostream& operator<<(std::ostream& stream,
                                    const GridWorld::State& state) {
      stream << "x: " << state.getX() << " y: " << state.getY();
      return stream;
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
      IDENTITY
    };

    Intervention(const State& state, Type interventionType) : obstacle(state), interventionType(interventionType) {}
    Intervention(): obstacle{}, interventionType(IDENTITY) {}

    const State obstacle;
    /** if true, adding obstacle. if false, removing */
    const Type interventionType;

    std::size_t hash() const {
      size_t seed = 37;

      seed = (seed * 17) ^ obstacle.hash();
      seed = (seed * 17) ^ std::hash<int>{}(interventionType);
      return seed;
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
    Patch(const Intervention intervention = {}, const std::vector<State> affectedStates = {})
      : intervention(intervention), affectedStates(affectedStates) {}

    const Intervention intervention;
    const std::vector<State> affectedStates;
  };

  /*Entry point for using this Domain*/
  GridWorld(const Configuration& configuration, std::istream& input)
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

    std::optional<State> tempStarState{};

    while (getline(input, line) && currentHeight < height) {
      for (char it : line) {
        switch (it) {
          case '@': // find the start location
            tempStarState = State(currentWidth, currentHeight);
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
//          default: // its an open cell nothing needs to be done
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

    if (!tempStarState.has_value() || goalLocations.size() == 0) {
      throw MetronomeException(
          "GridWorld unknown start or goal location. Start or goal location is "
          "not defined.");
    }

    startLocation = tempStarState.value();
    // If only 1 goal, set the singleGoal property
    if (goalLocations.size() == 1) {
      useSingleGoal = true;
      singleGoal = *goalLocations.cbegin();
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

  bool isGoal(const State& location, const State& goalLocation) const {
    return location == goalLocation;
  }

  const std::vector<State> getGoals() const {
    return goalVector;
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

  bool isStart(const State& state) const {
    return state.getX() == startLocation.getX() &&
           state.getY() == startLocation.getY();
  }

  Cost distance(const State& state, const State& other) const {
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

  bool safetyPredicate(const State&) const { return true; }

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
  void setCurrentGoal(State goal) {
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
   * Takes a vector of states and returns all interventions applicable to those states
   * NOTE: Only supports removing edges (adding obstacles) as of right now. Consider supporting add edges as well
   * NOTE: This might return a lot of interventions. Efficient algorithms will curate the states passed to this
   * function
   * @param states
   * @return
   */
  std::vector<InterventionBundle<GridWorld>> interventions(const std::vector<State>& states) const {
    std::vector<InterventionBundle<GridWorld>> interventions;
    std::unordered_set<State, metronome::Hash<State>> newObstacles;

    for (auto& state : states) {
      if (isValidState(state)) {
        newObstacles.insert(state);
      }
      for (auto& succ : successors(state)) {
        newObstacles.insert(succ.state);
      }
    }

    for (auto obstacle : newObstacles) {
      if (possibleInterventions.count(obstacle)) {
        interventions.emplace_back(possibleInterventions.at(obstacle), interventionCost);
      }
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
   * @return Optional Patch. If blank, the
   */
  std::optional<Patch> applyIntervention(const Intervention& intervention, const State& subjectState) {
    std::vector<State> affected;

    // ignore identity
    if (intervention.interventionType == Intervention::Type::IDENTITY) {
      return std::optional{Patch({}, {})};
    }

    if (intervention.interventionType == Intervention::Type::ADD) {
      // invalid intervention - no value
      if (subjectState == intervention.obstacle) return {};

      // If no obstacles added, that means it was invalid
      if (!addObstacle(intervention.obstacle)) return {};
    } else if (intervention.interventionType == Intervention::Type::REMOVE) {
      // invalid intervention - no value
      if (!isObstacle(intervention.obstacle)) return {};

      obstacles.erase(intervention.obstacle);
    }

    affected.emplace_back(intervention.obstacle);
    for (auto& bundle : successors(intervention.obstacle)) {
      affected.emplace_back(bundle.state);
    }

    return std::optional{Patch({intervention}, affected)};
  }

  /**
   * Reverses a patch by re-adding or removing obstacles
   * @param patch
   * @param subjectState Checking validity of reversal - can't re-add an obstacle on top of a subject
   */
  void reversePatch(const Patch& patch, const State& subjectState) {
      if (patch.intervention.interventionType == Intervention::Type::REMOVE) {
        if (subjectState == patch.intervention.obstacle) {
          throw MetronomeException("Attempted to add obstacle in the subject's state");
        }

        obstacles.insert(patch.intervention.obstacle);
      } else if (patch.intervention.interventionType == Intervention::Type::ADD) {
        obstacles.erase(patch.intervention.obstacle);
      }
      // ignore identity
  }

  void visualize(std::ostream& display) const {
    for (unsigned int i = 0; i < height; ++i) {
      for (unsigned int j = 0; j < width; ++j) {
        if (startLocation.getX() == j && startLocation.getY() == i) {
          display << '@';
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

  Intervention getIdentityIntervention() const {
    return Intervention();
  }


  unsigned int getWidth() { return width; }
  unsigned int getHeight() { return height; }

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
