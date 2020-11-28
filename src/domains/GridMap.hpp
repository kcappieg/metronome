#pragma once

#include <string>
#include <istream>

#include "GridWorld.hpp"
#include "utils/String.hpp"

namespace metronome {

class GridMap : public GridWorld {
 public:
  GridMap(const Configuration& configuration,
          std::istream& mapInput, std::istream& sceneInput)
  : GridWorld(configuration, mapInput, true) {
    std::string line;
    bool notEof;

    getNextLine(line, sceneInput);
    if (line.rfind(AGENT_PRE, 0) != 0) {
      throw MetronomeException("No agent info specified");
    }

    // skip "Agent:"
    line = line.substr(AGENT_PRE.size());
    auto startState = ptStringToState(line);
    setStartState(startState);

    // Get goals
    notEof = getNextLine(line, sceneInput);
    if (!notEof || line.rfind(GOALS_PRE) != 0) {
      throw MetronomeException("No goals specified");
    }

    std::vector<State> goalStates{};
    notEof = getNextLine(line, sceneInput);
    while (notEof) {
      if (line.rfind(OBSERVER_PRE, 0) == 0) {
        // next section
        break;
      }
      goalStates.push_back(ptStringToState(line));

      notEof = getNextLine(line, sceneInput);
    }
    addGoals(goalStates);

    // no observer info, just return
    if (!notEof) {
      return;
    }

    // Skip "Observer:"
    line = line.substr(OBSERVER_PRE.size());
    setObserverLocation(ptStringToState(line));

    // Get interventions
    getNextLine(line, sceneInput);
    if (line.rfind(INTERVENTIONS_PRE, 0) != 0) {
      throw MetronomeException("No interventions specified for Observer");
    }

    std::vector<State> interventions{};
    notEof = getNextLine(line, sceneInput);
    while (notEof) {
      interventions.push_back(ptStringToState(line));
      notEof = getNextLine(line, sceneInput);
    }

    addPossibleInterventions(interventions);
  }

  /**
   * Need to convert successor bundles from GridWorld to GridMap
   */
  std::vector<SuccessorBundle<GridMap>> successors(const State& state) const {
    std::vector<SuccessorBundle<GridMap>> successors;
    for (auto& successor : GridWorld::successors(state)) {
      successors.emplace_back(successor.state, successor.action, successor.actionCost);
    }
    return successors;
  }

  /**
   * Need to convert intervention bundles from GridWorld to GridMap
   */
  std::vector<InterventionBundle<GridMap>> interventions(
  const State& subjectState,
  const std::vector<State>& lookaheadStates) const {
    std::vector<InterventionBundle<GridMap>> interventions{};
    for (auto& intervention : GridWorld::interventions(subjectState, lookaheadStates)) {
      interventions.emplace_back(intervention.intervention, intervention.interventionCost);
    }

    return interventions;
  }

 private:
  State ptStringToState(const std::string& ptString) {
    auto ptStrings = split(ptString.substr(1, ptString.size() - 1), ',');
    return State(std::stoul(ptStrings[0]), std::stoul(ptStrings[1]));
  }

  // String constants for scene parsing
  const std::string AGENT_PRE = "Agent:";
  const std::string GOALS_PRE = "Goals:";
  const std::string OBSERVER_PRE = "Observer:";
  const std::string INTERVENTIONS_PRE = "Interventions:";

};
} // namespace metronome
