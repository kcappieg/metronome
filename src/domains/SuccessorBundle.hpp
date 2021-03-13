#pragma once

#include <ostream>

namespace metronome {

template <typename Domain>
class SuccessorBundle {
 public:
  typedef typename Domain::State State;
  typedef typename Domain::Action Action;
  typedef typename Domain::Cost Cost;

  SuccessorBundle(State state, Action action, Cost actionCost)
      : state(state), action(action), actionCost(actionCost) {}

  State state;
  Action action;
  Cost actionCost;

    friend std::ostream& operator<<(std::ostream& os, const SuccessorBundle& bundle) {
      os << "SuccessorBundle - state: " << bundle.state
         << ", action: " << bundle.action
         << ", cost: " << bundle.actionCost;
      return os;
    }

    friend void swap(SuccessorBundle& lhs, SuccessorBundle& rhs) {
      using std::swap;
      swap(lhs.state, rhs.state);
      swap(lhs.action, rhs.action);
      swap(lhs.actionCost, rhs.actionCost);
    }
};

template <typename Domain>
class InterventionBundle {
 public:
  typedef typename Domain::Intervention Intervention;
  typedef typename Domain::Cost Cost;

  InterventionBundle(const Intervention intervention, Cost interventionCost)
    : intervention(intervention), interventionCost(interventionCost) {}

  const Intervention intervention;
  const Cost interventionCost;

  friend std::ostream& operator<<(std::ostream& os, const InterventionBundle& bundle) {
    os << "InterventionBundle - intervention: " << bundle.intervention
      << ", cost: " << bundle.interventionCost;
    return os;
  }
};

}  // namespace metronome
