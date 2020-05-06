//
// Created by kevin on 5/5/20.
//

#ifndef METRONOME_NAIVEDYNAMICASTAR_H
#define METRONOME_NAIVEDYNAMICASTAR_H

#include <experiment/Configuration.hpp>
#include "DynamicPlanner.hpp"
#include "AStar.hpp"

namespace metronome {
  template<typename Domain>
  class NaiveDynamicAStar final : public DynamicPlanner<Domain> {
  public:
    NaiveDynamicAStar(const Domain& domain, const Configuration& config) : domain(domain) {}

    // TODO: implement replan method by calling A* every time
    // Rethink replan - do we want to pass states or edges? Is edges too hard? Do we
    // want to pass in the domain reference on each call to replan (probably)
    // TODO: Construct A* in replan every time
    // TODO: Grid world patch-application returns blocked states in affected states. Instead, only return states that have edges to blocked states
    // Reasoning here is that a dynamic algorithm (I'm thinking D* Lite) will want the reachable states so that it can
    // efficiently replan. Not relevant for this class, but should lock down the contract now.

  private:
    Domain domain;
  };

} // namespace metronome

#endif //METRONOME_NAIVEDYNAMICASTAR_H
