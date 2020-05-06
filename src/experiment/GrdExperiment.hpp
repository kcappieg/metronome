//
// Created by kevin on 5/5/20.
//

#ifndef METRONOME_GRDEXPERIMENT_HPP
#define METRONOME_GRDEXPERIMENT_HPP

#include "Experiment.hpp";

#include <utility>

namespace metronome{

template <typename Domain, typename GrdPlanner, typename SubjectPlanner>
class GrdExperiment : public Experiment<Domain, GrdPlanner> {
public:
  explicit GrdExperiment(Configuration configuration) : configuration(std::move(configuration)) {}

  Result plan(const Configuration&, const Domain&, GrdPlanner&) override {
    return {configuration, "Invalid signature for GRD Experiment"};
  }
  Result plan(const Configuration& configuration,
              const Domain& domain,
              GrdPlanner& grdPlanner,
              SubjectPlanner& subjectPlanner) {

    // Implement loop - get first action from subject planner, then get intervention. Repeat

    return Result(configuration, "Not implemented");
  }
private:
  const Configuration configuration;
};

} // namespace metronome

#endif //METRONOME_GRDEXPERIMENT_HPP
