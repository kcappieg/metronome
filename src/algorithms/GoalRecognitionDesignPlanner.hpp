//
// Created by kevin on 5/2/20.
//

#ifndef METRONOME_GOALRECOGNITIONDESIGNPLANNER_HPP
#define METRONOME_GOALRECOGNITIONDESIGNPLANNER_HPP

#include <vector>
#include "Planner.hpp"

namespace metronome {

  template<typename Domain>
  class GoalRecognitionDesignPlanner : public Planner<Domain> {
  public:
    /** Mimics the Action Bundle class of Planner */
    class InterventionBundle final {
      InterventionBundle() = default;
      InterventionBundle(typename Domain::Intervention intervention,
                   typename Domain::Cost interventionCost)
              : intervention{intervention}, interventionCost{interventionCost} {}
      InterventionBundle(const InterventionBundle&) = default;
      InterventionBundle(InterventionBundle&&) = default;
      InterventionBundle& operator=(const InterventionBundle&) = default;
      InterventionBundle& operator=(InterventionBundle&&) = default;
      ~InterventionBundle() = default;

      friend std::ostream& operator<<(std::ostream& os,
                                      const InterventionBundle& bundle) {
        os << "Intervention: " << bundle.intervention
           << "; cost: " << bundle.interventionCost;

        if (bundle.label.size() > 0) {
          os << "; label: " << bundle.label;
        }
        return os;
      }

      typename Domain::Intervention intervention;
      typename Domain::Cost interventionCost;
      std::string label;
    };

    friend std::ostream& operator<<(
            std::ostream& os,
            const std::vector<typename GoalRecognitionDesignPlanner<Domain>::InterventionBundle>&
            interventionBundles) {
      for (const auto& interventionBundle : interventionBundles) {
        os << interventionBundle << "\n";
      }
      return os;
    }

    virtual ~GoalRecognitionDesignPlanner() = default;
    virtual std::vector<typename GoalRecognitionDesignPlanner<Domain>::InterventionBundle> selectInterventions(
            const typename Domain::State& subjectState
    ) = 0;
  };

} // namespace metronome

#endif //METRONOME_GOALRECOGNITIONDESIGNPLANNER_HPP
