//
// Created by kevin on 5/2/20.
//

#ifndef METRONOME_GOALRECOGNITIONDESIGNPLANNER_HPP
#define METRONOME_GOALRECOGNITIONDESIGNPLANNER_HPP

#include <vector>
#include "Planner.hpp"
#include "domains/SuccessorBundle.hpp"

namespace metronome {

  template<typename Domain>
  class GoalRecognitionDesignPlanner : public Planner<Domain> {
  public:
    virtual ~GoalRecognitionDesignPlanner() override = default;
    virtual std::vector<InterventionBundle<Domain>> selectInterventions(
            const typename Domain::State& subjectState, const Domain& systemState
    ) = 0;

    // Attributes / Iterations
    virtual void beginIteration() final {
      ++iterationCount;

      iterationAttributes.emplace_back();
    }

    virtual void recordAttribute(const std::string key,
                                 std::int64_t value) final {
      assert(iterationAttributes.size() > iterationCount - 1);
      iterationAttributes[iterationCount - 1][key] = value;
    }

    virtual std::vector<std::pair<std::string, std::int64_t>> getAttributes() const override {
      // Calculate sum
      std::unordered_map<std::string, std::int64_t> attributeSums;
      std::unordered_map<std::string, std::int64_t> attributeCount;

      for (const auto& attributes : iterationAttributes) {
        for (const auto& attribute : attributes) {
          attributeSums[attribute.first] += attribute.second;
          ++attributeCount[attribute.first];
        }
      }

      auto attributes = Planner<Domain>::getAttributes();
      // Calculate aggregation
      for (auto& attributeSum : attributeSums) {
        const auto sum = attributeSum.second;

        std::int64_t agg{0};

        // find average based on number of iterations this attribute was recorded
        const auto frequency = attributeCount[attributeSum.first];
        agg = sum / frequency;

        attributes.emplace_back(attributeSum.first, agg);
      }

      return attributes;
    }

    /**
     * Must be implemented so that experiment runner can handle situation where the
     * GRD planner does not return any interventions.
     */
    virtual InterventionBundle<Domain> getIdentityIntervention(const Domain& systemState) const {
      return {systemState.getIdentityIntervention(), 1};
    }

    /**
     * Return current prediction of subject's goal
     * TODO: May need to refactor this to output goal sets with probabilities
     * @param systemState
     * @param subjectState
     * @return
     */
    virtual std::optional<typename Domain::State> getGoalPrediction(
            const Domain& systemState, const typename Domain::State& subjectState) = 0;

  protected:
    std::size_t iterationCount = 0;
  private:
    std::vector<std::unordered_map<std::string, std::int64_t>>
            iterationAttributes;
  };

} // namespace metronome

#endif //METRONOME_GOALRECOGNITIONDESIGNPLANNER_HPP
