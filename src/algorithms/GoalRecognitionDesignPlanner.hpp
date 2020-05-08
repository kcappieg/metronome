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
    public:
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

    virtual ~GoalRecognitionDesignPlanner() override = default;
    virtual std::vector<typename GoalRecognitionDesignPlanner<Domain>::InterventionBundle> selectInterventions(
            const typename Domain::State& subjectState, const Domain& currentDomain
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

  private:
    std::vector<std::unordered_map<std::string, std::int64_t>>
            iterationAttributes;
    std::size_t iterationCount = 0;
  };

} // namespace metronome

#endif //METRONOME_GOALRECOGNITIONDESIGNPLANNER_HPP
