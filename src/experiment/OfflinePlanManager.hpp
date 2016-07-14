#ifndef METRONOME_OFFLINEPLANMANAGER_HPP
#define METRONOME_OFFLINEPLANMANAGER_HPP

#include "util/TimeMeasurement.hpp"
#include "PlanManager.hpp"
namespace metronome {
template <typename Domain, typename Planner>
class OfflinePlanManager : PlanManager<Domain, Planner> {
public:
    Result plan(const Configuration& configuration, const Domain& domain, Planner planner) {
        auto executionTime = measureNanoTime([&configuration, &planner] {

            // TODO start state
            planner.plan(configuration);

        });

        return Result();
    }
};
}

#endif // METRONOME_OFFLINEPLANMANAGER_HPP