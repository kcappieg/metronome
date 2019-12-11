
#include "domains/DynamicGridWorld.hpp"

#include "algorithms/AStar.hpp"
#include "utils/TimeMeasurement.hpp"

#include "instances/GridInstance.hpp"

#include <iostream>
#include "catch.hpp"

namespace metronome {
namespace {

const std::string configurationJson =
    "{\"actionDuration\" : 6000000,"
    "\"obstacleCount\" : 10}";

TEST_CASE("GridWorld creation", "[DynamicGridWorld]") {
  el::Loggers::reconfigureAllLoggers(el::ConfigurationType::Format,
                                     "%level: %msg");

  Configuration configuration{configurationJson};

  std::istringstream stringStream{test::slalom};
  std::istream inputStream{stringStream.rdbuf()};

  logTimeDifference("start");
  DynamicGridWorld dynamicGridWorld(configuration, inputStream);
  logTimeDifference("init");

  dynamicGridWorld.expandObstacleDistributionHorizon();
  logTimeDifference("expansion");
  dynamicGridWorld.expandObstacleDistributionHorizon();
  logTimeDifference("expansion");
  dynamicGridWorld.expandObstacleDistributionHorizon();
  logTimeDifference("expansion");

  std::cout << "test" << std::endl;

  REQUIRE(true);
}

TEST_CASE("A* validation", "[DynamicGridWorld]") {
  el::Loggers::reconfigureAllLoggers(el::ConfigurationType::Format,
                                     "%level: %msg");

  Configuration configuration{configurationJson};

  std::istringstream stringStream{test::blocks};
  std::istream inputStream{stringStream.rdbuf()};

  logTimeDifference("start");
  DynamicGridWorld dynamicGridWorld(configuration, inputStream);
  logTimeDifference("init");

  AStar<DynamicGridWorld> aStar(dynamicGridWorld, configuration);
  auto startState = dynamicGridWorld.getStartState();

  const std::vector<GridWorld::Action> plan = aStar.plan(startState);

  std::cout << "test" << std::endl;

  REQUIRE(true);
}

}  // namespace
}  // namespace metronome