#include "catch.hpp"
#include "domains/Traffic.hpp"
#include "easylogging++.h"
#include "experiment/ConfigurationExecutor.hpp"

namespace {
const char* json = "{\"timeLimit\" : 150000000000,\n"
                   "\"domainPath\" : "
                   "\"/input/vacuum/smallwall.vw\",\n"
                   "\"domainInstanceName\" : \"Manual test instance\",\n"
                   "\"actionDuration\" : 6000000,\n"
                   "\"domainName\" : \"TRAFFIC\",\n"
                   "\"terminationType\" : \"time\",\n"
                   "\"list\" : [1, 2],\n"
                   "\"objects\" : {\"a\": [], \"b\": {}},\n"
                   "\"algorithmName\" : \"A_STAR\"}";

const std::string resourceDir = "/home/aifs2/doylew/Public/metronome/resources";
metronome::Traffic testGrid =
        metronome::ConfigurationExecutor::extractDomain<metronome::Traffic>(metronome::Configuration(json),
                resourceDir);

TEST_CASE("Traffic basic creation test", "[Traffic]") {
    metronome::Traffic traffic = testGrid;
    std::cout << testGrid.getStartLocation().getX() << "," << testGrid.getStartLocation().getY() << std::endl;
    std::cout << testGrid.getGoalState().getX() << "," << testGrid.getGoalState().getY() << std::endl;
    std::cout << testGrid.isGoal(testGrid.getStartLocation()) << std::endl;
    std::cout << testGrid.isGoal(testGrid.getGoalState()) << std::endl;

    testGrid.visualize(std::cout, testGrid.getStartLocation(), metronome::Traffic::Action(5));

    std::vector<SuccessorBundle<metronome::Traffic>> test = testGrid.successors(testGrid.getStartLocation());

    std::cout << "SUCCESSORS: \n" << std::endl;
    for (auto bundle : test) {
        testGrid.visualize(std::cout, bundle.state, bundle.action);
    }

    // REQUIRE(traffic.getStartLocation() == metronome::Traffic::State(6, 0));
}
TEST_CASE("Traffic object movements test", "[Traffic]") {
    metronome::Traffic traffic = testGrid;
}
}
