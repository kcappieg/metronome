#pragma once

#include <easylogging++.h>
#include <rapidjson/document.h>
#include <string>
#include <algorithms/NaiveOptimalActiveGoalRecognitionDesign.hpp>
#include "Configuration.hpp"
#include "MetronomeException.hpp"
#include "OfflineExperiment.hpp"
#include "RealTimeExperiment.hpp"
#include "Result.hpp"
#include "domains/Traffic.hpp"
#include "experiment/termination/ExpansionTerminationChecker.hpp"
#include "experiment/termination/TimeTerminationChecker.hpp"
#include "utils/File.hpp"

#ifdef ENABLE_GRID_WORLD
#include "domains/GridWorld.hpp"
#include "domains/GridMap.hpp"
#endif
#ifdef ENABLE_LOGISTICS
#include "domains/Logistics.hpp"
#endif
#ifdef ENABLE_DYNAMIC_GRID_WORLD
#include "domains/DynamicGridWorld.hpp"
#endif
#ifdef ENABLE_ORIENTATION_GRID
#include "domains/OrientationGrid.hpp"
#endif
#ifdef ENABLE_VACUUM_WORLD
#include "domains/VacuumWorld.hpp"
#endif
#ifdef ENABLE_SLIDING_TILE_PUZZLE
#include "domains/SlidingTilePuzzle.hpp"
#endif
#ifdef ENABLE_TRAFFIC_WORLD
#include "domains/Traffic.hpp"
#endif

#ifdef ENABLE_A_STAR
#include "algorithms/AStar.hpp"
#include "GrdExperiment.hpp"

#endif
#ifdef ENABLE_LSS_LRTA_STAR
#include "algorithms/LssLrtaStar.hpp"
#endif
#ifdef ENABLE_CLUSTER_RTS
#include "algorithms/ClusterRts.hpp"
#endif
#ifdef ENABLE_TIME_BOUNDED_A_STAR_BENCE
#include "algorithms/TimeBoundedAStar.hpp"
#endif
#ifdef ENABLE_TIME_BOUNDED_A_STAR_KEVIN
#include "algorithms/TBAStar.hpp"
#endif
#ifdef ENABLE_TRIVIAL_GRD
#include "algorithms/TrivialGrdPlanner.hpp"
#endif
#ifdef ENABLE_NAIVE_DYNAMIC
#include "algorithms/NaiveDynamicAStar.hpp"
#endif

namespace metronome {

/* We need template specializations for getDomain function, so that must be pulled
 * out of the class definition and into a free-floating function
 */

template<typename Domain>
static Domain getDomain(const Configuration& configuration,
                        const std::string& resourcesDir) {
  if (configuration.hasMember(RAW_DOMAIN)) {
    std::string rawDomain{configuration.getString(RAW_DOMAIN)};
    std::stringstream stringStream{rawDomain};

    return Domain{configuration, stringStream};
  } else if (configuration.hasMember(DOMAIN_PATH)) {
    std::string domainPath{resourcesDir +
                           configuration.getString(DOMAIN_PATH)};

    // Check if file exists
    if (!fileExists(domainPath)) {
      throw MetronomeException{"Invalid domain file path: " + domainPath};
    }

    std::fstream fileInputStream;
    fileInputStream.open(domainPath, std::fstream::in);
    Domain domain{configuration, fileInputStream};
    fileInputStream.close();

    return domain;
  } else {
    throw MetronomeException("Domain not found.");
  }
}

/**
 * For domains that also take a Scene input.
 * Note: does not permit Raw input; must be files
 * @tparam Domain
 * @param configuration
 * @param resourcesDir
 * @return
 */
template<typename Domain>
static Domain getDomainWithScene(const Configuration& configuration,
                                 const std::string& resourcesDir) {
  if (!configuration.hasMember(DOMAIN_PATH) || !configuration.hasMember(SCENE_PATH)) {
    throw MetronomeException("Domain and scene file paths required for grid map");
  }

  std::string domainPath{resourcesDir +
                         configuration.getString(DOMAIN_PATH)};
  std::string scenePath{resourcesDir +
                        configuration.getString(SCENE_PATH)};

  // Check if files exist
  if (!fileExists(domainPath)) {
    throw MetronomeException{"Invalid domain file path: " + domainPath};
  }
  if (!fileExists(scenePath)) {
    throw MetronomeException{"Invalid scene file path: " + scenePath};
  }

  std::fstream mapInputStream;
  std::fstream sceneInputStream;

  mapInputStream.open(domainPath, std::fstream::in);
  sceneInputStream.open(scenePath, std::fstream::in);

  Domain domain{configuration, mapInputStream, sceneInputStream};

  mapInputStream.close();
  sceneInputStream.close();

  return domain;
}


class ConfigurationExecutor {
 public:
  static Result executeConfiguration(const Configuration& configuration,
                                     const std::string& resourcesDir) {
    try {
      return unsafeExecuteConfiguration(configuration, resourcesDir);
    } catch (const MetronomeException& exception) {
      return Result(configuration, exception.what());
    } catch (const std::exception& exception) {
      return Result(configuration, exception.what());
    } catch (...) {
      return Result(configuration, "Unexpected exception.");
    }
  }

  template <typename Domain>
  static Domain extractDomain(const Configuration& configuration,
                              const std::string& resourcesDir) {
    return getDomain<Domain>(configuration, resourcesDir);
  }

 private:
  static Result unsafeExecuteConfiguration(const Configuration& configuration,
                                           const std::string& resourcesDir) {
    LOG(INFO) << "Configuration started.";

    if (!configuration.hasMember(DOMAIN_NAME)) {
      LOG(ERROR) << "Domain name not found." << std::endl;
      return Result(configuration, "Missing: domainName");
    }

    std::string domainName{configuration.getString(DOMAIN_NAME)};

#ifdef ENABLE_GRID_WORLD
    if (domainName == DOMAIN_GRID_WORLD) {
      return executeDomain<GridWorld>(configuration, resourcesDir);
    }

    if (domainName == DOMAIN_GRID_MAP) {
      return executeDomain<GridMap>(configuration, resourcesDir);
    }
#endif

#ifdef ENABLE_LOGISTICS
    if (domainName == DOMAIN_LOGISTICS) {
      return executeDomain<Logistics>(configuration, resourcesDir);
    }
#endif

#ifdef ENABLE_DYNAMIC_GRID_WORLD
    if (domainName == DOMAIN_DYNAMIC_GRID_WORLD) {
      return executeDomain<DynamicGridWorld>(configuration, resourcesDir);
    }
#endif

#ifdef ENABLE_ORIENTATION_GRID
    if (domainName == DOMAIN_ORIENTATION_GRID) {
      return executeDomain<OrientationGrid>(configuration, resourcesDir);
    }
#endif

#ifdef ENABLE_VACUUM_WORLD
      if (domainName == DOMAIN_VACUUM_WORLD) {
      return executeDomain<VacuumWorld>(configuration, resourcesDir);
    }
#endif

#ifdef ENABLE_SLIDING_TILE_PUZZLE
    if (domainName == DOMAIN_TILES) {
      return executeDomain<SlidingTilePuzzle<4>>(configuration, 
          resourcesDir);
    }
#endif

#ifdef ENABLE_TRAFFIC_WORLD
    if (domainName == DOMAIN_TRAFFIC) {
      return executeDomain<Traffic>(configuration, resourcesDir);
    }
#endif

    LOG(ERROR) << "Unknown domain name: " << domainName << std::endl;
    return Result(configuration, "Unknown domainName: " + domainName);
  }

  template <typename Domain>
  static Result executeDomain(const Configuration& configuration,
                              const std::string& resourcesDir) {
    if (!configuration.hasMember(DOMAIN_NAME)) {
      LOG(ERROR) << "Termination checker not found." << std::endl;
      return Result(configuration, "Missing: terminationCheckerType");
    }
    const std::string terminationCheckerType{
        configuration.getString(TERMINATION_CHECKER_TYPE)};

    if (terminationCheckerType == TERMINATION_CHECKER_EXPANSION) {
      return executeDomain<Domain, ExpansionTerminationChecker>(configuration,
                                                                resourcesDir);
    } else if (terminationCheckerType == TERMINATION_CHECKER_TIME) {
      return executeDomain<Domain, TimeTerminationChecker>(configuration,
                                                           resourcesDir);
    } else {
      LOG(ERROR) << "Unknown termination checker type: "
                 << terminationCheckerType << std::endl;
      return Result(configuration, "Unknown: termination checker type");
    }
  }

  template <typename Domain, typename TerminationChecker>
  static Result executeDomain(const Configuration& configuration,
                              const std::string& resourcesDir) {
    if (!(configuration.hasMember(RAW_DOMAIN) ||
          configuration.hasMember(DOMAIN_PATH))) {
      LOG(ERROR)
          << "Domain not found. Raw domain or domain path must be provided."
          << std::endl;
      return Result(configuration, "Missing: rawDomain AND domainPath");
    }

    Domain domain{getDomain<Domain>(configuration, resourcesDir)};

    if (!configuration.hasMember(ALGORITHM_NAME)) {
      LOG(ERROR) << "Algorithm name not found." << std::endl;
      return Result(configuration, "Missing: algorithmName");
    }

    std::string algorithmName{configuration.getString(ALGORITHM_NAME)};

#ifdef ENABLE_A_STAR
    if (algorithmName == ALGORITHM_A_STAR) {
      return executeOfflinePlanner<Domain, AStar<Domain>>(configuration,
                                                          domain);
    }
#endif

#ifdef ENABLE_LSS_LRTA_STAR
    if (algorithmName == ALGORITHM_LSS_LRTA_STAR) {
      return executeRealTimePlanner<Domain,
                                    LssLrtaStar<Domain, TerminationChecker>,
                                    TerminationChecker>(configuration, domain);
    }
#endif

#ifdef ENABLE_CLUSTER_RTS
    if (algorithmName == ALGORITHM_CLUSTER_RTS) {
      return executeRealTimePlanner<Domain,
                                    ClusterRts<Domain, TerminationChecker>,
                                    TerminationChecker>(configuration, domain);
    }
#endif

#ifdef ENABLE_TIME_BOUNDED_A_STAR_BENCE
    if (algorithmName == ALGORITHM_TIME_BOUNDED_A_STAR && configuration.getString(VARIANT, "kevin") == "bence") {
      LOG(INFO) << "TBA* variant: Bence implementation";
      return executeRealTimePlanner<
          Domain,
          TimeBoundedAStar<Domain, TerminationChecker>,
          TerminationChecker>(configuration, domain);
    }
#endif

#ifdef ENABLE_TIME_BOUNDED_A_STAR_KEVIN
    if (algorithmName == ALGORITHM_TIME_BOUNDED_A_STAR && configuration.getString(VARIANT, "kevin") == "kevin") {
      LOG(INFO) << "TBA* variant: Kevin implementation";
      return executeRealTimePlanner<Domain,
                                    TBAStar<Domain, TerminationChecker>,
                                    TerminationChecker>(configuration, domain);
    }
#endif

#ifdef ENABLE_TRIVIAL_GRD
    if (algorithmName == GRD_ALGORITHM_TRIVIAL) {
      LOG(INFO) << "Trivial GRD";

      return executeGoalRecognitionDesignPlanner<Domain, TrivialGrdPlanner<Domain>>(
                      configuration, domain);
    }
#endif

#ifdef ENABLE_NAIVE_OPTIMAL_AGRD
    if (algorithmName == GRD_ALGORITHM_NAIVE_OPTIMAL) {
      LOG(INFO) << "Naive Optimal GRD";

      return executeGoalRecognitionDesignPlanner<Domain, NaiveOptimalActiveGoalRecognitionDesign<Domain>>(
              configuration, domain);
    }
#endif

    LOG(ERROR) << "Unknown algorithms name: " << algorithmName << std::endl;
    return Result(configuration, "Unknown: algorithmName: " + algorithmName);
  }

  template <typename Domain, typename Planner>
  static Result executeOfflinePlanner(const Configuration& configuration,
                                      const Domain& domain) {
    Planner planner{domain, configuration};

    OfflineExperiment<Domain, Planner> offlinePlanManager;

    LOG(INFO) << "Configuration done.";
    return offlinePlanManager.plan(configuration, domain, planner);
  }

  template <typename Domain, typename Planner, typename TerminationChecker>
  static Result executeRealTimePlanner(const Configuration& configuration,
                                       const Domain& domain) {
    Planner planner{domain, configuration};

    RealTimeExperiment<Domain, Planner, TerminationChecker> realTimePlanManager(
        configuration);

    LOG(INFO) << "Configuration done.";
    return realTimePlanManager.plan(configuration, domain, planner);
  }

  template<typename Domain, typename GrdPlanner>
  static Result executeGoalRecognitionDesignPlanner(const Configuration& configuration,
                                                    Domain& domain) {
    GrdPlanner planner(domain, configuration);

    auto subjectAlgorithm = configuration.getString(SUBJECT_ALGORITHM);

    LOG(INFO) << "Finding subject algorithm from configuration" << std::endl;

#ifdef ENABLE_NAIVE_DYNAMIC
    if (subjectAlgorithm == ALGORITHM_NAIVE_DYNAMIC) {
      GrdExperiment<Domain, GrdPlanner, NaiveDynamicAStar<Domain>> grdPlanManager(configuration);
      return grdPlanManager.plan(configuration, domain, planner);
    }
#endif

    LOG(ERROR) << "Unknown Subject algorithm: " << subjectAlgorithm << std::endl;
    return Result(configuration, "Unknown Subject algorithm: " + subjectAlgorithm);
  }
};

// TODO Template specialization for KeyGrid domain -> needs 2 fstreams

#ifdef ENABLE_GRID_WORLD
template<>
GridMap getDomain<GridMap>(const Configuration& configuration,
                                             const std::string& resourcesDir) {
  return getDomainWithScene<GridMap>(configuration, resourcesDir);
}
#endif

}  // namespace metronome
