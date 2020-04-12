#pragma once

#include <chrono>

#include "GlobalDefinitions.h"
#include "Logging.h"
namespace QDVO {
/**
 * Profilier based on RAII
 * Simply construct this profiler in the scope you want to profile.
 * The time will then be logged will then be logged with the corresponding
 * function name. The time is logged in microseconds
 */
class Profiler {
 public:
  /// Start the clock upon construction of this class.
  Profiler(std::string functionName) : functionName(functionName) {
    startTime = std::chrono::high_resolution_clock::now();
  }

  /// Upon desctruction log the time delta
  ~Profiler() {
    std::chrono::duration<double, std::milli> duration =
        std::chrono::high_resolution_clock::now() - startTime;
    LOG_INFO("{} : {:.6f} ms", functionName, duration.count());
  }

 private:
  std::string functionName;
  std::chrono::time_point<std::chrono::high_resolution_clock> startTime;
};
}  // namespace QDVO

#ifdef QDVO_ENABLE_PROFILING
#define PROFILE(sectionName) QDVO::Profiler __profiler_(sectionName);
#else
#define PROFILE(sectionName) (void)0
#endif
