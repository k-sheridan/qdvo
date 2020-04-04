#pragma once

#ifdef QDVO_ENABLE_PROFILING
#include <chrono>

#include "Logging.h"
/**
 * Profilier based on RAII
 * Simply construct this profiler in the scope you want to profile.
 * The time will then be logged will then be logged with the corresponding
 * function name. The time is logged in microseconds
 */
class Profiler {
  /// Start the clock upon construction of this class.
  Profiler(std::string functionName) : functionName(functionName) {
    startTime = std::chrono::high_resolution_clock::now();
  }

  /// Upon desctruction log the time delta
  ~Profiler() {
    auto duration = std::chrono::high_resolution_clock::now() - startTime;
    LOG_INFO("{} : {}", functionName,
             std::chrono::duration_cast<std::chrono::microseconds>(duration));
  }

  std::string functionName;
  std::chrono::time_point<std::chrono::high_resolution_clock> startTime;
};

#define PROFILE(sectionName) Profiler(sectionName);
#else
#define PROFILE(sectionName) (void)0
#endif
