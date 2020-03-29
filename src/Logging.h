/*
 * This file is used to define the log level globally.
 * It can also be used to define new helper macros.
 */

#define SPDLOG_HEADER_ONLY
#define SPDLOG_ACTIVE_LEVEL SPDLOG_LEVEL_TRACE

#include <stdlib.h>

#include "spdlog/fmt/ostr.h"  // must be included
#include "spdlog/spdlog.h"

/// Define trace macro to force a log level set.
#if SPDLOG_ACTIVE_LEVEL <= SPDLOG_LEVEL_TRACE
#define LOG_TRACE(...)                       \
  {                                          \
    spdlog::set_level(spdlog::level::trace); \
    SPDLOG_TRACE(__VA_ARGS__);               \
  }
#else
#define LOG_TRACE(...) (void)0
#endif

// Info
#if SPDLOG_ACTIVE_LEVEL <= SPDLOG_LEVEL_INFO
#define LOG_INFO(...) \
  { SPDLOG_INFO(__VA_ARGS__); }
#else
#define LOG_INFO(...) (void)0
#endif

// Error
#if SPDLOG_ACTIVE_LEVEL <= SPDLOG_LEVEL_ERROR
#define LOG_ERROR(...) \
  { SPDLOG_ERROR(__VA_ARGS__); }
#else
#define LOG_ERROR(...) (void)0
#endif

// Warn
#if SPDLOG_ACTIVE_LEVEL <= SPDLOG_LEVEL_WARN
#define LOG_WARN(...) \
  { SPDLOG_WARN(__VA_ARGS__); }
#else
#define LOG_WARN(...) (void)0
#endif

// Debug
#if SPDLOG_ACTIVE_LEVEL <= SPDLOG_LEVEL_DEBUG
#define LOG_DEBUG(...)                       \
  {                                          \
    spdlog::set_level(spdlog::level::debug); \
    SPDLOG_DEBUG(__VA_ARGS__);               \
  }
#else
#define LOG_DEBUG(...) (void)0
#endif

#define CHECK(condition, ...) \
  {                           \
    if (!(condition)) {         \
      LOG_ERROR(__VA_ARGS__); \
      exit(EXIT_FAILURE);     \
    }                         \
  }

