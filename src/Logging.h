/*
 * This file is used to define the log level globally.
 * It can also be used to define new helper macros.
 */

#ifndef SPDLOG_ACTIVE_LEVEL
#define SPDLOG_ACTIVE_LEVEL SPDLOG_LEVEL_TRACE
#endif

#include "spdlog/spdlog.h"
#include "spdlog/fmt/ostr.h" // must be included

