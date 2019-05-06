#pragma once

#define SCALAR_TYPE double
#define SMALL_NUMBER 1e-12
#define IMUSTATE_DIMENSIONS 9
#define ID_TYPE uint64_t

// This allows the settings to be globally accessible.
#include <Settings.h>
extern QDVO::Settings settings;
