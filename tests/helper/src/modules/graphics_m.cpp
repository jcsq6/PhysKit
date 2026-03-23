module;

#define GRAPHICS_IN_MODULE_IMPL
#include "graphics/detail/gmf.h"

#ifndef PHYSKIT_MODULES
#include <physkit/physkit.h>
#endif

export module graphics;

export import :camera;
export import :convert;
export import :magnum;

#ifdef PHYSKIT_MODULES
import physkit;
import mp_units;
#endif

#ifdef PHYSKIT_IMPORT_STD
import std;
#endif
#define GRAPHICS_IN_MODULE_IMPL

// clang-format off
#include "graphics/graphics.h"
#include "../graphics.cpp" // NOLINT(bugprone-suspicious-include)
// clang-format on
