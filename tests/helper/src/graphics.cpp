module;

#define GRAPHICS_IN_MODULE_IMPL
#include "graphics/detail/gmf.h"

#ifndef PHYSKIT_MODULES
#include <physkit/physkit.h>
#endif

export module graphics;

export import graphics.camera;
export import graphics.convert;
export import graphics.magnum;

#ifdef PHYSKIT_MODULES
import physkit;
import mp_units;
#endif

#ifdef PHYSKIT_IMPORT_STD
import std;
#endif

#include "graphics/graphics.h"
