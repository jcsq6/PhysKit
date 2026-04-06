module;

#define PHYSKIT_IN_MODULE_IMPL
#include "physkit/detail/gmf.h"

export module physkit:decl;

import mp_units;

#ifdef PHYSKIT_IMPORT_STD
import std;
#endif

#include "physkit/collision/collision.h"
#include "physkit/physkit.h"
