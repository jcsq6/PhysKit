module;
#include "physkit/detail/gmf.h"
module physkit:mesh;

import :decl;
import mp_units;

#ifdef PHYSKIT_IMPORT_STD
import std;
#endif

#define PHYSKIT_IN_MODULE_IMPL
#include "../mesh.cpp" // NOLINT(bugprone-suspicious-include)