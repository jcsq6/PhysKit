#pragma once

#ifdef PHYSKIT_IN_MODULE_IMPL
#ifdef PHYSKIT_IMPORT_STD
import std;
#endif
#else
#include "macros.h"
#include <version>
#if __cpp_lib_generator >= 202207L
#include <generator>
#endif
#endif

#include "_generator_impl.h"

namespace physkit
{
#if __cpp_lib_generator < 202207L
using generator_impl::generator;
#else
using std::generator;
#endif
} // namespace physkit