#pragma once
#include "accum.h"               // IWYU pragma: keep
#include "detail/eigen_format.h" // IWYU pragma: keep
#include "integrator.h"          // IWYU pragma: keep
#include "world.h"               // IWYU pragma: keep

namespace physkit
{
constexpr std::string_view version_string() noexcept { return PHYSKIT_VERSION; }
} // namespace physkit