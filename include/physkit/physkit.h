#pragma once
#include "core/accum.h"               // IWYU pragma: keep
#include "algebra/eigen_format.h" // IWYU pragma: keep
#include "core/integrator.h"          // IWYU pragma: keep
#include "core/world.h"               // IWYU pragma: keep

namespace physkit
{
constexpr std::string_view version_string() noexcept { return PHYSKIT_VERSION; }
} // namespace physkit