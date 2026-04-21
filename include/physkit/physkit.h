#pragma once
#include "algebra/eigen_format.h" // IWYU pragma: keep
#include "core/accum.h"           // IWYU pragma: keep
#include "core/awaitables.h"      // IWYU pragma: keep
#include "core/world.h"           // IWYU pragma: keep
#include "detail/integrate.h"     // IWYU pragma: keep

PHYSKIT_EXPORT
namespace physkit
{
constexpr std::string_view version_string() noexcept { return PHYSKIT_VERSION; }
} // namespace physkit