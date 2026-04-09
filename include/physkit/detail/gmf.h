#pragma once

#define PHYSKIT_IN_GMF

#include "macros.h"

#include <absl/container/flat_hash_map.h>
#include <absl/container/flat_hash_set.h>
#include <absl/container/inlined_vector.h>

#include <Eigen/Dense>
#include <Eigen/Geometry>

#ifndef PHYSKIT_IMPORT_STD
#include <algorithm>
#include <array>
#include <cassert>
#include <cmath>
#include <concepts>
#include <cstddef>
#include <cstdint>
#include <cstdio>
#include <expected>
#include <format>
#include <fstream>
#include <functional>
#include <iostream>
#include <limits>
#include <memory>
#include <numbers>
#include <optional>
#include <print>
#include <ranges>
#include <span>
#include <stack>
#include <stdexcept>
#include <string>
#include <utility>
#include <variant>
#include <vector>
#endif

// TODO: remove
#if defined(_WIN32)
#include <windows.h>
#elif defined(__APPLE__)
#include <sys/sysctl.h>
#include <sys/types.h>
#include <unistd.h>
#endif

#undef PHYSKIT_IN_GMF