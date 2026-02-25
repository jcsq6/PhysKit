#include "physkit/collision.h"

namespace physkit
{
std::optional<collision_info> gjk_epa(const mesh::instance &a, const mesh::instance &b)
{
    assert(false && "GJK-EPA not implemented");
}

std::optional<collision_info> sat(const mesh::instance &a, const mesh::instance &b)
{
    assert(false && "SAT not implemented");
}
} // namespace physkit