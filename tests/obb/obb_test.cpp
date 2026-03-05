#include "physkit/collision.h"
#include "physkit/detail/bounds.h"
#include "test.h"

#include <numbers>

using namespace mp_units;
using namespace mp_units::si::unit_symbols;
using namespace physkit;
using namespace testing;

namespace
{
void test_default_constructor()
{
    obb box;
    CHECK_APPROX(box.center, vec3{0.0, 0.0, 0.0} * m);
    CHECK_APPROX(box.orientation.angular_distance(quat<one>::identity()), 0.0 * si::radian);
    CHECK_APPROX(box.half_extents, vec3{0.0, 0.0, 0.0} * m);
}

void test_basic_properties()
{
    obb box{
        vec3{1.0, 2.0, 3.0} * m,
        quat<one>::identity(),
        vec3{2.0, 3.0, 4.0} * m,
    };

    CHECK_APPROX(box.center_pos(), vec3{1.0, 2.0, 3.0} * m);
    CHECK_APPROX(box.extents(), vec3{2.0, 3.0, 4.0} * m);
    CHECK_APPROX(box.size(), vec3{4.0, 6.0, 8.0} * m);
    CHECK_APPROX(box.volume(), 192.0 * m * m * m);
    CHECK_APPROX(box.surface_area(), 208.0 * m * m);
}

void test_from_aabb_identity()
{
    aabb source{
        .min = vec3{-1.0, -2.0, -3.0} * m,
        .max = vec3{5.0, 4.0, 3.0} * m,
    };

    auto box = obb::from_aabb(source);
    CHECK_APPROX(box.center, source.center());
    CHECK_APPROX(box.half_extents, source.extent());
    CHECK_APPROX(box.orientation.angular_distance(quat<one>::identity()), 0.0 * si::radian);
}

void test_from_aabb_with_transform()
{
    aabb source{
        .min = vec3{0.0, 0.0, 0.0} * m,
        .max = vec3{2.0, 4.0, 6.0} * m,
    };
    auto rot =
        quat<one>::from_angle_axis((std::numbers::pi / 2.0) * si::radian, vec3{0.0, 0.0, 1.0});
    auto position = vec3{10.0, 20.0, 30.0} * m;

    auto box = obb::from_aabb(source, position, rot);
    CHECK_APPROX(box.center, vec3{8.0, 21.0, 33.0} * m);
    CHECK_APPROX(box.half_extents, vec3{1.0, 2.0, 3.0} * m);
    CHECK_APPROX(box.orientation.angular_distance(rot), 0.0 * si::radian);
}

void test_point_and_corners()
{
    obb box{
        vec3{10.0, 0.0, -2.0} * m,
        quat<one>::identity(),
        vec3{1.0, 2.0, 3.0} * m,
    };

    CHECK_APPROX(box.point(0), vec3{9.0, -2.0, -5.0} * m);
    CHECK_APPROX(box.point(7), vec3{11.0, 2.0, 1.0} * m);

    auto corners = box.corners();
    for (unsigned int i = 0; i < 8; ++i) { CHECK_APPROX(corners[i], box.point(i)); }
}

void test_axes_rotation()
{
    auto rot =
        quat<one>::from_angle_axis((std::numbers::pi / 2.0) * si::radian, vec3{0.0, 0.0, 1.0});
    obb box{
        vec3{0.0, 0.0, 0.0} * m,
        rot,
        vec3{1.0, 1.0, 1.0} * m,
    };

    auto axes = box.axes();
    CHECK_APPROX(axes[0], vec3<one>{0.0, 1.0, 0.0});
    CHECK_APPROX(axes[1], vec3<one>{-1.0, 0.0, 0.0});
    CHECK_APPROX(axes[2], vec3<one>{0.0, 0.0, 1.0});
}

void test_transform()
{
    obb source{
        vec3{1.0, 0.0, 0.0} * m,
        quat<one>::identity(),
        vec3{2.0, 3.0, 4.0} * m,
    };
    auto rot =
        quat<one>::from_angle_axis((std::numbers::pi / 2.0) * si::radian, vec3{0.0, 0.0, 1.0});
    auto translation = vec3{2.0, 3.0, 4.0} * m;

    auto transformed = obb::transform(source, rot, translation);
    CHECK_APPROX(transformed.center, vec3{2.0, 4.0, 4.0} * m);
    CHECK_APPROX(transformed.orientation.angular_distance(rot), 0.0 * si::radian);
    CHECK_APPROX(transformed.half_extents, source.half_extents);
}

void test_contains_and_contains_local()
{
    obb axis_aligned{
        vec3{0.0, 0.0, 0.0} * m,
        quat<one>::identity(),
        vec3{1.0, 2.0, 3.0} * m,
    };

    CHECK(axis_aligned.contains_local(vec3{1.0, 2.0, 3.0} * m));
    CHECK(!axis_aligned.contains_local(vec3{1.1, 0.0, 0.0} * m));
    CHECK(axis_aligned.contains(vec3{0.0, 0.0, 0.0} * m));
    CHECK(!axis_aligned.contains(vec3{1.1, 0.0, 0.0} * m));

    auto rot =
        quat<one>::from_angle_axis((std::numbers::pi / 2.0) * si::radian, vec3{0.0, 0.0, 1.0});
    obb rotated{
        vec3{5.0, 0.0, 0.0} * m,
        rot,
        vec3{1.0, 2.0, 1.0} * m,
    };

    auto inside_local = vec3{0.5, 1.5, 0.0} * m;
    auto outside_local = vec3{1.1, 0.0, 0.0} * m;
    CHECK(rotated.contains(rotated.center + rotated.orientation * inside_local));
    CHECK(!rotated.contains(rotated.center + rotated.orientation * outside_local));
}

void test_support_point()
{
    obb axis_aligned{
        vec3{0.0, 0.0, 0.0} * m,
        quat<one>::identity(),
        vec3{1.0, 2.0, 3.0} * m,
    };
    CHECK_APPROX(axis_aligned.support(vec3<one>{1.0, -2.0, 0.3}), vec3{1.0, -2.0, 3.0} * m);

    auto rot =
        quat<one>::from_angle_axis((std::numbers::pi / 2.0) * si::radian, vec3{0.0, 0.0, 1.0});
    obb rotated{
        vec3{1.0, 1.0, 0.0} * m,
        rot,
        vec3{2.0, 1.0, 1.0} * m,
    };
    CHECK_APPROX(rotated.support(vec3<one>{1.0, 0.0, 0.0}), vec3{2.0, 3.0, 1.0} * m);
}

void test_compound_assign()
{
    obb box{
        vec3{1.0, 2.0, 3.0} * m,
        quat<one>::identity(),
        vec3{2.0, 3.0, 4.0} * m,
    };

    box += vec3{1.0, -2.0, 3.0} * m;
    CHECK_APPROX(box.center, vec3{2.0, 0.0, 6.0} * m);

    box -= vec3{2.0, 1.0, 1.0} * m;
    CHECK_APPROX(box.center, vec3{0.0, -1.0, 5.0} * m);

    box *= 0.5;
    CHECK_APPROX(box.half_extents, vec3{1.0, 1.5, 2.0} * m);

    box *= 2.0 * one;
    CHECK_APPROX(box.half_extents, vec3{2.0, 3.0, 4.0} * m);
}
} // namespace

int main()
{
    suite s;
    s.group("OBB")
        .test("default constructor", test_default_constructor)
        .test("basic properties", test_basic_properties)
        .test("from_aabb identity", test_from_aabb_identity)
        .test("from_aabb transform", test_from_aabb_with_transform)
        .test("point and corners", test_point_and_corners)
        .test("axes rotation", test_axes_rotation)
        .test("transform", test_transform)
        .test("contains and contains_local", test_contains_and_contains_local)
        .test("support_point", test_support_point)
        .test("compound assignment", test_compound_assign);
    return s.run();
}
