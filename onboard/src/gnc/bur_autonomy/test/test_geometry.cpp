// Tests for include/bur_autonomy/geometry.hpp, the nav2-free replacement for
// nav2_util::geometry_utils::euclidean_distance (D-7).

#include <gtest/gtest.h>

#include "bur_autonomy/geometry.hpp"

namespace
{

struct P
{
    double x;
    double y;
    double z;
};

struct PF   // float members, like some message types
{
    float x;
    float y;
    float z;
};

}  // namespace

TEST(Geometry, HorizontalIgnoresDepth)
{
    // Same as nav2's euclidean_distance(a, b) with its default is_3d = false.
    EXPECT_DOUBLE_EQ(5.0, bur_autonomy::horizontalDistance(P{0, 0, 0}, P{3, 4, -10}));
}

TEST(Geometry, ThreeDIncludesDepth)
{
    EXPECT_DOUBLE_EQ(3.0, bur_autonomy::distance3d(P{1, 1, 1}, P{2, 3, -1}));
}

TEST(Geometry, SymmetricAndZero)
{
    const P a{1.5, -2.0, -0.5};
    const P b{-4.0, 7.0, -3.0};
    EXPECT_DOUBLE_EQ(bur_autonomy::horizontalDistance(a, b),
                     bur_autonomy::horizontalDistance(b, a));
    EXPECT_DOUBLE_EQ(bur_autonomy::distance3d(a, b), bur_autonomy::distance3d(b, a));
    EXPECT_DOUBLE_EQ(0.0, bur_autonomy::distance3d(a, a));
}

TEST(Geometry, MixedMemberTypes)
{
    EXPECT_NEAR(5.0, bur_autonomy::horizontalDistance(PF{0, 0, 0}, P{3, 4, 0}), 1e-6);
}
