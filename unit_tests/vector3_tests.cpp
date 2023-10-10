#include <random>
#include <gtest/gtest.h>

#include <arwain/vector3.hpp>

namespace Random
{
    /** \brief Generates a random double in the interval [-10, 10).
     * \return Double in [-10, 10).
     */
    static double Double()
    {
        static std::random_device rd;
        static std::mt19937 mt(rd());
        static std::uniform_real_distribution<double> dist(-10.0, 10.0);
        return dist(mt);
    }
}

TEST(Vector3, magnitude)
{
    for (int i = 0; i < 1000; i++)
    {
        double x = Random::Double();
        double y = Random::Double();
        double z = Random::Double();
        Vector3 vector{x, y, z};
        double expected_magnitude = std::sqrt(x*x + y*y + z*z);

        EXPECT_EQ(vector.magnitude(), expected_magnitude);
    }
}

TEST(Vector3, distance_between)
{
    for (int i = 0; i < 1000; i++)
    {
        double x1 = Random::Double();
        double y1 = Random::Double();
        double z1 = Random::Double();
        double x2 = Random::Double();
        double y2 = Random::Double();
        double z2 = Random::Double();

        Vector3 v1{x1, y1, z1};
        Vector3 v2{x2, y2, z2};

        double expected_distance = std::sqrt( (x2-x1)*(x2-x1) + (y2-y1)*(y2-y1) + (z2-z1)*(z2-z1) );

        EXPECT_EQ(Vector3::distance_between(v1, v2), expected_distance);
    }
}

TEST(Vector3, addition_operator)
{
    for (int i = 0; i < 1000; i++)
    {
        double x1 = Random::Double();
        double y1 = Random::Double();
        double z1 = Random::Double();
        double x2 = Random::Double();
        double y2 = Random::Double();
        double z2 = Random::Double();

        Vector3 v1{x1, y1, z1};
        Vector3 v2{x2, y2, z2};
        Vector3 v3{x1+x2, y1+y2, z1+z2};

        EXPECT_EQ(v1 + v2, v3);
    }
}

TEST(Vector3, subtraction_operator)
{
    for (int i = 0; i < 1000; i++)
    {
        double x1 = Random::Double();
        double y1 = Random::Double();
        double z1 = Random::Double();
        double x2 = Random::Double();
        double y2 = Random::Double();
        double z2 = Random::Double();

        Vector3 v1{x1, y1, z1};
        Vector3 v2{x2, y2, z2};
        Vector3 v3{x1-x2, y1-y2, z1-z2};

        EXPECT_EQ(v1 - v2, v3);
    }
}

TEST(Vector3, product_operator)
{
    for (int i = 0; i < 1000; i++)
    {
        double x1 = Random::Double();
        double y1 = Random::Double();
        double z1 = Random::Double();
        double x2 = Random::Double();
        double y2 = Random::Double();
        double z2 = Random::Double();

        Vector3 v1{x1, y1, z1};
        Vector3 v2{x2, y2, z2};
        Vector3 v3{x1*x2, y1*y2, z1*z2};

        EXPECT_EQ(v1 * v2, v3);
    }
}

TEST(Vector3, ostream)
{
    for (int i = 0; i < 1000; i++)
    {
        double x = Random::Double();
        double y = Random::Double();
        double z = Random::Double();

        Vector3 vector{x, y, z};

        EXPECT_NO_THROW(std::cout << vector << "\n");
    }
}

TEST(Vector3, from_array)
{
    std::array<double, 3> arr{1, 2, 3};
    Vector3 vec = Vector3::from_array(arr);
    EXPECT_EQ(vec.x, 1);
    EXPECT_EQ(vec.y, 2);
    EXPECT_EQ(vec.z, 3);
}

/** \brief Tests that the cross product of two Vector3s is correct, as compared against an independent calculation
 * on WolframAlpha.
 */
TEST(Vector3, cross)
{
    Vector3 v0{0.1857669481991132, 0.32978932882087486, 0.6884939739258459};
    Vector3 v1{0.780797532183143, 0.2626766978368277, 0.28715335316602153};
    Vector3 v0_v1 = Vector3::cross(v0, v1);
    Vector3 v0_v1_expected{-0.08615121194211027, 0.484230793681472, -0.208702045563506};
    EXPECT_NEAR(v0_v1.x, v0_v1_expected.x, 0.0000001);
    EXPECT_NEAR(v0_v1.y, v0_v1_expected.y, 0.0000001);
    EXPECT_NEAR(v0_v1.z, v0_v1_expected.z, 0.0000001);

    Vector3 v2{0.8041964747102701, 0.6292249757761414, 0.9353910130782362};
    Vector3 v3{0.47864511009380295, 0.891823245890142, 0.13513004312589838};
    Vector3 v2_v3 = Vector3::cross(v2, v3);
    Vector3 v2_v3_expected{-0.749176251347379, 0.3390492301262920, 0.416025652605381};
    EXPECT_NEAR(v2_v3.x, v2_v3_expected.x, 0.0000001);
    EXPECT_NEAR(v2_v3.y, v2_v3_expected.y, 0.0000001);
    EXPECT_NEAR(v2_v3.z, v2_v3_expected.z, 0.0000001);

    Vector3 v4{0.22470798356309896, 0.7116971485311132, 0.4219348611837962};
    Vector3 v5{0.08542676692363194, 0.8393346354334873, 0.1890299232765289};
    Vector3 v4_v5 = Vector3::cross(v4, v5);
    Vector3 v4_v5_expected{-0.2196124855054199, -0.006432001849252958, 0.1278072070351469};
    EXPECT_NEAR(v4_v5.x, v4_v5_expected.x, 0.0000001);
    EXPECT_NEAR(v4_v5.y, v4_v5_expected.y, 0.0000001);
    EXPECT_NEAR(v4_v5.z, v4_v5_expected.z, 0.0000001);
}

TEST(Vector3, normalized)
{
    for (int i = 0; i < 1000; i++)
    {
        double x = Random::Double();
        double y = Random::Double();
        double z = Random::Double();

        Vector3 vector{x, y, z};
        Vector3 vector_normalized = vector.normalized();

        double magnitude = std::sqrt(x*x+y*y+z*z);
        double x_norm = x / magnitude;
        double y_norm = y / magnitude;
        double z_norm = z / magnitude;

        EXPECT_NEAR(vector_normalized.x, x_norm, 0.0000001);
        EXPECT_NEAR(vector_normalized.y, y_norm, 0.0000001);
        EXPECT_NEAR(vector_normalized.z, z_norm, 0.0000001);
    }
}

TEST(Vector3, dot)
{
    for (int i = 0; i < 1000; i++)
    {
        double x1 = Random::Double();
        double y1 = Random::Double();
        double z1 = Random::Double();
        double x2 = Random::Double();
        double y2 = Random::Double();
        double z2 = Random::Double();

        Vector3 v1{x1, y1, z1};
        Vector3 v2{x2, y2, z2};
        double expected_dot_product = x1*x2 + y1*y2 + z1*z2;

        EXPECT_EQ(Vector3::dot(v1, v2), expected_dot_product);
    }
}

TEST(Vector3, angle_between)
{
    Vector3 v0{0.1857669481991132, 0.32978932882087486, 0.6884939739258459};
    Vector3 v1{0.780797532183143, 0.2626766978368277, 0.28715335316602153};
    double expected_angle_01 = 0.89383216130855;
    EXPECT_NEAR(Vector3::angle_between(v0, v1), expected_angle_01, 0.0000001);

    Vector3 v2{0.8041964747102701, 0.6292249757761414, 0.9353910130782362};
    Vector3 v3{0.47864511009380295, 0.891823245890142, 0.13513004312589838};
    double expected_angle_23 = 0.70986287423397;
    EXPECT_NEAR(Vector3::angle_between(v2, v3), expected_angle_23, 0.0000001);

    Vector3 v4{0.22470798356309896, 0.7116971485311132, 0.4219348611837962};
    Vector3 v5{0.08542676692363194, 0.8393346354334873, 0.1890299232765289};
    double expected_angle_45 = 0.350006093883595;
    EXPECT_NEAR(Vector3::angle_between(v4, v5), expected_angle_45, 0.0000001);
}

TEST(Vector3, equality_operator)
{
    for (int i = 0; i < 1000; i++)
    {
        double x1 = Random::Double();
        double y1 = Random::Double();
        double z1 = Random::Double();
        double x2 = Random::Double();
        double y2 = Random::Double();
        double z2 = Random::Double();

        Vector3 v1{x1, y1, z1};
        Vector3 v2{x2, y2, z2};

        EXPECT_TRUE(v1 == v1);
        EXPECT_TRUE(v2 == v2);
        EXPECT_FALSE(v1 == v2);
        EXPECT_FALSE(v2 == v1);

        Vector3 v3{x1, y1, z1};
        Vector3 v4{x1, y2, z1};

        EXPECT_FALSE(v3 == v4);
        EXPECT_FALSE(v4 == v3);

        Vector3 v5{x1, y1, z1};
        Vector3 v6{x1, y1, z2};

        EXPECT_FALSE(v5 == v6);
        EXPECT_FALSE(v6 == v5);
    }
}

TEST(Vector3, inequality_operator)
{
    for (int i = 0; i < 1000; i++)
    {
        double x1 = Random::Double();
        double y1 = Random::Double();
        double z1 = Random::Double();
        double x2 = Random::Double();
        double y2 = Random::Double();
        double z2 = Random::Double();

        Vector3 v1{x1, y1, z1};
        Vector3 v2{x2, y2, z2};

        EXPECT_TRUE(v1 != v2);
        EXPECT_TRUE(v2 != v1);
    }   
}

TEST(ImuData, index_operator)
{
    ImuData v{{1, 2, 3}, {4, 5, 6}};

    EXPECT_EQ(v[3], 1.0);
    EXPECT_EQ(v[4], 2.0);
    EXPECT_EQ(v[5], 3.0);
    EXPECT_EQ(v[0], 4.0);
    EXPECT_EQ(v[1], 5.0);
    EXPECT_EQ(v[2], 6.0);
    EXPECT_THROW(v[7], std::exception);
}

TEST(Vector3, scaler_division_operator)
{
    Vector3 v{Random::Double(), Random::Double(), Random::Double()};

    EXPECT_EQ(v.x/3, (v/3).x);
    EXPECT_EQ(v.x/3.1, (v/3.1).x);
    EXPECT_EQ(v.x/3.0f, (v/3.0f).x);
    EXPECT_EQ(v.x/3L, (v/3L).x);
    EXPECT_EQ(v.x/3U, (v/3U).x);
}

TEST(Vector3, scalar_product_operator)
{
    for (int i = 0; i < 1000; i++)
    {
        Vector3 v{Random::Double(), Random::Double(), Random::Double()};

        EXPECT_EQ(v.x*3, (v*3).x);
        EXPECT_EQ(v.x*3.1, (v*3.1).x);
        EXPECT_EQ(v.x*3.0f, (v*3.0f).x);
        EXPECT_EQ(v.x*3L, (v*3L).x);
        EXPECT_EQ(v.x*3U, (v*3U).x);

        EXPECT_EQ(v.x*3, (3*v).x);
        EXPECT_EQ(v.x*3.1, (3.1*v).x);
        EXPECT_EQ(v.x*3.0f, (3.0f*v).x);
        EXPECT_EQ(v.x*3L, (3L*v).x);
        EXPECT_EQ(v.x*3U, (3U*v).x);
    }
}
