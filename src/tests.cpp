#include <map>
#include <random>
#include <iostream>
#include <functional>

#include "quaternion.hpp"
#include "timers.hpp"
#include "input_parser.hpp"

namespace Random
{
    double Double()
    {
        static std::random_device rd;
        static std::mt19937 mt(rd());
        static std::uniform_real_distribution<double> dist(-10.0, 10.0);
        return dist(mt);
    }
    double DoubleBetween(const double lower_bound, const double upper_bound)
    {
        static std::random_device rd;
        static std::mt19937 mt(rd());
        std::uniform_real_distribution<double> dist(lower_bound, upper_bound);
        return dist(mt);
    }
}

int fail_test()
{
    return 1;
}

int pass_test()
{
    return 0;
}

/** \brief Testing that the unit test framework is configured correctly. */
int Test_Test()
{
    return pass_test();
}

/* QUATERNION_TESTS */

/** \brief Test that the 4-element quaternion constructor produces valid output. */
int Test_QuaternionConstructors()
{
    bool passing = true;
    
    for (int i = 0; i < 1000; i++)
    {
        double w = Random::Double();
        double x = Random::Double();
        double y = Random::Double();
        double z = Random::Double();

        Quaternion q{w, x, y, z};

        passing &= (q.w == w);
        passing &= (q.x == x);
        passing &= (q.y == y);
        passing &= (q.z == z);
    }

    return passing ? pass_test() : fail_test();
}

bool is_close(double num1, double num2, double tolerance = 0.0000001)
{
    return std::abs(num1 - num2) < tolerance;
}

int Test_QuaternionInverse()
{
    bool passing = true;

    // Quaternion inverse is the same as conjugate for unit quaternion.
    for (int i = 0; i < 1000; i++)
    {
        Quaternion q1{Random::Double(), Random::Double(), Random::Double(), Random::Double()};
        q1 = q1.unit();
        Quaternion q1i = q1.inverse();
        Quaternion q1c = q1.conjugate();
        passing &= is_close(q1i.w, q1c.w);
        passing &= is_close(q1i.x, q1c.x);
        passing &= is_close(q1i.y, q1c.y);
        passing &= is_close(q1i.z, q1c.z);
    }

    return passing ? pass_test() : fail_test();
}

int Test_QuaternionSum()
{
    bool passing = true;

    // Quaternions add element-wise.
    for (int i = 0; i < 1000; i++)
    {
        Quaternion q1{Random::Double(), Random::Double(), Random::Double(), Random::Double()};
        Quaternion q2{Random::Double(), Random::Double(), Random::Double(), Random::Double()};
        Quaternion q3 = q1 + q2;
        passing &= (q3.w == q1.w + q2.w);
        passing &= (q3.x == q1.x + q2.x);
        passing &= (q3.y == q1.y + q2.y);
        passing &= (q3.z == q1.z + q2.z);
    }
    
    return passing ? pass_test() : fail_test();
}

int Test_QuaternionAxisAngleConstructor()
{
    bool passing = true;
    /*
    The axis-angle constructor should produce the following quaternion:
        w = cos(angle/2.0);
        x = sin(angle/2.0) * axis[0] * axisInvNorm;
        y = sin(angle/2.0) * axis[1] * axisInvNorm;
        z = sin(angle/2.0) * axis[2] * axisInvNorm;
    In addition, it will store the axis and angle originally passed.
    */

    double original_angle = Random::Double();
    std::array<double, 3> original_axis{Random::Double(), Random::Double(), Random::Double()};
    Quaternion q1{original_angle, original_axis};
    passing &= (q1.getAxis() == original_axis);
    passing &= (q1.getAngle() == original_angle);
    passing &= (q1.w == std::cos(original_angle/2.0));
    
    double original_axis_norm = std::sqrt(
        original_axis[0] * original_axis[0]
        + original_axis[1] * original_axis[1]
        + original_axis[2] * original_axis[2]
    );

    double tolerance = 0.00001;
    double x = (std::sin(original_angle/2.0) * original_axis[0] * 1.0 / original_axis_norm);
    double y = (std::sin(original_angle/2.0) * original_axis[1] * 1.0 / original_axis_norm);
    double z = (std::sin(original_angle/2.0) * original_axis[2] * 1.0 / original_axis_norm);

    passing &= std::abs(x - q1.x) < tolerance;
    passing &= std::abs(y - q1.y) < tolerance;
    passing &= std::abs(z - q1.z) < tolerance;

    return passing ? pass_test() : fail_test();
}

int Test_QuaternionDefaultConstructor()
{
    bool passing = true;

    for (int i = 0; i < 1000; i++)
    {
        try
        {
            Quaternion q1;
        }
        catch (const std::exception& e)
        {
            passing = false;
        }
    }

    return passing ? pass_test() : fail_test();
}

int Test_QuaternionVectorConstructor()
{
    bool passing = true;

    for (int i = 0; i < 1000; i++)
    {
        std::array<double, 3> vector{Random::Double(), Random::Double(), Random::Double()};
        Quaternion q{vector};
        passing &= (q.w == 0);
        passing &= (q.x == vector[0]);
        passing &= (q.y == vector[1]);
        passing &= (q.z == vector[2]);
    }

    return passing ? pass_test() : fail_test();
}

int Test_QuaternionDotProduct()
{
    bool passing = true;

    for (int i = 0; i < 1000; i++)
    {
        Quaternion q1{Random::Double(), Random::Double(), Random::Double(), Random::Double()};
        Quaternion q2{Random::Double(), Random::Double(), Random::Double(), Random::Double()};

        double dot_product = Quaternion::dot(q1, q2);

        passing &= (dot_product == q1.w * q2.w + q1.x * q2.x + q1.y * q2.y + q1.z * q2.z);
    }

    return passing ? pass_test() : fail_test();
}

/** \brief Test computation of the angle between two quaternions. Note that this*/
int Test_QuaternionAngleBetween()
{
    bool passing = true;

    for (int i = 0; i < 1000; i++)
    {
        Quaternion q1{Random::Double(), Random::Double(), Random::Double(), Random::Double()};
        Quaternion q2{Random::Double(), Random::Double(), Random::Double(), Random::Double()};
        q1 = q1.unit();
        q2 = q2.unit();

        double angle = Quaternion::angle_between(q1, q2);
        double expected_angle = 2 * std::acos(q1.w*q2.w + q1.x*q2.x + q1.y*q2.y + q1.z*q2.z);

        passing &= (angle == expected_angle);
    }

    return passing ? pass_test() : fail_test();
}

int Test_QuaternionSlerp()
{
    bool passing = true;

    for (int i = 0; i < 1000; i++)
    {
        
    }

    return passing ? pass_test() : fail_test();
}

int Test_QuaternionSubtractionOperator()
{
    bool passing = true;
    
    for (int i = 0; i < 1000; i++)
    {
        Quaternion q1{Random::Double(), Random::Double(), Random::Double(), Random::Double()};
        Quaternion q2{Random::Double(), Random::Double(), Random::Double(), Random::Double()};

        Quaternion diff = q1 - q2;

        passing &= (diff.w == q1.w - q2.w);
        passing &= (diff.x == q1.x - q2.x);
        passing &= (diff.y == q1.y - q2.y);
        passing &= (diff.z == q1.z - q2.z);
    }

    return passing ? pass_test() : fail_test();
}

int Test_Timer()
{
    bool passing = true;
    {
        arwain::Timers::ScopedTimer{"adadsf"};
    }
    return fail_test();
}

int Test_InputParser()
{
    bool passing = true;



    return passing ? pass_test() : fail_test();
}




int main(int argc, char* argv[])
{
    std::map<std::string, std::function<int()>> funcs = {
        {"Test_Test", Test_Test},
        {"Test_QuaternionConstructors", Test_QuaternionConstructors},
        {"Test_QuaternionInverse", Test_QuaternionInverse},
        {"Test_QuaternionSum", Test_QuaternionSum},
        {"Test_QuaternionAxisAngleConstructor", Test_QuaternionAxisAngleConstructor},
        {"Test_QuaternionDefaultConstructor", Test_QuaternionDefaultConstructor},
        {"Test_QuaternionVectorConstructor", Test_QuaternionVectorConstructor},
        {"Test_QuaternionDotProduct", Test_QuaternionDotProduct},
        {"Test_QuaternionAngleBetween", Test_QuaternionAngleBetween},
        {"Test_QuaternionSubtractionOperator", Test_QuaternionSubtractionOperator},
        {"Test_Timer", Test_Timer},
        {"Test_InputParser", Test_InputParser},
    };

    if (argc > 1)
    {
        try
        {
            return funcs.at(argv[1])();
        }
        catch (const std::out_of_range& e)
        {
            return fail_test();
        }
    }
    else
    {
        return fail_test();
    }
}
