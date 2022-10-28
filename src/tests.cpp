#include <map>
#include <random>
#include <iostream>
#include <functional>

#include "quaternion.hpp"
#include "input_parser.hpp"

namespace Random
{
    /** \brief Generates a random double in the interval [-10, 10).
     * \return Double in [-10, 10).
     */
    double Double()
    {
        static std::random_device rd;
        static std::mt19937 mt(rd());
        static std::uniform_real_distribution<double> dist(-10.0, 10.0);
        return dist(mt);
    }

    /** \brief Generates a random double in the interval [lower_bound, upper_bound). 
     * \param lower_bound Smallest value that can be returned.
     * \param upper_bound Lowest upper bound on returnable values.
     * \return Double in [lower_bound, upper_bound).
     */
    double DoubleBetween(const double lower_bound, const double upper_bound)
    {
        static std::random_device rd;
        static std::mt19937 mt(rd());
        std::uniform_real_distribution<double> dist(lower_bound, upper_bound);
        return dist(mt);
    }
}

/** \brief Return this to fail a test.
 * \return Integer 1, to indicate failed execution in UNIX-like environment.
 */
int fail_test()
{
    return 1;
}

/** \brief Return this to pass a test.
 * \return Integer 0, to indicate successful execution in UNIX-like environment.
 */
int pass_test()
{
    return 0;
}

/** \brief Checks that two doubles are 'equal' to within a certain (optional specified) tolerances.
 * \param num1 First double to compare.
 * \param num2 Second double to compare.
 * \param tolerance [optional] The two doubles should differ by no more than this to be considered equal.
 * \return Boolean indicating whether the two doubles are equal according to the given definition.
 */
bool is_close(double num1, double num2, double tolerance = 0.0000001)
{
    return std::abs(num1 - num2) < tolerance;
}

/** \brief Testing that the unit test framework is configured correctly.
 * \return 0 for test pass, 1 for test fail.
 */
int Test_Test()
{
    return pass_test();
}

/* -------------------------------------- QUATERNION_TESTS ------------------------------------- */

/** \brief Test that the 4-element quaternion constructor produces valid output.
 * \return 0 for test pass, 1 for test fail.
 */
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

/** \brief The inverse of a versor is equal to its conjugate. The general inverse should agree with that given by
 * wolframalpha, e.g. https://www.wolframalpha.com/input/?i=inverse+of+quaternion+1%2B0i%2B0j%2B2k+**+%28-1i%2B3%2B4j%2B3k%29
 * \return 0 for test pass, 1 for test fail.
 */
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

    // General quaternion inverse, as compard to WolframAlpha result.
    Quaternion q1{0.089732334, 0.036265566, 0.4167531, 0.878915397};
    Quaternion q1i = q1.inverse();
    Quaternion q1i_expected{0.0939072, -0.0379529, -0.436143, -0.919808};
    passing &= is_close(q1i.w, q1i_expected.w, 0.0001);
    passing &= is_close(q1i.x, q1i_expected.x, 0.0001);
    passing &= is_close(q1i.y, q1i_expected.y, 0.0001);
    passing &= is_close(q1i.z, q1i_expected.z, 0.0001);

    Quaternion q2{0.143179759, 0.53385755, 0.366605684, 0.100738656};
    Quaternion q2i = q2.inverse();
    Quaternion q2i_expected{0.31814, -1.18621, -0.814585, -0.223838};
    passing &= is_close(q2i.w, q2i_expected.w, 0.0001);
    passing &= is_close(q2i.x, q2i_expected.x, 0.0001);
    passing &= is_close(q2i.y, q2i_expected.y, 0.0001);
    passing &= is_close(q2i.z, q2i_expected.z, 0.0001);
    
    Quaternion q3{0.004420628, 0.616040595, 0.840586052, 0.798462594};
    Quaternion q3i = q3.inverse();
    Quaternion q3i_expected{0.00256469, -0.357404, -0.487677, -0.463239};
    passing &= is_close(q3i.w, q3i_expected.w, 0.0001);
    passing &= is_close(q3i.x, q3i_expected.x, 0.0001);
    passing &= is_close(q3i.y, q3i_expected.y, 0.0001);
    passing &= is_close(q3i.z, q3i_expected.z, 0.0001);
    
    Quaternion q4{0.463493268, 0.516989217, 0.820006171, 0.978068835};
    Quaternion q4i = q4.inverse();
    Quaternion q4i_expected{0.219547, -0.244887, -0.38842, -0.463291};
    passing &= is_close(q4i.w, q4i_expected.w, 0.0001);
    passing &= is_close(q4i.x, q4i_expected.x, 0.0001);
    passing &= is_close(q4i.y, q4i_expected.y, 0.0001);
    passing &= is_close(q4i.z, q4i_expected.z, 0.0001);
    
    Quaternion q5{0.701366791, 0.938252281, 0.837144943, 0.37816936};
    Quaternion q5i = q5.inverse();
    Quaternion q5i_expected{0.316493, -0.423388, -0.377763, -0.17065};
    passing &= is_close(q5i.w, q5i_expected.w, 0.0001);
    passing &= is_close(q5i.x, q5i_expected.x, 0.0001);
    passing &= is_close(q5i.y, q5i_expected.y, 0.0001);
    passing &= is_close(q5i.z, q5i_expected.z, 0.0001);
    
    return passing ? pass_test() : fail_test();
}

/** \brief Quaternions add element-wise.
 * \return 0 for test pass, 1 for test fail.
 */
int Test_QuaternionSum()
{
    bool passing = true;
    
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

/** \brief A rotor quaternion can be constructed from a real number and a 3-vector.
 * The real number is the angle by which the rotor will rotate a vector, and the 3-vector
 * is the axis about which the rotation is performed.
 * \return 0 for test pass, 1 for test fail.
 */
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

/** \brief The default quaternion constructor creates the unit real quaternion.
 * \return 0 for test pass, 1 for test fail.
 */
int Test_QuaternionDefaultConstructor()
{
    bool passing = true;

    for (int i = 0; i < 1000; i++)
    {
        try
        {
            Quaternion q1;
            passing &= (q1 == Quaternion{1, 0, 0, 0});
        }
        catch (const std::exception& e)
        {
            passing = false;
        }
    }

    return passing ? pass_test() : fail_test();
}

/** \brief A quaternion can be constructed from a vector; it will have zero real part, and the vector part
 * will be equal to the supplied vector.
 * \return 0 for test pass, 1 for test fail.
 */
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

/** \brief The quaternion dot product is the same as for other vectors, i.e. the sum of element-wise products.
 * \return 0 for test pass, 1 for test fail.
 */
int Test_QuaternionDotProduct()
{
    bool passing = true;

    for (int i = 0; i < 1000; i++)
    {
        Quaternion q1{Random::Double(), Random::Double(), Random::Double(), Random::Double()};
        Quaternion q2{Random::Double(), Random::Double(), Random::Double(), Random::Double()};

        double dot_product = Quaternion::dot(q1, q2);

        passing &= (dot_product == ((q1.w * q2.w) + (q1.x * q2.x) + (q1.y * q2.y) + (q1.z * q2.z)));
    }

    return passing ? pass_test() : fail_test();
}

/** \brief Test computation of the angle between two rotor quaternions. Note that this concept
 * is only defined for unit quaternions.
 * \return 0 for test pass, 1 for test fail.
 */
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

/** \brief Test subtraction of two quaternions. Quaternion subtraction is element-wise.
 * \return 0 for test pass, 1 for test fail.
 */
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

/** \brief Test unary subtraction i.e. negation of a quaternion. All elements are negated.
 * \return 0 for test pass, 1 for test fail.
 */
int Test_QuaternionUnarySubtraction()
{
    bool passing = true;

    for (int i = 0; i < 1000; i++)
    {
        Quaternion q1{Random::Double(), Random::Double(), Random::Double(), Random::Double()};
        Quaternion q2 = -q1;

        passing &= (q1.w == -q2.w);
        passing &= (q1.x == -q2.x);
        passing &= (q1.y == -q2.y);
        passing &= (q1.z == -q2.z);
    }

    return passing ? pass_test() : fail_test();
}

/** \brief A unit quaternion has norm of one, within the tolerances of floating point arithmetic.
 * \return 0 for test pass, 1 for test fail.
 */
int Test_QuaternionIsNormal()
{
    bool passing = true;

    for (int i = 0; i < 1000; i++)
    {
        Quaternion q1{Random::Double(), Random::Double(), Random::Double(), Random::Double()};
        Quaternion q2{Random::Double(), Random::Double(), Random::Double(), Random::Double()};
        q1 = q1.unit();

        passing &= q1.isNormal();
        passing &= !(q2.isNormal());
    }

    return passing ? pass_test() : fail_test();
}

/** \brief The vector part of a quaternion is simply a 3-vector formed of elements [x, y, z].
 * \return 0 for test pass, 1 for test fail.
 */
int Test_QuaternionVectorPart()
{
    bool passing = true;

    for (int i = 0; i < 1000; i++)
    {
        Quaternion q1{Random::Double(), Random::Double(), Random::Double(), Random::Double()};
        double x = q1.x;
        double y = q1.y;
        double z = q1.z;
        passing &= (q1.vector_part() == Vector3{q1.x, q1.y, q1.z});
    }

    return passing ? pass_test() : fail_test();
}

/** \brief Two quaternions are equal if all of their elements are equal.
 * \return 0 for test pass, 1 for test fail.
 */
int Test_QuaternionEqualityOperator()
{
    bool passing = true;

    for (int i = 0; i < 1000; i++)
    {
        // Any quaternion should be equal to itself, and any random quaternion is (almost certainly) not equal to
        // another random quaternion.
        {
            Quaternion q1{Random::Double(), Random::Double(), Random::Double(), Random::Double()};
            Quaternion q2{Random::Double(), Random::Double(), Random::Double(), Random::Double()};
            passing &= !(q1 == q2);
            passing &= (q1 == q1);
        }

        // Two quaternions are the same except for element w
        {
            Quaternion q1{Random::Double(), 1.5, 1.5, 1.5};
            Quaternion q2{Random::Double(), 1.5, 1.5, 1.5};
            passing &= !(q1 == q2);
        }

        // Two quaternions are the same except for element x
        {
            Quaternion q1{1.5, Random::Double(), 1.5, 1.5};
            Quaternion q2{1.5, Random::Double(), 1.5, 1.5};
            passing &= !(q1 == q2);
        }

        // Two quaternions are the same except for element y
        {
            Quaternion q1{1.5, 1.5, Random::Double(), 1.5};
            Quaternion q2{1.5, 1.5, Random::Double(), 1.5};
            passing &= !(q1 == q2);
        }

        // Two quaternions are the same except for element z
        {
            Quaternion q1{1.5, 1.5, 1.5, Random::Double()};
            Quaternion q2{1.5, 1.5, 1.5, Random::Double()};
            passing &= !(q1 == q2);
        }
    }

    return passing ? pass_test() : fail_test();
}

/** \brief We can send a string representation of a quaternion using the stream operators.
 * \return 0 for test pass, 1 for test fail.
 */
int Test_QuaternionOutputStreamOperator()
{
    bool passing = true;

    for (int i = 0; i < 1000; i++)
    {
        Quaternion q1{Random::Double(), Random::Double(), Random::Double(), Random::Double()};
        try
        {
            std::cout << q1 << "\n";
        }
        catch (const std::exception& e)
        {
            passing = false;
        }
    }

    return passing ? pass_test() : fail_test();
}

/** \brief The quaternion product is defined here:
 * https://colin-broderick.medium.com/deriving-the-quaternion-product-a22858c40921
 * \return 0 for test pass, 1 for test fail.
 */
int Test_QuaternionProduct()
{
    bool passing = true;

    for (int i = 0; i < 1000; i++)
    {
        // Generate eight random numbers, then build two quaternions from them,
        // then do the product manually, and confirm the quaternion product is the same.

        double p1 = Random::Double();
        double p2 = Random::Double();
        double p3 = Random::Double();
        double p4 = Random::Double();
        double q1 = Random::Double();
        double q2 = Random::Double();
        double q3 = Random::Double();
        double q4 = Random::Double();

        Quaternion p{p1, p2, p3, p4};
        Quaternion q{q1, q2, q3, q4};

        Quaternion expected_product = p * q;
        Quaternion actual_product{
            p1*q1 - p2*q2 - p3*q3 - p4*q4,
            p1*q2 + p2*q1 + p3*q4 - p4*q3,
            p1*q3 - p2*q4 + p3*q1 + p4*q2,
            p1*q4 + p2*q3 - p3*q2 + p4*q1
        };

        passing &= (expected_product == actual_product);
    }

    return passing ? pass_test() : fail_test();
}

/** \brief We generate some quaternions, and test that the slerp of them is in agreement with an
 * an independent calculation. The independent calculator is here (on 27/10/2022):
 * https://www.euclideanspace.com/maths/algebra/realNormedAlgebra/quaternions/slerp/index.htm
 * \return 0 for test pass, 1 for test fail.
 */
int Test_QuaternionSlerp()
{
    bool passing = true;

    {
        Quaternion q1{1, 0, 0, 0};
        Quaternion q2{0, 1, 0, 0};
        Quaternion q3{0.8837656300886934, 0.46792981426057344, 0, 0};
        double t = 0.31;
        Quaternion q4 = Quaternion::slerp(q1, q2, t);
        passing &= is_close(q3.w, q4.w);
        passing &= is_close(q3.x, q4.x);
        passing &= is_close(q3.y, q4.y);
        passing &= is_close(q3.z, q4.z);
    }

    {
        Quaternion q1{1, 0, 0, 0};
        Quaternion q2{-1, 0, 0, 0};
        double t = 0.45;
        Quaternion q4 = Quaternion::slerp(q1, q2, t);
        passing &= (q1 == q4);
    }
    
    {
        Quaternion q1{0.99, 0, 0, 0};
        Quaternion q2{-0.99, 0, 0, 0};
        double t = 0.72;
        Quaternion q3{0.9940009637008222, 0, 0, 0};
        Quaternion q4{-0.9940009637008222, 0, 0, 0};
        Quaternion q5 = Quaternion::slerp(q1, q2, t);
        passing &= ((q3 == q5) || (q4 == q5));
    }

    {
        Quaternion q1{0.951189731, 0.210262993, 0.220275517, 0.050062617};
        Quaternion q2{0.951189731, 0.220275517, 0.210262993, 0.050062617};
        double t = 0.77;
        Quaternion q3{0.9512066194738531, 0.21797647460766956, 0.2125696796489197, 0.050063505867037415};
        Quaternion q4 = Quaternion::slerp(q1, q2, t);
        passing &= is_close(q3.w, q4.w, 0.001);
        passing &= is_close(q3.x, q4.x, 0.001);
        passing &= is_close(q3.y, q4.y, 0.001);
        passing &= is_close(q3.z, q4.z, 0.001);
    }

    return passing ? pass_test() : fail_test();
}

/** \brief Quaternion::nslerp calls Quaternion::slerp, then normalizes the result before returning.
 * \return 0 for test pass, 1 for test fail.
 */
int Test_QuaternionNormSlerp()
{
    bool passing = true;

    for (int i = 0; i < 1000; i++)
    {
        Quaternion q1{Random::Double(), Random::Double(), Random::Double(), Random::Double()};
        Quaternion q2{Random::Double(), Random::Double(), Random::Double(), Random::Double()};
        double t = Random::DoubleBetween(0, 1);
        Quaternion q3 = Quaternion::nslerp(q1, q2, t);
        passing &= q3.isNormal();
    }

    return passing ? pass_test() : fail_test();
}

int Test_InputParser()
{
    int j = 2;
    std::string program = "arwain_test";
    std::string command = "hello";
    char* input_array[2] = {program.data(),command.data()};
    InputParser parser(j, input_array);
    
    if(parser.contains("hello"))
    {
        return pass_test();
    }
    else
    {
        return fail_test();
    }
}

int Test_InputParserGetCmdOption()
{
    int j = 3;
    std::string program = "arwain_test";
    std::string command = "hello";
    std::string paramater = "1";
    char* input_array[3] = {program.data(),command.data(), paramater.data()};
    InputParser parser(j, input_array);

    if(parser.getCmdOption("hello") == "1")
    {
        return pass_test();
    }
    else
    {
        return fail_test();   
    }
}

int Test_InputParserGetCmdOption_error()
{
    int j = 2;
    std::string program = "arwain_test";
    std::string command = "hello";
    char* input_array[2] = {program.data(),command.data()};
    InputParser parser(j, input_array);

     if(parser.getCmdOption("hello") == "")
    {
        return pass_test();
    }
    else
    {
        return fail_test();   
    }
}

int Test_InputParserContainerError()
{
    int j = 2;
    std::string program = "arwain_test";
    std::string command = "hello";
    char* input_array[2] = {program.data(),command.data()};
    InputParser parser(j, input_array);

    if(parser.contains("bye") == false)
    {
        return pass_test();
    }
    else
    {
        return fail_test();
    }
}

/** \brief Defines and execute tests according to string command line argument.
 * Should be called as
 *     ./binary_name Test_NameOfTest
 * \param argc Argument count.
 * \param argv Array of command line arguments.
 * \return 0 for test pass, 1 for test fail.
 */
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
        {"Test_QuaternionUnarySubtraction", Test_QuaternionUnarySubtraction},
        {"Test_QuaternionIsNormal", Test_QuaternionIsNormal},
        {"Test_QuaternionVectorPart", Test_QuaternionVectorPart},
        {"Test_QuaternionEqualityOperator", Test_QuaternionEqualityOperator},
        {"Test_QuaternionOutputStreamOperator", Test_QuaternionOutputStreamOperator},
        {"Test_QuaternionProduct", Test_QuaternionProduct},
        {"Test_QuaternionSlerp", Test_QuaternionSlerp},
        {"Test_QuaternionNormSlerp", Test_QuaternionNormSlerp},
        {"Test_InputParser", Test_InputParser},
        {"Test_InputParserGetCmdOption",Test_InputParserGetCmdOption},
        {"Test_InputParserGetCmdOption_error",Test_InputParserGetCmdOption_error},
        {"Test_InputParserContainerError", Test_InputParserContainerError},
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
