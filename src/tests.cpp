#include <map>
#include <iostream>
#include <functional>

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

int main(int argc, char* argv[])
{
    std::map<std::string, std::function<int()>> funcs = {
        {"Test_Test", Test_Test},
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
