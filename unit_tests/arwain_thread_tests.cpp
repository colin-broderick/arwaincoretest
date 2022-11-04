#include <gtest/gtest.h>
#include <chrono>

#include "arwain_thread.hpp"

namespace
{
    void test_func()
    {
        std::this_thread::sleep_for(std::chrono::milliseconds{100});
    }

    void test_func2(int i)
    {
        std::this_thread::sleep_for(std::chrono::milliseconds{13});
    }

    void test_func3(std::chrono::milliseconds ms)
    {
        std::this_thread::sleep_for(ms);
    }
}

TEST(ArwainThread, Constructors)
{
    EXPECT_NO_THROW(
        // "Normal" std thread invokation.
        ArwainThread th(test_func);
        th.join();
    );

    EXPECT_NO_THROW(
        // ARWAIN thread with a name a core affinity set.
        ArwainThread th2(test_func, "name");
        th2.join();
    );

    EXPECT_NO_THROW(
        // ARWAIN thread with a name, core affinity set, and function arguments.
        ArwainThread th3(test_func2, "name", 1);
        th3.join();
    );
}

TEST(ArwainThread, SetGetName)
{
    ArwainThread th(test_func, "TestName");
    std::this_thread::sleep_for(std::chrono::milliseconds{80});
    std::string name = th.get_name();
    th.join();
    EXPECT_EQ(name, "TestName");
}
