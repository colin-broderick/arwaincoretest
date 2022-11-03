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
    int i = 1;
    
    EXPECT_NO_THROW(
        // "Normal" std thread invokation.
        ArwainThread th(test_func);
        th.join();
    );

    EXPECT_NO_THROW(
        // ARWAIN thread with a name a core affinity set.
        ArwainThread th2(test_func, "name", {ArwainThread::AllCores});
        th2.join();
    );

    EXPECT_NO_THROW(
        ArwainThread th3(test_func2, "name", {ArwainThread::AllCores}, 1);
        th3.join();
    );
}

TEST(ArwainThread, GetName)
{
    ArwainThread th(test_func, "TestName", {ArwainThread::AllCores});
    std::this_thread::sleep_for(std::chrono::milliseconds{80});
    std::string name = th.get_name();
    th.join();
    EXPECT_EQ(name, "TestName");
}

TEST(ArwainThread, SetProcessorAffinity)
{
    ArwainThread th(test_func3, "TestName", {1, 2}, std::chrono::milliseconds{500});
    std::this_thread::sleep_for(std::chrono::milliseconds{100});
    cpu_set_t cpuset_expected;
    CPU_SET(1, &cpuset_expected);
    CPU_SET(2, &cpuset_expected);
    cpu_set_t cpuset_actual;
    pthread_getaffinity_np(th.native_handle(), sizeof(cpu_set_t), &cpuset_actual);

    int equal = CPU_EQUAL(&cpuset_actual, &cpuset_expected);

    EXPECT_NE(0, equal);
    // EXPECT_TRUE(false);
    th.join();
}
