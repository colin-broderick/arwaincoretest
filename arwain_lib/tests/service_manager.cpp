#include <gtest/gtest.h>

#include "arwain/service_manager.hpp"

TEST(ServiceManager, register_unregister)
{
    // 'job' has a random junk address (at least in g++).
    // This is safe as long as 'job' is never dereferenced.
    IArwainJob* job;
    ServiceManager::register_service(job, "A");
    EXPECT_EQ(job, ServiceManager::get_service<IArwainJob>("A"));
    ServiceManager::unregister_service("A");
    EXPECT_EQ(ServiceManager::get_service<IArwainJob>("A"), nullptr);
}

int main(int argc, char** argv)
{
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
