#include <gtest/gtest.h>

#include "arwain/service_manager.hpp"

TEST(ServiceManager, register_unregister)
{
    // 'service' has a random junk address (at least in g++).
    // This is safe as long as 'service' is never dereferenced.
    IArwainService* service;
    ServiceManager::register_service(service, "A");
    EXPECT_EQ(service, ServiceManager::get_service<IArwainService>("A"));
    ServiceManager::unregister_service("A");
    EXPECT_EQ(ServiceManager::get_service<IArwainService>("A"), nullptr);
}

int main(int argc, char** argv)
{
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
