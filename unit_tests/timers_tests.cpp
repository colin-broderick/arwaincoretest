#include <gtest/gtest.h>
#include <iostream>

#include "timers.hpp"

TEST(Timers, ScopedTimerConstructors)
{
    EXPECT_NO_THROW(arwain::Timers::ScopedTimer{"timername"});
    EXPECT_NO_THROW(arwain::Timers::ScopedTimer("timername", std::cout));
}
