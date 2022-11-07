#include <gtest/gtest.h>
#include <iostream>
#include <thread>
#include <chrono>
#include <sstream>

#include "timers.hpp"

TEST(Timers, ScopedTimerConstructors)
{
    EXPECT_NO_THROW(arwain::Timers::ScopedTimer{"timername"});
    EXPECT_NO_THROW(arwain::Timers::ScopedTimer("timername", std::cout));
}

TEST(Timers, ScopedTimerDestructor)
{
   std::stringstream string;
   {
      arwain::Timers::ScopedTimer timer("timername", string);
      std::this_thread::sleep_for(std::chrono::nanoseconds{1000000});
   }

    std::string temp;
    std::string temp2;
    int temp_int;
     while(!string.eof()) {
      string >> temp; 
      if(std::stringstream(temp) >> temp_int) {
         temp2 += temp;
      }
     }
    std::cout << temp2 <<std::endl;
    temp_int = stoi(temp2);

   if(temp_int >= 1000000)
   {
    SUCCEED();
   }else
   {
    FAIL();
   }
}




TEST(Timers, CountDownTimerConstructor)
{
   EXPECT_NO_THROW(arwain::Timers::CountdownTimer{1000});
}

TEST(Timers, CountDownTimerFinished)
{
   arwain::Timers::CountdownTimer timer{100};
   ASSERT_EQ(timer.finished(), false);
   std::this_thread::sleep_for(std::chrono::milliseconds{100});
   ASSERT_EQ(timer.finished(), true);
}

TEST(Timers, CountDownTimerReset)
{
   arwain::Timers::CountdownTimer timer{100};
   ASSERT_EQ(timer.finished(), false);
   std::this_thread::sleep_for(std::chrono::milliseconds{90});
   timer.reset();
   ASSERT_EQ(timer.finished(), false);
   std::this_thread::sleep_for(std::chrono::milliseconds{100});
   ASSERT_EQ(timer.finished(), true);
}





