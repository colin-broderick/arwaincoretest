#include <gtest/gtest.h>
#include <iostream>
#include <thread>
#include <chrono>
#include <sstream>

#include "arwain.hpp"
#include "timers.hpp"

extern std::streambuf* original_cout_buffer;

TEST(Timers__ScopedTimer, ScopedTimer)
{
    EXPECT_NO_THROW(Timers::ScopedTimer{"timername"});
    EXPECT_NO_THROW(Timers::ScopedTimer("timername", std::cout));
}

TEST(Timers__ScopedTimer, DestructorScopedTimer)
{
   std::stringstream string;
   {
      Timers::ScopedTimer timer("timername", string);
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

   EXPECT_GE(temp_int, 1000000);
}

TEST(Timers__CountdownTimer, CountdownTimer)
{
   EXPECT_NO_THROW(Timers::CountdownTimer{1000});
}

TEST(Timers__CountdownTimer, finished)
{
   Timers::CountdownTimer timer{100};
   ASSERT_EQ(timer.finished(), false);
   std::this_thread::sleep_for(std::chrono::milliseconds{100});
   ASSERT_EQ(timer.finished(), true);
}

TEST(Timers__CountdownTimer, reset)
{
   Timers::CountdownTimer timer{100};
   ASSERT_EQ(timer.finished(), false);
   std::this_thread::sleep_for(std::chrono::milliseconds{90});
   timer.reset();
   ASSERT_EQ(timer.finished(), false);
   std::this_thread::sleep_for(std::chrono::milliseconds{100});
   ASSERT_EQ(timer.finished(), true);
}

TEST(Timers__IntervalTimer, IntervalTimer)
{
   Timers::IntervalTimer<std::chrono::seconds> t1{1};
   std::chrono::seconds s{1};
   EXPECT_EQ(t1.interval, s);
   EXPECT_EQ(t1.stored_name, "no_name_set");
   Timers::IntervalTimer<std::chrono::seconds> t2{1, __FUNCTION__};
   EXPECT_EQ(t2.interval, s);
   EXPECT_EQ(t2.stored_name, __FUNCTION__);
}

TEST(Timers__IntervalTimer, await)
{
   std::cout.rdbuf(original_cout_buffer);

   Timers::IntervalTimer<std::chrono::milliseconds> t1{10};
   // If the timer does a wait, await returns true.
   EXPECT_TRUE(t1.await());
   // If the timer doesn't do a wait, because too much time has elapsed already, it returns false.
   // Force a too-long wait so await triggers a warning.
   sleep_ms(11);
   testing::internal::CaptureStdout();
   EXPECT_FALSE(t1.await());
   EXPECT_EQ("WARNING: Interval timer named \"no_name_set\" appears to be running slow\n", testing::internal::GetCapturedStdout());
    
   std::cout.rdbuf(nullptr);
}
