#ifndef _TEST_BASE_HPP
#define _TEST_BASE_HPP

#define STUB

#ifdef DO_HARDWARE_TESTS
#define HARDWARE_TEST(x, y) TEST(x, y)
#else
#define HARDWARE_TEST(x, y) void f##x##y(void)
#endif

#endif
