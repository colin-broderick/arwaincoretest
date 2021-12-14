#ifndef _ARWAIN_RUNTIME_TESTS_HPP
#define _ARWAIN_RUNTIME_TESTS_HPP

namespace arwain
{
    int test_imu();
    int test_lora_tx();
    int test_lora_rx();
    #ifdef USEROS
    int test_mag(int argc, char **argv);
    #else
    int test_mag();
    #endif
    int test_pressure();
    int test_ori(int rate);
}

#endif
