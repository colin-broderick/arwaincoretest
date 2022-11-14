#ifndef _ARWAIN_RUNTIME_TESTS_HPP
#define _ARWAIN_RUNTIME_TESTS_HPP

namespace arwain
{
    int test_imu();
    int test_lora_tx();
    int test_lora_rx();
    #if USE_ROS
    int test_mag(int argc, char **argv);
    #else
    int test_mag();
    #endif
    int test_pressure();
    int test_ori(int rate);
    #if USE_UUBLA
    int test_uubla_integration();
    #endif
}

#endif
