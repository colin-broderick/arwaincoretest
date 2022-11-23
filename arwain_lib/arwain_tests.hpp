#ifndef _ARWAIN_RUNTIME_TESTS_HPP
#define _ARWAIN_RUNTIME_TESTS_HPP

namespace arwain
{
    enum class ReturnCode;

    arwain::ReturnCode test_imu(const std::string& i2c_bus, const int i2c_address);
    arwain::ReturnCode test_inference();
    arwain::ReturnCode test_lora_tx();
    arwain::ReturnCode test_lora_rx();
    #if USE_ROS
    int test_mag(int argc, char **argv);
    #else
    arwain::ReturnCode test_mag();
    #endif
    arwain::ReturnCode test_pressure();
    arwain::ReturnCode test_ori(int rate);
    #if USE_UUBLA
    arwain::ReturnCode test_uubla_integration();
    #endif
    arwain::ReturnCode interactive_test();
}

#endif
