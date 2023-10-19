#include <string>
#include <iostream>

#include <arwain/input_parser.hpp>
#include <arwain/devices/rfm95w.hpp>

int main(int argc, char* argv[])
{
    RFM95W<LinuxSpiDevice> transmitter{"/dev/spidev0.1", false};
    arwain::InputParser input{argc, argv};
    if (input.contains("--cmd"))
    {
        auto message = "C." + input.get_cmd_option("--cmd");
        transmitter.send_message(message);
        std::cout << "Sent message: " << message << std::endl;
    }
}