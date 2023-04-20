#include <string>
#include <iostream>

#include "input_parser.hpp"
#include "lora.hpp"

int main(int argc, char* argv[])
{
    LoRa<SPIDEVICEDRIVER> transmitter{"/dev/spidev0.1", false};
    InputParser input{argc, argv};
    if (input.contains("-cmd"))
    {
        auto message = "C." + input.get_cmd_option("-cmd");
        transmitter.send_message(message);
        std::cout << "Sent message: " << message << std::endl;
    }
}