#include <string>
#include <iostream>

#include "input_parser.hpp"
#include "lora.hpp"

int main(int argc, char* argv[])
{
    LoRa transmitter{"/dev/spidev0.1", false};
    InputParser input{argc, argv};
    if (input.contains("-cmd"))
    {
        auto message = "C." + input.getCmdOption("-cmd");
        transmitter.send_message(message);
        std::cout << "Sent message: " << message << std::endl;
    }
}