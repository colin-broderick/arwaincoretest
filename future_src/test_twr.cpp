#include <iostream>

#include "serial.hpp"

namespace arwain
{
    int test_twr(const std::string& port)
    {
        std::cout << "Running TWR test\n";

        Serial serial{port, 115200};
        serial.flush();

        std::string message = serial.readline();
        std::cout << message << "\n";

        std::cout << "Finished running TWR test\n";
    
        return 0;
    }

}
