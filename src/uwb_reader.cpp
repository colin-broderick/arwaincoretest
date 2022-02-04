#include <tuple>
#include <string>
#include <sstream>
#include <algorithm>

#include "arwain.hpp"
#include "serial.hpp"
#include "logger.hpp"

namespace
{
    std::tuple<bool, double, double, double> extract_uwb_coords(const std::string& text)
    {
        size_t first_index = text.find("est");
        
        if (first_index != std::string::npos)
        {
            first_index += 4; // remove the text 'est['
            int second_index = text.find("]", first_index);
            std::string pos_string = text.substr(first_index, second_index - first_index);
            std::replace(pos_string.begin(), pos_string.end(), ',', ' '); // To allow stringstream operations to follow.
            std::istringstream ss{pos_string};
            double x, y, z;
            ss >> x >> y >> z;
            return {true, x, y, z};
        }
        else
        {
            return {false, 0, 0, 0};
        }
    }
}

void uwb_reader(const std::string& port, const int baudrate)
{
    // Quit immediately if not enabled by configuration.
    if (!arwain::config.use_uwb_positioning)
    {
        return;
    }

    Serial serial{port, baudrate};
    serial.write("reset\r");
    sleep_ms(1000);
    serial.write("\r\r");
    sleep_ms(1000);
    serial.write("les\r");
    sleep_ms(1000);

    while (arwain::system_mode != arwain::OperatingMode::Terminate)
    {
        switch (arwain::system_mode)
        {
            case arwain::OperatingMode::TestSerial:
            {
                serial.flush();
                while (arwain::system_mode == arwain::OperatingMode::TestSerial)
                {
                    std::string message = serial.readline();
                    std::cout << message << std::endl;
                }
                break;
            }
            case arwain::OperatingMode::Inference:
            {
                arwain::Logger uwb_log;
                if (arwain::config.log_to_file)
                {
                    uwb_log.open(arwain::folder_date_string + "/uwb_log.txt");
                    uwb_log << "time x y z" << "\n";
                }
                serial.flush();

                while (arwain::system_mode == arwain::OperatingMode::Inference)
                {
                    std::string message = serial.readline();

                    // Only log data if the est[x,y,z] substring is found in the message.
                    auto [found_position, x, y, z] = extract_uwb_coords(message);
                    if (found_position)
                    {
                        std::cout << x << " " << y << " " << z << std::endl;
                        auto time = std::chrono::system_clock::now();
                        uwb_log << time.time_since_epoch().count() << " " << x << " " << y << " " << z << "\n";
                    }
                }

                uwb_log.close();

                break;
            }
            default:
            {
                sleep_ms(100);
                break;
            }
        }
    }
}
