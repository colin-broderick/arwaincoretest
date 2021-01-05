#include <string>
#include <sstream>
#include <ctime>
#include <fstream>
#include <iostream>


typedef struct euler_orientation_t
{
    float roll;
    float pitch;
    float yaw;
} euler_orientation_t;

typedef struct quat_orientation_t
{
    float w;
    float x;
    float y;
    float z;
} quat_orientation_t;

typedef struct velocity_t
{
    float x;
    float y;
    float z;
} velocity_t;

typedef struct position_t
{
    float x;
    float y;
    float z;
} position_t;

// Date/time as a string in the form YYYY_MM_DD_HH_mm_ss.
std::string datetimestring();

template <class T>
T get_config(std::string filename, std::string option)
{
    T ret = -1;
    std::string line;
    std::ifstream file(filename);
    while (getline(file, line))
    {
        // Skip this loop iteration if line does not contain sought option.
        if (line[0] == '[' || line[0] == '#' || line.empty())
        {
            continue;
        }
        auto delimiter = line.find("=");
        auto name = line.substr(0, delimiter);
        if (name != option)
        {
            continue;
        }

        // If option found get its value and break out of loop.
        auto value = line.substr(delimiter + 1);
        std::stringstream ss(value);
        ss >> ret;
        break;
    }

    // Warn if configuration option not found in file.
    if (ret == -1)
    {
        std::cout << option << " not found in config file - add it or expect undesirable behaviour" << std::endl;
    }
    file.close();
    return ret;
}
