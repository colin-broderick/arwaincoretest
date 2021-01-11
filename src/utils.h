#ifndef UTILS_H
#define UTILS_H

#include <string>
#include <sstream>
#include <ctime>
#include <fstream>
#include <vector>
#include <algorithm>
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
        std::cout << option << " not found in config file - add it or expect undesirable behaviour" << "\n";
    }
    file.close();
    return ret;
}


class InputParser{
    private:
        std::vector<std::string> tokens;
    
    public:
        InputParser(int &argc, char **argv)
        {
            // Add all tokens in command line to tokens vector.
            for (int i=1; i < argc; ++i)
            {
                this->tokens.push_back(std::string(argv[i]));
            }
        }

        const std::string& getCmdOption(const std::string &option) const
        {
            // Identify the position of a requested option and return the next token if found.
            std::vector<std::string>::const_iterator itr;
            itr = std::find(this->tokens.begin(), this->tokens.end(), option);
            if (itr != this->tokens.end() && ++itr != this->tokens.end())
            {
                return *itr;
            }
            static const std::string empty_string("");
            return empty_string;
        }

        bool contains(const std::string &option) const
        {
            // Check for the presense of a sought option.
            return std::find(this->tokens.begin(), this->tokens.end(), option)
                   != this->tokens.end();
        }
};

#endif
