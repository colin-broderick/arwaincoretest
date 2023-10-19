#include <iostream>
#include <fstream>
#include <vector>
#include <sstream>
#include <algorithm>

#include <arwain/input_parser.hpp>

#include "madgwick.h"

#define PI 3.14159265359

double frequency = 120;
double beta = 0.1;

int accelindex, gyroindex;

int main(int argc, char **argv)
{
    // Create input parser.
    arwain::InputParser input_parser{argc, argv};
    
    std::string line;   // Stores a line from the file being processed.
    int count = 0;      // Counts how many lines have been processed.
    double v;           // Stores value pulled from a line.
    int commaCount;     // Counts the number of commas on a line.
    int delimiter;      // Stores the position of the first comma on a line.
    double x_bias = 0;
    double y_bias = 0;
    double z_bias = 0;

    // Check that a filename has been passed and process optional parameters.
    if (!input_parser.contains("--file") || !input_parser.contains("--accelindex") || !input_parser.contains("--gyroindex"))
    {
        std::cout << "Usage:\n";
        std::cout << "  ./arwain_file_inference -file <filename> [-beta <beta=0.1>] [-freq <frequency=120>]\n";
        std::cout << "The file should contain either 6 or 9 comma-separated columns, in the order\n";
        std::cout << "  gyro_x, gyro_y, gyro_z, accel_x, accel_y, accel_z, mag_x, mag_y, mag_z\n";
        return 0;
    }
    std::stringstream(input_parser.get_cmd_option("--accelindex")) >> accelindex;
    std::stringstream(input_parser.get_cmd_option("--gyroindex")) >> gyroindex;
    if (input_parser.contains("--beta"))
    {
        std::stringstream(input_parser.get_cmd_option("--beta")) >> beta;
    }
    if (input_parser.contains("--freq"))
    {
        std::stringstream(input_parser.get_cmd_option("--freq")) >> frequency;
    }
    if (input_parser.contains("--xbias"))
    {
        std::stringstream(input_parser.get_cmd_option("--xbias")) >> x_bias;
    }
    if (input_parser.contains("--ybias"))
    {
        std::stringstream(input_parser.get_cmd_option("--ybias")) >> y_bias;
    }
    if (input_parser.contains("--zbias"))
    {
        std::stringstream(input_parser.get_cmd_option("--zbias")) >> z_bias;
    }

    // Create orientation filter.
    std::cout << "Creating Madgwick filter with frequency " << frequency << " and gain " << beta << "\n";
    arwain::Madgwick filter{frequency, beta};

    // Open input and output file handles.
    std::string filename = input_parser.get_cmd_option("--file");
    std::ifstream inputfile{filename};
    std::ofstream outputfile{filename+".processed.csv"};
    outputfile << "W, X, Y, Z\n";

    // Vector to store each line of data as doubles.
    std::vector<double> data_line;

    // Skip the first ten lines to clear any junk.
    for (int i = 0; i < 1; i++)
    {
        std::getline(inputfile, line);
    }

    std::cout << "Processing " << filename << "\n";

    // Loop over lines in file, adding values to a buffer, updating filter, and writing result to output file.
    while (getline(inputfile, line))
    {
        // The last line is likely to be empty so don't attempt to read it.
        if (line.empty())
        {
            continue;
        }
        
        // Make sure the line buffer is clear before putting things in it.
        data_line.clear();

        // Add all comma-separated values on the line to a buffer.
        commaCount = std::count(line.begin(), line.end(), ',');
        for (int i = 0; i < commaCount; i++)
        {
            delimiter = line.find(",");
            std::stringstream(line.substr(0, delimiter)) >> v;
            data_line.push_back(v);
            line = line.substr(delimiter+1);
            count++;
        }
        std::stringstream(line) >> v;
        data_line.push_back(v);
        count++;
        
        // Update the orientation filter and write the result to the output file.
        filter.update(
            (data_line[gyroindex]-x_bias)/180.0*PI,
            (data_line[gyroindex+1]-y_bias)/180.0*PI,
            (data_line[gyroindex+2]-z_bias)/180.0*PI,     // Units of angular velocity must be rad/second
            data_line[accelindex],
            data_line[accelindex+1],
            data_line[accelindex+2]                       // Units of acceleration don't matter since only the norm is used
        );
        outputfile << filter.getW() << "," << filter.get_x() << "," << filter.get_y() << "," << filter.get_z() << "\n";
    }

    std::cout << "Read " << count << " total lines\n";

    return 0;
}
