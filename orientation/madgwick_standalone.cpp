#include <iostream>
#include <fstream>
#include <vector>
#include <sstream>
#include <algorithm>

#include "madgwick.h"
#include "input_parser.h"

#define PI 3.14159265359

float frequency = 120;
float beta = 0.1;

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
    if (!input_parser.contains("-file"))
    {
        std::cout << "Usage:\n";
        std::cout << "  ./arwain_file_inference -file <filename> [-beta <beta=0.1>] [-freq <frequency=120>]\n";
        std::cout << "The file should contain either 6 or 9 comma-separated columns, in the order\n";
        std::cout << "  gyro_x, gyro_y, gyro_z, accel_x, accel_y, accel_z, mag_x, mag_y, mag_z\n";
        return 0;
    }
    if (input_parser.contains("-beta"))
    {
        std::stringstream(input_parser.getCmdOption("-beta")) >> beta;
    }
    if (input_parser.contains("-freq"))
    {
        std::stringstream(input_parser.getCmdOption("-freq")) >> frequency;
    }
    if (input_parser.contains("-xbias"))
    {
        std::stringstream(input_parser.getCmdOption("-xbias")) >> x_bias;
    }
    if (input_parser.contains("-ybias"))
    {
        std::stringstream(input_parser.getCmdOption("-ybias")) >> y_bias;
    }
    if (input_parser.contains("-zbias"))
    {
        std::stringstream(input_parser.getCmdOption("-zbias")) >> z_bias;
    }

    // Create orientation filter.
    std::cout << "Creating Madgwick filter with frequency " << frequency << " and gain " << beta << "\n";
    arwain::Madgwick filter{frequency, beta};

    // Open input and output file handles.
    std::string filename = input_parser.getCmdOption("-file");
    std::ifstream inputfile{filename};
    std::ofstream outputfile{filename+".processed.csv"};
    outputfile << "# Roll, Pitch, Yaw\n";

    // Vector to store each line of data as doubles.
    std::vector<double> data_line;

    // Skip the first ten lines to clear any junk.
    for (int i = 0; i < 10; i++)
    {
        std::getline(inputfile, line);
    }

    std::cout << "Processing " << filename << "\n";
    std::cout << "x_bias: " << x_bias << "\n";
    std::cout << "y_bias: " << y_bias << "\n";
    std::cout << "z_bias: " << z_bias << "\n";

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
        filter.update((data_line[0]-x_bias)/180*PI, (data_line[1]-y_bias)/180*PI, (data_line[2]-z_bias)/180*PI, data_line[3], data_line[4], data_line[5]);
        outputfile << filter.getRoll() << "," << filter.getPitch() << "," << filter.getYaw() << "\n";
    }

    std::cout << "Read " << count << " total lines\n";

    return 1;
}
