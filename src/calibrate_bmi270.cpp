#include <stdio.h>
#include <time.h>
#include <vector>
#include <iostream>
#include <fstream>

#include "bmi2.h"
#include "bmi270.h"
#include "imu_utils.h"

int main(int argc, char **argv)
{
    struct vec_scaled_output mag_d;
    std::vector<float> x, y, z;
    
    int n_sec = 30;
    
    if (init_bmi270(1, "no_calib_here") != 0)
    {
        printf("Node failed to start\n");
        return 1;
    }
    
    for(int i=0; i<n_sec*100; i++)
    {
        get_bmm150_data(&mag_d);
        x.push_back(mag_d.x);
        y.push_back(mag_d.y);
        z.push_back(mag_d.z);
        
        if (i%100 == 0){
            printf("T = %ds\n", i/100);
        }
        
        delay_ms(10);
    }
    
    std::string path = "./calib.txt";
    
    std::cout << "Saving calibration file to: " << path << "\n";
    
    float max_x = -10;
    float min_x = 10;
    float max_y = -10;
    float min_y = 10;
    float max_z = -10;
    float min_z = 10;
    
    for (auto i = x.begin(); i != x.end(); ++i)
    {
        if (*i < min_x)
            min_x = *i;
        if(*i > max_x)
            max_x = *i;
    }
    
     for (auto i = y.begin(); i != y.end(); ++i)
    {
        if (*i < min_y)
            min_y = *i;
        if(*i > max_y)
            max_y = *i;
    }
    
     for (auto i = z.begin(); i != z.end(); ++i)
    {
        if (*i < min_z)
            min_z = *i;
        if(*i > max_z)
            max_z = *i;
    }
    
    printf("X range is [%f, %f]\n", min_x, max_x);
    printf("Y range is [%f, %f]\n", min_y, max_y);
    printf("Z range is [%f, %f]\n", min_z, max_z);

    float offset_x = (max_x + min_x) / 2;
    float offset_y = (max_y + min_y) / 2;
    float offset_z = (max_z + min_z) / 2;
    
    float avg_delta_x = (max_x - min_x) / 2;
    float avg_delta_y = (max_y - min_y) / 2;
    float avg_delta_z = (max_z - min_z) / 2;

    float avg_delta = (avg_delta_x + avg_delta_y + avg_delta_z) / 3;

    float scale_x = avg_delta / avg_delta_x;
    float scale_y = avg_delta / avg_delta_y;
    float scale_z = avg_delta / avg_delta_z;

    
    std::ofstream myfile;
    myfile.open(path);
    myfile << "Offset_X, Offset_Y, Offset_Z, Scale_X, Scale_Y, Scale_Z\n";
    myfile << offset_x << "," << offset_y << "," << offset_z << ",";
    myfile << scale_x << "," << scale_y << "," << scale_z << "\n";
    myfile.close();

    float corrected_max_x = (max_x - offset_x) * scale_x;
    float corrected_min_x = (min_x - offset_x) * scale_x;
    
    float corrected_max_y = (max_y - offset_y) * scale_y;
    float corrected_min_y = (min_y - offset_y) * scale_y;

    float corrected_max_z = (max_z - offset_z) * scale_z;
    float corrected_min_z = (min_z - offset_z) * scale_z;
    
    printf("Corrected X range is [%f, %f]\n", corrected_min_x, corrected_max_x);
    printf("Corrected Y range is [%f, %f]\n", corrected_min_y, corrected_max_y);
    printf("Corrected Z range is [%f, %f]\n", corrected_min_z, corrected_max_z);

    
    return 0;
}

