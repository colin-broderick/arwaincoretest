#include <string>

#include "vector3.hpp"

#include "bmi2_defs.h"
#include "bmm150_defs.h"
#include "imu_utils.hpp"

class BMI270
{
    public:
        BMI270();
        int init_bmi270(int mag_enabled, std::string calib_file);
        int get_bmi270_data(struct vector3 *acc, struct vector3 *gyr);
        int get_bmm150_data(struct vector3 *mag);
        


        struct bmi2_sensor_data acce;
        struct bmi2_sensor_data gyro;
        struct bmi2_sensor_data magn;
        


    public:
        float acc_scale, gyr_scale;
};
