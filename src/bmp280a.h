#ifndef ARWAIN_BMP280_H
#define ARWAIN_BMP280_H

#include "bmp280.h"

int init_bmp280(bmp280_dev& bmp, bmp280_config &conf, bmp280_uncomp_data& uncomp_data, const double sealevelpressure);
double altitude_from_pressure_and_temperature(const double pressure, const double temperature);

#endif
