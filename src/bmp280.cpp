#include "bmp280.h"

// Constructors ===================================================================================

arwain::BMP280::BMP280()
{
    // TODO
    sea_level_pressure = 1013.250;
    hypsometric = true;
    t_mode = OSAMPLE_1;
    p_mode = OSAMPLE_1;
    standby = STANDBY_10;
    filter = FILTER_off;
}

// Getters ========================================================================================

double arwain::BMP280::getTemperature()
{
    // TODO
    return 0;
}

double arwain::BMP280::getAltitude()
{
    // TODO
    return 0;
}

double arwain::BMP280::getAltitude()
{
    // TODO
    return 0;
}

void arwain::BMP280::calculateAltitude()
{
    // TODO
    if (this->hypsometric)
    {
        this->altitude = ((pow(this->sea_level_pressure / this->pressure,1.0/5.257)-1) * this->temperature+273.15)/0.0065;
    }
    else
    {
        this->altitude = 44330 * (1.0 - pow(this->pressure / 100.0 / this->sea_level_pressure, 0.1903));
    }
}

void arwain::BMP280::load_calibration()
{
    // TODO
}

unsigned char arwain::BMP280::read_register(unsigned char address, unsigned int num_bytes)
{
    // TODO
    return 0;
}

void arwain::BMP280::write_register(unsigned char address)
{
    // TODO
}