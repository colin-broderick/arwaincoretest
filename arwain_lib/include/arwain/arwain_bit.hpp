#ifndef _ARWAIN_BIT_HPP
#define _ARWAIN_BIT_HPP

enum class ArwainBIT
{
    ALL_OK,
    IMU1_FAILED,
    IMU2_FAILED,
    IMU3_FAILED,
    PRESSURE_FAILED,
    MAG_FAILED,
    LORA_FAILED,
    UWB_FAILED,
    CONFIG_FAILED,
};

ArwainBIT arwain_bit(int argc, char **argv);

#endif
