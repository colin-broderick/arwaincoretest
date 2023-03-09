#ifndef _ARWAIN_OPERATING_MODES
#define _ARWAIN_OPERATING_MODES

namespace arwain
{
    enum class OperatingMode
    {
        Terminate,
        Inference,
        Idle,
        SelfTest,
        GyroscopeCalibration,
        MagnetometerCalibration,
        AccelerometerCalibration,
        TestSerial,
        TestStanceDetector,
        DataCollection,
        InvalidMode
    };
}

#endif
