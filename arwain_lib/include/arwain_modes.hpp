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

inline std::ostream& operator<<(std::ostream& stream, arwain::OperatingMode token)
{
    switch (token)
    {
        case arwain::OperatingMode::Inference:
            stream << "Inference";
            break;
        case arwain::OperatingMode::Idle:
            stream << "Idle/autocalibrating";
            break;
        case arwain::OperatingMode::SelfTest:
            stream << "Self test";
            break;
        case arwain::OperatingMode::Terminate:
            stream << "Terminate";
            break;
        case arwain::OperatingMode::DataCollection:
            stream << "Data collection";
            break;
        case arwain::OperatingMode::GyroscopeCalibration:
            stream << "Gyroscope calibration";
            break;
        case arwain::OperatingMode::MagnetometerCalibration:
            stream << "Magnetometer calibration";
            break;
        case arwain::OperatingMode::AccelerometerCalibration:
            stream << "Accelerometer calibration";
            break;
        case arwain::OperatingMode::TestSerial:
            stream << "Test serial";
            break;
        case arwain::OperatingMode::TestStanceDetector:
            stream << "Test stance detector";
            break;
        default:
            stream << "Mode not specified";
            break;
    }
    return stream;
}

#endif
