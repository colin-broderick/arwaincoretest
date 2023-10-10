#ifndef _ANDROID_FUSION_HPP
#define _ANDROID_FUSION_HPP

#include <vector>
#include <eigen3/Eigen/Dense>

#include <arwain/quaternion.hpp>
#include <arwain/vector3.hpp>

enum FUSION_MODE
{
    FUSION_9AXIS, // use accel gyro mag
    FUSION_NOMAG, // use accel gyro (game rotation, gravity)
    FUSION_NOGYRO, // use accel mag (geomag rotation)
    NUM_FUSION_MODE
};

class AndroidFusion
{
    private:
        Quaternion x0;
        Vector3 x1;

        double P[4]; // Predicated covariance matrix
        double GQGt[4]; // Process noise covariance matrix

    public:
        AndroidFusion();
        void init(int mode = FUSION_9AXIS);
        void handleGyro(const Vector3& angular_velocity, float dT);
        void handleAcc(const Vector3& acceleration, float dT);
        void handleMag(const Vector3& magnetic_field);
        Quaternion getAttitude() const;
        Vector3 getBias() const;
        void getRotationMatrix(double output[4]) const;
        bool hasEstimate() const;

    private:
        struct Paramater
        {
            double gyroVar;
            double gyroBiasVar;
            double accStdev;
            double magStdev;
        } mParam;

        double Phi[4];
        Vector3 Ba, Bm;
        uint32_t mInitState;
        double mGyroRate;
        Vector3 mData[3];
        size_t mCount[3];
        int mMode;

        enum {ACC=0x1, MAG=0x2, GYRO=0x4};
        bool checkInitComplete(int, const Vector3& angular_velocity, double d=0);
        void initFusion(const Quaternion& q0, double dT);
        void checkState();
        void predict(const Vector3& angular_velocity, double dT);
        void update(const Vector3& z, const Vector3& Bi, double sigma);
        // static mat34_t getF(const Quaternion& p);
        static Vector3 getOrthogonal(const Vector3& v);


};

#endif
