#include <cmath>

#include "android_fusion.hpp"

namespace
{
    const double DEFAULT_GYRO_VAR = 1e-7;
    const double DEFAULT_GYRO_BIAS_VAR = 1e-12;
    const double GEOMAG_GYRO_VAR = 1e-4;
    const double GEOMAG_GYRO_BIAS_VAR = 1e-8;

    const double DEFAULT_ACC_STDEV = 0.015;
    const double DEFAULT_MAG_STDEV = 0.1;
    const double GEOMAG_ACC_STDEV = 0.05;
    const double GEOMAG_MAG_STDEV = 0.1;

    const double SYMMETRY_TOLERANCE = 1e-10;
    
    const double NOMINAL_GRAVITY = 9.81;
    const double FREE_FALL_THRESHOLD = 0.1;

    const double MAX_VALID_MAGNETIC_FIELD = 100;
    const double MAX_VALID_MAGNETIC_FIELD_SQ = MAX_VALID_MAGNETIC_FIELD * MAX_VALID_MAGNETIC_FIELD;

    const double MIN_VALID_MAGNETIC_FIELD = 10;
    const double MIN_VALID_MAGNETIC_FIELD_SQ = MIN_VALID_MAGNETIC_FIELD * MIN_VALID_MAGNETIC_FIELD;

    const double MIN_VALID_CROSS_PRODUCT_MAG = 1e-3;
    const double MIN_VALID_CROSS_PRODUCT_MAG_SQ = MIN_VALID_CROSS_PRODUCT_MAG * MIN_VALID_CROSS_PRODUCT_MAG;

    const double WVEC_EPS = 1e-4 / std::sqrt(3.0);
}

AndroidFusion::AndroidFusion()
{
    // TODO Need matrices to set Phi

    Ba = {0, 0, 1};
    Bm = {0, 1, 0};

    x0 = {0, 0, 0, 0};
    x1 = {0, 0, 0};

    init();
}

void AndroidFusion::init(int mode)
{
    mInitState = 0;
    mGyroRate = 0;
    mCount[0] = 0;
    mCount[1] = 0;
    mCount[2] = 0;

    mData.clear();
    mMode = mode;

    if (mMode != FUSION_NOGYRO) // Normal mode
    {
        mParam.gyroVar = DEFAULT_GYRO_VAR;
        mParam.gyroBiasVar = DEFAULT_GYRO_BIAS_VAR;
        mParam.accStdev = DEFAULT_ACC_STDEV;
        mParam.magStdev = DEFAULT_MAG_STDEV;
    }
    else
    {
        mParam.gyroVar = GEOMAG_GYRO_VAR;
        mParam.gyroBiasVar = GEOMAG_GYRO_BIAS_VAR;
        mParam.accStdev = GEOMAG_ACC_STDEV;
        mParam.magStdev = GEOMAG_MAG_STDEV;
    }
}

void AndroidFusion::initFusion(const Quaternion& q, double dT)
{
    x0 = q;
    x1 = {0, 0, 0};

    const double dT2 = dT * dT;
    const double dT3 = dT2 * dT;

    const double q00 = mParam.gyroVar * dT + 0.33333 * mParam.gyroBiasVar * dT3;

    const double q11 = mParam.gyroBiasVar * dT;
    const double q10 = 0.5 * mParam.gyroBiasVar * dT2;
    const double q01 = q10;

    // TODO Need matrix for GQGt and P parts
}

bool AndroidFusion::hasEstimate() const
{
    return ((mInitState & MAG) || (mMode == FUSION_NOMAG)) &&
           ((mInitState & GYRO) || (mMode == FUSION_NOGYRO)) &&
           (mInitState & ACC);
}

bool AndroidFusion::checkInitComplete(int what, const Vector3& d, double dT)
{
    if (hasEstimate())
    {
        return true;
    }

    // TODO Need matrix for mdata parts
    if (what == ACC)
    {

    }
    else if (what == MAG)
    {

    }
    else if (what == GYRO)
    {

    }

    if (hasEstimate())
    {

    }

    return false;
}

void AndroidFusion::handleGyro(const Vector3& w, double dT)
{
    if (!checkInitComplete(GYRO, w, dT))
    {
        return;
    }

    predict(w, dT);
}

void AndroidFusion::handleAcc(const Vector3& a, float dT)
{
    if (!checkInitComplete(ACC, a, dT))
    {
        // todo need BAD_VALUE?
        return BAD_VALUE
    }

    const dobule l = a.magnitude();
    if (l < FREE_FALL_THRESHOLD)
    {
        return BAD_VALUE;
    }

    const double l_inv = 1.0 / l;

    if (mMode == FUSION_NOGYRO)
    {
        Vector3 w_dummy = x1;
        predict(w_dummy, dT);
    }
    if (mMode == FUSION_NOMAG)
    {
        Vector3 m = getRotationMatrix * Bm;
        update(m, Bm, mParam.magStdev);
    }

    Vector3 unityA = a * l_inv;
    const double d = std::sqrt(std::abs(l - NOMINAL_GRAVITY));
    const double p = l_inv * mParam.accStdev * std::exp(d);

    update(unityA, Ba, p);
    return NO_ERROR;
}

Vector3 AndroidFusion::getOrthogonal(const Vector3& v)
{
    Vector3 w;

    if (std::abs(v.x) <= std::abs(v.y) && std::abs(v.x) <= std::abs(v.z)) // if x smaller than y and z
    {
        w = {0, v.z, -v.y};
    }
    else if (std::abs(v.y) <= std::abs(v.z)) // else if y less than z
    {
        w = {v.z, 0, -v.x};
    }
    else
    {
        w = {v.y, -v.x, 0};
    }

    return w.normalized();
}

void AndroidFusion::update(const Vector3& z, const Vector3& Bi, double sigma)
{
    Quaternion q = this->x0;

    // TODO Need matrix implementation    
}
