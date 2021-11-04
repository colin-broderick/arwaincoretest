//=============================================================================================
// MadgwickAHRS.c
//=============================================================================================
//
// Implementation of Madgwick's IMU and AHRS algorithms.
// See: http://www.x-io.co.uk/open-source-imu-and-ahrs-algorithms/
//
// From the x-io website "Open-source resources available on this website are
// provided under the GNU General Public Licence unless an alternative licence
// is provided in source."
//
// Date			Author          Notes
// 29/09/2011	SOH Madgwick    Initial release
// 02/10/2011	SOH Madgwick	Optimised for reduced CPU load
// 19/02/2012	SOH Madgwick	Magnetometer measurement is normalised
//
//=============================================================================================
//
// This file modified by Greeve to bring comments in line with the rest of the project,
// and to match APIs with another library, but core functionality is not changed.
//
// 16/02/2021    Colin Broderick    Modified to use doubles instead of floats where possible.
// 04/11/2021	 Colin Broderick	Removed fast inv sqrt as not necessary on new platforms
//                                  and can only reduce accuracy due to use of float instead of
//                                  double. Removed all other instances of floats being used
//                                  instead of doubles.
//
//=============================================================================================

#include <cmath>

#include "madgwick.hpp"

#define sampleFreqDef   512.0          // sample frequency in Hz
#define betaDef         0.1            // 2 * proportional gain

// Constructors -----------------------------------------------------------------------------------

/** \brief Constructor using default sample frequency.
 */
arwain::Madgwick::Madgwick()
{
	m_beta = betaDef;
	q0 = 1.0;
	q1 = 0.0;
	q2 = 0.0;
	q3 = 0.0;
	invSampleFreq = 1.0 / sampleFreqDef;
	anglesComputed = 0;
}

/** \brief Constructor using custom sample frequency.
 * \param sample_frequency The anticipated update frequency in Hz.
 */
arwain::Madgwick::Madgwick(double sample_frequency, double beta)
{
	m_beta = beta;
	q0 = 1.0;
	q1 = 0.0;
	q2 = 0.0;
	q3 = 0.0;
	invSampleFreq = 1.0 / sample_frequency;
	anglesComputed = 0;
}

// General methods --------------------------------------------------------------------------------

/** \brief Update internal orientation state using IMU and magnetometer data.
 *  \param timestamp Timestamp of the supplied data in nanoseconds.
 *  \param gx x-axis gyroscope value in rad/s
 *  \param gy y-axis gyroscope value in rad/s
 *  \param gz z-axis gyroscope value in rad/s
 *  \param ax x-axis accelerometer value in m/s2
 *  \param ay y-axis accelerometer value in m/s2
 *  \param az x-axis accelerometer value in m/s2
 *  \param mx x-axis magnetfic field strength
 *  \param my y-axis magnetfic field strength
 *  \param mz z-axis magnetfic field strength
 *  \return Nothing; updates internal state.
 */
void arwain::Madgwick::update(double gx, double gy, double gz, double ax, double ay, double az, double mx, double my, double mz)
{
	// Use IMU algorithm if magnetometer measurement invalid (avoids NaN in magnetometer normalisation)
	if ((mx == 0.0) && (my == 0.0) && (mz == 0.0))
	{
		update(gx, gy, gz, ax, ay, az);
		return;
	}

	double recipNorm;
	double s0, s1, s2, s3;
	double qDot1, qDot2, qDot3, qDot4;
	double hx, hy;
	double _2q0mx, _2q0my, _2q0mz, _2q1mx, _2bx, _2bz, _4bx, _4bz, _2q0, _2q1, _2q2, _2q3, _2q0q2, _2q2q3, q0q0, q0q1, q0q2, q0q3, q1q1, q1q2, q1q3, q2q2, q2q3, q3q3;

	// Rate of change of quaternion from gyroscope; qdot = 0.5 * q ** w
	// where w = (0, gx, gy, gz) and ** is the quaternion product.
	// See https://www.euclideanspace.com/physics/kinematics/angularvelocity/QuaternionDifferentiation2.pdf.
	qDot1 = 0.5 * (-q1 * gx - q2 * gy - q3 * gz);
	qDot2 = 0.5 * (q0 * gx + q2 * gz - q3 * gy);
	qDot3 = 0.5 * (q0 * gy - q1 * gz + q3 * gx);
	qDot4 = 0.5 * (q0 * gz + q1 * gy - q2 * gx);

	// Compute feedback only if accelerometer measurement valid (avoids NaN in accelerometer normalisation)
	if (!((ax == 0.0) && (ay == 0.0) && (az == 0.0)))
	{
		// Normalise accelerometer measurement
		recipNorm = invSqrt(ax * ax + ay * ay + az * az);
		ax *= recipNorm;
		ay *= recipNorm;
		az *= recipNorm;

		// Normalise magnetometer measurement
		recipNorm = invSqrt(mx * mx + my * my + mz * mz);
		mx *= recipNorm;
		my *= recipNorm;
		mz *= recipNorm;

		// Auxiliary variables to avoid repeated arithmetic
		_2q0mx = 2.0 * q0 * mx;
		_2q0my = 2.0 * q0 * my;
		_2q0mz = 2.0 * q0 * mz;
		_2q1mx = 2.0 * q1 * mx;
		_2q0 = 2.0 * q0;
		_2q1 = 2.0 * q1;
		_2q2 = 2.0 * q2;
		_2q3 = 2.0 * q3;
		_2q0q2 = 2.0 * q0 * q2;
		_2q2q3 = 2.0 * q2 * q3;
		q0q0 = q0 * q0;
		q0q1 = q0 * q1;
		q0q2 = q0 * q2;
		q0q3 = q0 * q3;
		q1q1 = q1 * q1;
		q1q2 = q1 * q2;
		q1q3 = q1 * q3;
		q2q2 = q2 * q2;
		q2q3 = q2 * q3;
		q3q3 = q3 * q3;

		// Reference direction of Earth's magnetic field
		hx = mx * q0q0 - _2q0my * q3 + _2q0mz * q2 + mx * q1q1 + _2q1 * my * q2 + _2q1 * mz * q3 - mx * q2q2 - mx * q3q3;
		hy = _2q0mx * q3 + my * q0q0 - _2q0mz * q1 + _2q1mx * q2 - my * q1q1 + my * q2q2 + _2q2 * mz * q3 - my * q3q3;
		_2bx = sqrtf(hx * hx + hy * hy);
		_2bz = -_2q0mx * q2 + _2q0my * q1 + mz * q0q0 + _2q1mx * q3 - mz * q1q1 + _2q2 * my * q3 - mz * q2q2 + mz * q3q3;
		_4bx = 2.0 * _2bx;
		_4bz = 2.0 * _2bz;

		// Gradient decent algorithm corrective step
		s0 = -_2q2 * (2.0 * q1q3 - _2q0q2 - ax) + _2q1 * (2.0 * q0q1 + _2q2q3 - ay) - _2bz * q2 * (_2bx * (0.5 - q2q2 - q3q3) + _2bz * (q1q3 - q0q2) - mx) + (-_2bx * q3 + _2bz * q1) * (_2bx * (q1q2 - q0q3) + _2bz * (q0q1 + q2q3) - my) + _2bx * q2 * (_2bx * (q0q2 + q1q3) + _2bz * (0.5 - q1q1 - q2q2) - mz);
		s1 = _2q3 * (2.0 * q1q3 - _2q0q2 - ax) + _2q0 * (2.0 * q0q1 + _2q2q3 - ay) - 4.0 * q1 * (1 - 2.0 * q1q1 - 2.0 * q2q2 - az) + _2bz * q3 * (_2bx * (0.5 - q2q2 - q3q3) + _2bz * (q1q3 - q0q2) - mx) + (_2bx * q2 + _2bz * q0) * (_2bx * (q1q2 - q0q3) + _2bz * (q0q1 + q2q3) - my) + (_2bx * q3 - _4bz * q1) * (_2bx * (q0q2 + q1q3) + _2bz * (0.5 - q1q1 - q2q2) - mz);
		s2 = -_2q0 * (2.0 * q1q3 - _2q0q2 - ax) + _2q3 * (2.0 * q0q1 + _2q2q3 - ay) - 4.0 * q2 * (1 - 2.0 * q1q1 - 2.0 * q2q2 - az) + (-_4bx * q2 - _2bz * q0) * (_2bx * (0.5 - q2q2 - q3q3) + _2bz * (q1q3 - q0q2) - mx) + (_2bx * q1 + _2bz * q3) * (_2bx * (q1q2 - q0q3) + _2bz * (q0q1 + q2q3) - my) + (_2bx * q0 - _4bz * q2) * (_2bx * (q0q2 + q1q3) + _2bz * (0.5 - q1q1 - q2q2) - mz);
		s3 = _2q1 * (2.0 * q1q3 - _2q0q2 - ax) + _2q2 * (2.0 * q0q1 + _2q2q3 - ay) + (-_4bx * q3 + _2bz * q1) * (_2bx * (0.5 - q2q2 - q3q3) + _2bz * (q1q3 - q0q2) - mx) + (-_2bx * q0 + _2bz * q2) * (_2bx * (q1q2 - q0q3) + _2bz * (q0q1 + q2q3) - my) + _2bx * q1 * (_2bx * (q0q2 + q1q3) + _2bz * (0.5 - q1q1 - q2q2) - mz);
		recipNorm = invSqrt(s0 * s0 + s1 * s1 + s2 * s2 + s3 * s3); // normalise step magnitude
		s0 *= recipNorm;
		s1 *= recipNorm;
		s2 *= recipNorm;
		s3 *= recipNorm;

		// Apply feedback step
		qDot1 -= m_beta * s0;
		qDot2 -= m_beta * s1;
		qDot3 -= m_beta * s2;
		qDot4 -= m_beta * s3;
	}

	// Integrate rate of change of quaternion to yield quaternion
	q0 += qDot1 * invSampleFreq;
	q1 += qDot2 * invSampleFreq;
	q2 += qDot3 * invSampleFreq;
	q3 += qDot4 * invSampleFreq;

	// Normalise quaternion
	recipNorm = invSqrt(q0 * q0 + q1 * q1 + q2 * q2 + q3 * q3);
	q0 *= recipNorm;
	q1 *= recipNorm;
	q2 *= recipNorm;
	q3 *= recipNorm;
	anglesComputed = 0;
}

/** \brief This is a dummy function to allow eFaroe and Madgwick filters to use the same API - timestamp is not used.
 *  \param timestamp Timestamp of the supplied data in nanoseconds.
 *  \param gx x-axis gyroscope value in rad/s
 *  \param gy y-axis gyroscope value in rad/s
 *  \param gz z-axis gyroscope value in rad/s
 *  \param ax x-axis accelerometer value in m/s2
 *  \param ay y-axis accelerometer value in m/s2
 *  \param az x-axis accelerometer value in m/s2
 *  \return Nothing; updates internal state.
 */
void arwain::Madgwick::update(double timestamp, double gx, double gy, double gz, double ax, double ay, double az)
{
	update(gx, gy, gz, ax, ay, az);
}

/** \brief This is a dummy function to allow eFaroe and Madgwick filters to use the same API - timestamp is not used.
 *  \param timestamp Timestamp of the supplied data in nanoseconds.
 *  \param gx x-axis gyroscope value in rad/s
 *  \param gy y-axis gyroscope value in rad/s
 *  \param gz z-axis gyroscope value in rad/s
 *  \param ax x-axis accelerometer value in m/s2
 *  \param ay y-axis accelerometer value in m/s2
 *  \param az x-axis accelerometer value in m/s2
 *  \param mx x-axis magnetfic field strength
 *  \param my y-axis magnetfic field strength
 *  \param mz z-axis magnetfic field strength
 *  \return Nothing; updates internal state.
 */
void arwain::Madgwick::update(double timestamp, double gx, double gy, double gz, double ax, double ay, double az, double mx, double my, double mz)
{
	update(gx, gy, gz, ax, ay, az, mx, my, mz);
}

/** \brief Update internal state using new IMU reading, without magnetometer data.
 *  \param gx x-axis gyroscope value in rad/s
 *  \param gy y-axis gyroscope value in rad/s
 *  \param gz z-axis gyroscope value in rad/s
 *  \param ax x-axis accelerometer value in m/s2
 *  \param ay y-axis accelerometer value in m/s2
 *  \param az x-axis accelerometer value in m/s2
 *  \return Nothing; updates internal state.
 */
void arwain::Madgwick::update(double gx, double gy, double gz, double ax, double ay, double az)
{
	double recipNorm;
	double s0, s1, s2, s3;
	double qDot1, qDot2, qDot3, qDot4;
	double _2q0, _2q1, _2q2, _2q3, _4q0, _4q1, _4q2 ,_8q1, _8q2, q0q0, q1q1, q2q2, q3q3;

	// Rate of change of quaternion from gyroscope; qdot = 0.5 * q ** w
	// where w = (0, gx, gy, gz) and ** is the quaternion product.
	// See https://www.euclideanspace.com/physics/kinematics/angularvelocity/QuaternionDifferentiation2.pdf.
	qDot1 = 0.5 * (-q1 * gx - q2 * gy - q3 * gz);
	qDot2 = 0.5 * (q0 * gx + q2 * gz - q3 * gy);
	qDot3 = 0.5 * (q0 * gy - q1 * gz + q3 * gx);
	qDot4 = 0.5 * (q0 * gz + q1 * gy - q2 * gx);

	// Compute feedback only if accelerometer measurement valid (avoids NaN in accelerometer normalisation)
	if (!((ax == 0.0) && (ay == 0.0) && (az == 0.0)))
	{
		// Normalise accelerometer measurement
		recipNorm = invSqrt(ax * ax + ay * ay + az * az);
		ax *= recipNorm;
		ay *= recipNorm;
		az *= recipNorm;

		// Auxiliary variables to avoid repeated arithmetic
		_2q0 = 2.0 * q0;
		_2q1 = 2.0 * q1;
		_2q2 = 2.0 * q2;
		_2q3 = 2.0 * q3;
		_4q0 = 4.0 * q0;
		_4q1 = 4.0 * q1;
		_4q2 = 4.0 * q2;
		_8q1 = 8.0 * q1;
		_8q2 = 8.0 * q2;
		q0q0 = q0 * q0;
		q1q1 = q1 * q1;
		q2q2 = q2 * q2;
		q3q3 = q3 * q3;

		// Gradient decent algorithm corrective step
		s0 = _4q0 * q2q2 + _2q2 * ax + _4q0 * q1q1 - _2q1 * ay;
		s1 = _4q1 * q3q3 - _2q3 * ax + 4.0 * q0q0 * q1 - _2q0 * ay - _4q1 + _8q1 * q1q1 + _8q1 * q2q2 + _4q1 * az;
		s2 = 4.0 * q0q0 * q2 + _2q0 * ax + _4q2 * q3q3 - _2q3 * ay - _4q2 + _8q2 * q1q1 + _8q2 * q2q2 + _4q2 * az;
		s3 = 4.0 * q1q1 * q3 - _2q1 * ax + 4.0 * q2q2 * q3 - _2q2 * ay;
		recipNorm = invSqrt(s0 * s0 + s1 * s1 + s2 * s2 + s3 * s3); // normalise step magnitude
		s0 *= recipNorm;
		s1 *= recipNorm;
		s2 *= recipNorm;
		s3 *= recipNorm;

		// Apply feedback step
		qDot1 -= m_beta * s0;
		qDot2 -= m_beta * s1;
		qDot3 -= m_beta * s2;
		qDot4 -= m_beta * s3;
	}

	// Integrate rate of change of quaternion to yield quaternion
	if (std::isnan(q0 + qDot1 * invSampleFreq))
	{
		return;
	}
	q0 += qDot1 * invSampleFreq;
	q1 += qDot2 * invSampleFreq;
	q2 += qDot3 * invSampleFreq;
	q3 += qDot4 * invSampleFreq;

	// Normalise quaternion
	recipNorm = invSqrt(q0 * q0 + q1 * q1 + q2 * q2 + q3 * q3);
	q0 *= recipNorm;
	q1 *= recipNorm;
	q2 *= recipNorm;
	q3 *= recipNorm;
	anglesComputed = 0;
}

/** \brief Calculates inverse square root of number.
 * \param x Find square root of this number.
 * \return The square root of x.
 */
double arwain::Madgwick::invSqrt(double x)
{
	return 1.0 / std::sqrt(x);
}

/** \brief Updates the internally-stored Euler angles.
 */
void arwain::Madgwick::computeAngles()
{
	roll = std::atan2(q0*q1 + q2*q3, 0.5 - q1*q1 - q2*q2);
	pitch = std::asin(-2.0 * (q1*q3 - q0*q2));
	yaw = std::atan2(q1*q2 + q0*q3, 0.5 - q2*q2 - q3*q3);
	anglesComputed = 1;
}

// Getters ----------------------------------------------------------------------------------------

double arwain::Madgwick::getW()
{
	return q0;
}

double arwain::Madgwick::getX()
{
	return q1;
}

double arwain::Madgwick::getY()
{
	return q2;
}

double arwain::Madgwick::getZ()
{
	return q3;
}

double arwain::Madgwick::getRoll()
{
	if (!anglesComputed)
	{
		computeAngles();
	}
	return roll * 57.29578;
}

double arwain::Madgwick::getPitch()
{
	if (!anglesComputed)
	{
		computeAngles();
	}
	return pitch * 57.29578;
}

double arwain::Madgwick::getYaw()
{
	if (!anglesComputed)
	{
		computeAngles();
	}
	return yaw * 57.29578 + 180.0;
}

double arwain::Madgwick::getRollRadians()
{
	if (!anglesComputed)
	{
		computeAngles();
	}
	return roll;
}

double arwain::Madgwick::getPitchRadians()
{
	if (!anglesComputed)
	{
		computeAngles();
	}
	return pitch;
}

double arwain::Madgwick::getYawRadians()
{
	if (!anglesComputed)
	{
		computeAngles();
	}
	return yaw;
}

void arwain::Madgwick::setQ(double w, double x, double y, double z)
{	
	q0 = w;
	q1 = x;
	q2 = y;
	q3 = z;
}
