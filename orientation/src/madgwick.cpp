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

// Constructors -----------------------------------------------------------------------------------

/** \brief Default constructor.
 * Sets the initial quaternion q = (1, 0, 0, 0). Sets the inverse_sample_frequency as reciprocal of
 * sample_frequency_default. Sets angles_computed flag to 0. Sets beta equal to beta_default.
 */
arwain::Madgwick::Madgwick()
{
	beta = beta_default;
	w = 1.0;
	x = 0.0;
	y = 0.0;
	z = 0.0;
	inverse_sample_frequency = 1.0 / sample_frequency_default;
	angles_computed = 0;
}

/** \brief Constructor using custom sample frequency.
 * \param sample_frequency The anticipated update frequency in Hz.
 */
arwain::Madgwick::Madgwick(double sample_frequency, double beta_)
{
	beta = beta_;
	w = 1.0;
	x = 0.0;
	y = 0.0;
	z = 0.0;
	inverse_sample_frequency = 1.0 / sample_frequency;
	angles_computed = 0;
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

	double reciprocal_norm;
	double s0, s1, s2, s3;
	double qdot1, qdot2, qdot3, qdot4;
	double hx, hy;
	double _2_w_mx, _2_w_my, _2_w_mz, _2_x_mx, _2bx, _2bz, _4bx, _4bz, _2_w, _2_x, _2_y, _2_z, _2_w_y, _2_y_z, w_w, w_x, w_y, w_z, x_x, x_y, x_z, y_y, y_z, z_z;

	// Rate of change of quaternion from gyroscope; qdot = 0.5 * q ** w
	// where w = (0, gx, gy, gz) and ** is the quaternion product.
	// See https://www.euclideanspace.com/physics/kinematics/angularvelocity/QuaternionDifferentiation2.pdf.
	qdot1 = 0.5 * (-x * gx - y * gy - z * gz);
	qdot2 = 0.5 * (w * gx + y * gz - z * gy);
	qdot3 = 0.5 * (w * gy - x * gz + z * gx);
	qdot4 = 0.5 * (w * gz + x * gy - y * gx);

	// Compute feedback only if accelerometer measurement valid (avoids NaN in accelerometer normalisation)
	if (!((ax == 0.0) && (ay == 0.0) && (az == 0.0)))
	{
		// Normalise accelerometer measurement
		reciprocal_norm = inverse_sqrt(ax * ax + ay * ay + az * az);
		ax *= reciprocal_norm;
		ay *= reciprocal_norm;
		az *= reciprocal_norm;

		// Normalise magnetometer measurement
		reciprocal_norm = inverse_sqrt(mx * mx + my * my + mz * mz);
		mx *= reciprocal_norm;
		my *= reciprocal_norm;
		mz *= reciprocal_norm;

		// Auxiliary variables to avoid repeated arithmetic
		_2_w_mx = 2.0 * w * mx;
		_2_w_my = 2.0 * w * my;
		_2_w_mz = 2.0 * w * mz;
		_2_x_mx = 2.0 * x * mx;
		_2_w = 2.0 * w;
		_2_x = 2.0 * x;
		_2_y = 2.0 * y;
		_2_z = 2.0 * z;
		_2_w_y = 2.0 * w * y;
		_2_y_z = 2.0 * y * z;
		w_w = w * w;
		w_x = w * x;
		w_y = w * y;
		w_z = w * z;
		x_x = x * x;
		x_y = x * y;
		x_z = x * z;
		y_y = y * y;
		y_z = y * z;
		z_z = z * z;

		// Reference direction of Earth's magnetic field
		hx = mx * w_w - _2_w_my * z + _2_w_mz * y + mx * x_x + _2_x * my * y + _2_x * mz * z - mx * y_y - mx * z_z;
		hy = _2_w_mx * z + my * w_w - _2_w_mz * x + _2_x_mx * y - my * x_x + my * y_y + _2_y * mz * z - my * z_z;
		_2bx = sqrtf(hx * hx + hy * hy);
		_2bz = -_2_w_mx * y + _2_w_my * x + mz * w_w + _2_x_mx * z - mz * x_x + _2_y * my * z - mz * y_y + mz * z_z;
		_4bx = 2.0 * _2bx;
		_4bz = 2.0 * _2bz;

		// Gradient decent algorithm corrective step
		s0 = -_2_y * (2.0 * x_z - _2_w_y - ax) + _2_x * (2.0 * w_x + _2_y_z - ay) - _2bz * y * (_2bx * (0.5 - y_y - z_z) + _2bz * (x_z - w_y) - mx) + (-_2bx * z + _2bz * x) * (_2bx * (x_y - w_z) + _2bz * (w_x + y_z) - my) + _2bx * y * (_2bx * (w_y + x_z) + _2bz * (0.5 - x_x - y_y) - mz);
		s1 = _2_z * (2.0 * x_z - _2_w_y - ax) + _2_w * (2.0 * w_x + _2_y_z - ay) - 4.0 * x * (1 - 2.0 * x_x - 2.0 * y_y - az) + _2bz * z * (_2bx * (0.5 - y_y - z_z) + _2bz * (x_z - w_y) - mx) + (_2bx * y + _2bz * w) * (_2bx * (x_y - w_z) + _2bz * (w_x + y_z) - my) + (_2bx * z - _4bz * x) * (_2bx * (w_y + x_z) + _2bz * (0.5 - x_x - y_y) - mz);
		s2 = -_2_w * (2.0 * x_z - _2_w_y - ax) + _2_z * (2.0 * w_x + _2_y_z - ay) - 4.0 * y * (1 - 2.0 * x_x - 2.0 * y_y - az) + (-_4bx * y - _2bz * w) * (_2bx * (0.5 - y_y - z_z) + _2bz * (x_z - w_y) - mx) + (_2bx * x + _2bz * z) * (_2bx * (x_y - w_z) + _2bz * (w_x + y_z) - my) + (_2bx * w - _4bz * y) * (_2bx * (w_y + x_z) + _2bz * (0.5 - x_x - y_y) - mz);
		s3 = _2_x * (2.0 * x_z - _2_w_y - ax) + _2_y * (2.0 * w_x + _2_y_z - ay) + (-_4bx * z + _2bz * x) * (_2bx * (0.5 - y_y - z_z) + _2bz * (x_z - w_y) - mx) + (-_2bx * w + _2bz * y) * (_2bx * (x_y - w_z) + _2bz * (w_x + y_z) - my) + _2bx * x * (_2bx * (w_y + x_z) + _2bz * (0.5 - x_x - y_y) - mz);
		reciprocal_norm = inverse_sqrt(s0 * s0 + s1 * s1 + s2 * s2 + s3 * s3); // normalise step magnitude
		s0 *= reciprocal_norm;
		s1 *= reciprocal_norm;
		s2 *= reciprocal_norm;
		s3 *= reciprocal_norm;

		// Apply feedback step
		qdot1 -= beta * s0;
		qdot2 -= beta * s1;
		qdot3 -= beta * s2;
		qdot4 -= beta * s3;
	}

	// Integrate rate of change of quaternion to yield quaternion
	w += qdot1 * inverse_sample_frequency;
	x += qdot2 * inverse_sample_frequency;
	y += qdot3 * inverse_sample_frequency;
	z += qdot4 * inverse_sample_frequency;

	// Normalise quaternion
	reciprocal_norm = inverse_sqrt(w * w + x * x + y * y + z * z);
	w *= reciprocal_norm;
	x *= reciprocal_norm;
	y *= reciprocal_norm;
	z *= reciprocal_norm;
	angles_computed = 0;
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
	double reciprocal_norm;
	double s0, s1, s2, s3;
	double qdot1, qdot2, qdot3, qdot4;
	double _2_w, _2_x, _2_y, _2_z, _4q0, _4q1, _4q2 ,_8q1, _8q2, w_w, x_x, y_y, z_z;

	// Rate of change of quaternion from gyroscope; qdot = 0.5 * q ** w
	// where w = (0, gx, gy, gz) and ** is the quaternion product.
	// See https://www.euclideanspace.com/physics/kinematics/angularvelocity/QuaternionDifferentiation2.pdf.
	qdot1 = 0.5 * (-x * gx - y * gy - z * gz);
	qdot2 = 0.5 * (w * gx + y * gz - z * gy);
	qdot3 = 0.5 * (w * gy - x * gz + z * gx);
	qdot4 = 0.5 * (w * gz + x * gy - y * gx);

	// Compute feedback only if accelerometer measurement valid (avoids NaN in accelerometer normalisation)
	if (!((ax == 0.0) && (ay == 0.0) && (az == 0.0)))
	{
		// Normalise accelerometer measurement
		reciprocal_norm = inverse_sqrt(ax * ax + ay * ay + az * az);
		ax *= reciprocal_norm;
		ay *= reciprocal_norm;
		az *= reciprocal_norm;

		// Auxiliary variables to avoid repeated arithmetic
		_2_w = 2.0 * w;
		_2_x = 2.0 * x;
		_2_y = 2.0 * y;
		_2_z = 2.0 * z;
		_4q0 = 4.0 * w;
		_4q1 = 4.0 * x;
		_4q2 = 4.0 * y;
		_8q1 = 8.0 * x;
		_8q2 = 8.0 * y;
		w_w = w * w;
		x_x = x * x;
		y_y = y * y;
		z_z = z * z;

		// Gradient decent algorithm corrective step
		s0 = _4q0 * y_y + _2_y * ax + _4q0 * x_x - _2_x * ay;
		s1 = _4q1 * z_z - _2_z * ax + 4.0 * w_w * x - _2_w * ay - _4q1 + _8q1 * x_x + _8q1 * y_y + _4q1 * az;
		s2 = 4.0 * w_w * y + _2_w * ax + _4q2 * z_z - _2_z * ay - _4q2 + _8q2 * x_x + _8q2 * y_y + _4q2 * az;
		s3 = 4.0 * x_x * z - _2_x * ax + 4.0 * y_y * z - _2_y * ay;
		reciprocal_norm = inverse_sqrt(s0 * s0 + s1 * s1 + s2 * s2 + s3 * s3); // normalise step magnitude
		s0 *= reciprocal_norm;
		s1 *= reciprocal_norm;
		s2 *= reciprocal_norm;
		s3 *= reciprocal_norm;

		// Apply feedback step
		qdot1 -= beta * s0;
		qdot2 -= beta * s1;
		qdot3 -= beta * s2;
		qdot4 -= beta * s3;
	}

	// Integrate rate of change of quaternion to yield quaternion
	if (std::isnan(w + qdot1 * inverse_sample_frequency))
	{
		return;
	}
	w += qdot1 * inverse_sample_frequency;
	x += qdot2 * inverse_sample_frequency;
	y += qdot3 * inverse_sample_frequency;
	z += qdot4 * inverse_sample_frequency;

	// Normalise quaternion
	reciprocal_norm = inverse_sqrt(w * w + x * x + y * y + z * z);
	w *= reciprocal_norm;
	x *= reciprocal_norm;
	y *= reciprocal_norm;
	z *= reciprocal_norm;
	angles_computed = 0;
}

/** \brief Calculates inverse square root of number.
 * \param x Find square root of this number.
 * \return The square root of x.
 */
double arwain::Madgwick::inverse_sqrt(double x)
{
	return 1.0 / std::sqrt(x);
}

/** \brief Updates the internally-stored Euler angles.
 */
void arwain::Madgwick::compute_angles()
{
	roll = std::atan2(w*x + y*z, 0.5 - x*x - y*y);
	pitch = std::asin(-2.0 * (x*z - w*y));
	yaw = std::atan2(x*y + w*z, 0.5 - y*y - z*z);
	angles_computed = 1;
}

// Getters ----------------------------------------------------------------------------------------

/** \brief Check whether a new reading has been received since the Euler angles were last calculated.
 * \return True if no new reading has been received; false if a new reading has been received.
 */
bool arwain::Madgwick::angles_updated() const
{
	return angles_computed == 0 ? false : true;
}

/** \brief Gives the inverse sample frequency in 1/Hz, i.e., the assumed time between updates in seconds.
 * \return Expected time between samples in seconds.
 */
double arwain::Madgwick::get_inverse_sample_frequency() const
{
	return inverse_sample_frequency;
}

/** \brief Gives the inverse sample frequency in 1/Hz, i.e., the assumed time between updates in seconds.
 * \return Expected time between samples in seconds.
 */
double arwain::Madgwick::get_dt() const
{
	return get_inverse_sample_frequency();
}

/** \brief Reports the frequency in Hz at which the filter expects to receive new data. Note that frequency is not stored;
 * when requested, it is computed as 1.0 / inverse_sample_frequency. Therefore there may be some precision lost due
 * to floating point error.
 * \return Expected data update frequency.
 */
double arwain::Madgwick::get_frequency() const
{
	return 1.0 / inverse_sample_frequency;
}

/** \brief Returns w, also called q0; the real quaternion component.
 * \return The real part of the quaternion rotation operator.
 */
double arwain::Madgwick::get_w() const
{
	return w;
}

/** \brief Returns x, also called q1; the ith complex quaternion component.
 * \return The ith complex quaternion component.
 */
double arwain::Madgwick::get_x() const
{
	return x;
}

/** \brief Returns y, also called q2; the jth complex quaternion component.
 * \return The jth complex quaternion component.
 */
double arwain::Madgwick::get_y() const
{
	return y;
}

/** \brief Returns z, also called q3; the kth complex quaternion component.
 * \return The kth complex quaternion component.
 */
double arwain::Madgwick::get_z() const
{
	return z;
}

double arwain::Madgwick::get_roll()
{
	if (!angles_computed)
	{
		compute_angles();
	}
	return roll * 57.29578;
}

double arwain::Madgwick::get_pitch()
{
	if (!angles_computed)
	{
		compute_angles();
	}
	return pitch * 57.29578;
}

double arwain::Madgwick::get_yaw()
{
	if (!angles_computed)
	{
		compute_angles();
	}
	return yaw * 57.29578;
}

double arwain::Madgwick::get_roll_radians()
{
	if (!angles_computed)
	{
		compute_angles();
	}
	return roll;
}

double arwain::Madgwick::get_pitch_radians()
{
	if (!angles_computed)
	{
		compute_angles();
	}
	return pitch;
}

double arwain::Madgwick::get_yaw_radians()
{
	if (!angles_computed)
	{
		compute_angles();
	}
	return yaw;
}

double arwain::Madgwick::get_beta() const
{
	return beta;
}

void arwain::Madgwick::set_q(double w_, double x_, double y_, double z_)
{	
	w = w_;
	x = x_;
	y = y_;
	z = z_;
}

void arwain::Madgwick::set_beta(double beta_)
{
	beta = beta_;
}
