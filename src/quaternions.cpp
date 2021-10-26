/* Copyright (c) 2012 Alex Allen, MIT License
 *
 * Permission is hereby granted, free of charge, to any person obtaining w copy of this software 
 * and associated documentation files (the "Software"), to deal in the Software without restriction, 
 * including without limitation the rights to use, copy, modify, merge, publish, distribute, 
 * sublicense, and/or sell copies of the Software, and to permit persons to whom the Software is 
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in all copies or 
 * substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, INCLUDING 
 * BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND 
 * NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, 
 * DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, 
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
 */
 
#include "quaternions.hpp"

/** \brief Default quaternion constructor. Fills components with zeroes.
 */
quaternion::quaternion()
{
    w = 0;
    x = 0;
    y = 0;
    z = 0;
}

/** \brief Quaternion constructor. We always use the (w, x, y, z) convention, never (x, y, z, w).
 * \param real The real (w) component of the quaternion.
 * \param im_i The i imaginary component of the quaternion.
 * \param im_j The j imaginary component of the quaternion.
 * \param im_k The k imaginary component of the quaternion.
 */
quaternion::quaternion(double real, double im_i, double im_j, double im_k)
{
    w = real;
    x = im_i;
    y = im_j;
    z = im_k;
}

/** \brief Construct a quaternion from a three-vector.
 * \param vec A three-vector to convert to an orientation quaternion.
 */
quaternion::quaternion(const vector3 &vec)
{
    w = 0;
    x = vec.x;
    y = vec.y;
    z = vec.z;
}

/** \brief Add two quaternions.
 * Quaternions are added element-wise as
 * \f$ w = w_1 + w_2 \f$,
 * \f$x = x_1 + x_2\f$,
 * \f$y = y_1 + y_2\f$,
 * \f$z = z_1 + z_2\f$.
 * Quaternion addition is associative and commutative.
 * \param quat2 The quaternion to be added.
 * \return A new quaternion.
 */
quaternion quaternion::operator+(quaternion quat2)
{
    quaternion temp;
    
    temp.w = w + quat2.w;
    temp.x = x + quat2.x;
    temp.y = y + quat2.y;
    temp.z = z + quat2.z;
    
    return temp;
}

/** \brief See quaternion addition.
 * \param quat2 A quaternion to subtract.
 * \return A new quaternion.
 */
quaternion quaternion::operator-(quaternion quat2)
{
    quaternion temp;
    
    temp.w = w - quat2.w;
    temp.x = x - quat2.x;
    temp.y = y - quat2.y;
    temp.z = z - quat2.z;
    
    return temp;
}

/** \brief Quaternions follow a special multiplication rule.
 * \param quat2 Another quaternion by which to multiply.
 * \return A new quaternion.
 */
quaternion quaternion::operator*(quaternion quat2)
{
    quaternion temp;
    double e=quat2.w, f=quat2.x, g=quat2.y, h=quat2.z;
    
    temp.w = w*e -x*f -y*g -z*h;
    temp.x = w*f +x*e +y*h -z*g;
    temp.y = w*g -x*h +y*e +z*f;
    temp.z = w*h +x*g -y*f +z*e;
    
    return temp;
}

/** \brief Scalar multiplication of quaternions is done element-wise.
 * \param num Scalar by which to multiply the quaternion.
 * \return A new quaternion.
 */
quaternion quaternion::operator*(double num)
{
    quaternion temp;
    
    temp.w = num*w;
    temp.x = num*x;
    temp.y = num*y;
    temp.z = num*z;
    
    return temp;
}

/** \brief Division by a scalar is done element-wise.
 * \param num Scalar by which to divide the quaternion.
 * \return A new quaternion.
 */
quaternion quaternion::operator/(double num)
{
    quaternion temp;
    
    temp.w = w/num;
    temp.x = x/num;
    temp.y = y/num;
    temp.z = z/num;
    
    return temp;
}

/** \brief See quaternion addition.
 * \return A new quaternion.
 */
quaternion quaternion::operator+=(quaternion quat2)
{
    quaternion temp;
    
    temp.w = w + quat2.w;
    temp.x = x + quat2.x;
    temp.y = y + quat2.y;
    temp.z = z + quat2.z;
    
    return temp;
}

/** \brief See quaternion subtraction.
 * \return A new quaternion.
 */
quaternion quaternion::operator-=(quaternion quat2)
{
    quaternion temp;
    
    temp.w = w - quat2.w;
    temp.x = x - quat2.x;
    temp.y = y - quat2.y;
    temp.z = z - quat2.z;
    
    return temp;
}

/** \brief See quaternion multiplication.
 * \return A new quaternion.
 */
quaternion quaternion::operator*=(quaternion quat2)
{
    quaternion temp;
    double e=quat2.w, f=quat2.x, g=quat2.y, h=quat2.z;
    
    temp.w = w*e -x*f -y*g -z*h;
    temp.x = w*f +x*e +y*h -z*g;
    temp.y = w*g -x*h +y*e +z*f;
    temp.z = w*h +x*g -y*f +z*e;
    
    return temp;
}
 
/** \brief See quaternion scalar multiplication.
 * \return A new quaternion.
 */
quaternion quaternion::operator*=(double num)
{
    quaternion temp;
    
    temp.w = num*w;
    temp.x = num*x;
    temp.y = num*y;
    temp.z = num*z;
    
    return temp;
}

/** \brief See quaternion scalar division.
 * \return A new quaternion.
 */
quaternion quaternion::operator/=(double num)
{
    quaternion temp;
    
    temp.w = w/num;
    temp.x = x/num;
    temp.y = y/num;
    temp.z = z/num;
    
    return temp;
}

/** \brief Compares quaternions for equality.
 * Two quaternions are considered equivalent if all corresponding elements are equal.
 * \return 0 for inequality, 1 for equality.
 */
int quaternion::operator==(quaternion q2)
{
    int ret = 1;
    if (w != q2.w)
    {
        ret = 0;
    }
    if (x != q2.x)
    {
        ret = 0;
    }
    if (y != q2.y)
    {
        ret = 0;
    }
    if (z != q2.z)
    {
        ret = 0;
    }
    return ret;
}

/** \brief The L2 norm of the quaternion as an array.
 * \return Scalar value representing the L2 norm of the quaternion.
 */
double quaternion::mag()
{
    return sqrt(w*w + x*x + y*y + z*z);
}

/** \brief Normalize a quaterion.
 * A quaternion is considered normal if its L2 norm is equal to 1.
 * \return A normalized quaternion.
 */
quaternion quaternion::unit()
{
    quaternion temp;
    double size = mag();
    
    temp.w = w/size;
    temp.x = x/size;
    temp.y = y/size;
    temp.z = z/size;
    
    return temp;
}

/** \brief Conjugates the quaternion.
 * A quaternion is conjugates by negating all its imaginary components.
 * \return A new quaternion.
 */
quaternion quaternion::conj()
{
    quaternion temp;
    
    temp.w = w;
    temp.x = -x;
    temp.y = -y;
    temp.z = -z;
    
    return temp;
}
 
/** \brief Invert a quaterion.
 * \return A new quaternion.
 */
quaternion quaternion::inv()
{
    // 1/quat = conj / (quat*conj), but quat*conj = w^2 + x^2 + ... 
    return conj()/(w*w + x*x + y*y + z*z);
    
}

/** \brief Conjugate a quaternion.
 * \return A new quaternion.
 */
quaternion quaternion::unit_inv()
{
    return this->conj();
}
 
/** \brief Get the value of the real component.
 * \return Real/w component of the quaternion.
 */
double quaternion::getRe() 
{
    return w;
}

/** \brief Get the value of the first imaginary component.
 * \return Imaginary/i component of the quaternion.
 */
double quaternion::getIm_i() 
{
    return x;
}
 
/** \brief Get the value of the second imaginary component.
 * \return Imaginary/j component of the quaternion.
 */
double quaternion::getIm_j()
{
    return y;
}

/** \brief Get the value of the third imaginary component.
 * \return Imaginary/z component of the quaternion.
 */
double quaternion::getIm_k()
{ 
    return z;
}
