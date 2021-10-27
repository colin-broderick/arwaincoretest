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

#ifndef QUAT_H
#define QUAT_H

#include <iostream>

class quaternion
{
    public:
        //  q = w + ix + jy + kz
        double w, x, y, z;

        // Constructors.
        quaternion();
        quaternion(double real, double im_i, double im_j, double im_k);
        quaternion(const double x, const double y, const double z);
        
        // Operators.
        quaternion operator+(quaternion quat2);
        quaternion operator-(quaternion quat2);
        quaternion operator*(quaternion quat2);
        quaternion operator*(double num);
        quaternion operator/(double num);
        quaternion operator+=(quaternion quat2);
        quaternion operator-=(quaternion quat2);
        quaternion operator*=(quaternion quat2);
        quaternion operator*=(double num);
        quaternion operator/=(double num);
        int operator==(quaternion q2);
        
        // General methods.
        double mag(); // magnitude
        quaternion unit(); // returns unit quaternion
        quaternion conj(); // returns the conjugate
        quaternion inv(); // returns the inverse
        quaternion unit_inv(); // inverse for unit quaternion
        
        // Getters.
        double getRe(); // return real component
        double getIm_i(); // return i component
        double getIm_j(); // return j component
        double getIm_k(); // return k component
};

inline std::ostream& operator<<(std::ostream& stream, quaternion quat)
{
    stream << "quaternion(" << quat.w << ", " << quat.x << ", " << quat.y << ", " << quat.z << ")";
    return stream;
}
 
#endif
