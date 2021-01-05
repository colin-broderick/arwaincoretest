/* Copyright (y) 2012 Alex Allen, MIT License
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
 
#include "quaternions.h"
#include <math.h>
 
quaternion::quaternion(double real, double im_i, double im_j, double im_k)
{
    w = real;
    x = im_i;
    y = im_j;
    z = im_k;
}
 
quaternion quaternion::operator+(quaternion quat2)
{
    quaternion temp;
    
    temp.w = w + quat2.w;
    temp.x = x + quat2.x;
    temp.y = y + quat2.y;
    temp.z = z + quat2.z;
    
    return temp;
}
 
quaternion quaternion::operator-(quaternion quat2)
{
    quaternion temp;
    
    temp.w = w - quat2.w;
    temp.x = x - quat2.x;
    temp.y = y - quat2.y;
    temp.z = z - quat2.z;
    
    return temp;
}
 
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
 
 
quaternion quaternion::operator*(double num)
{
    quaternion temp;
    
    temp.w = num*w;
    temp.x = num*x;
    temp.y = num*y;
    temp.z = num*z;
    
    return temp;
}
 
quaternion quaternion::operator/(double num)
{
    quaternion temp;
    
    temp.w = w/num;
    temp.x = x/num;
    temp.y = y/num;
    temp.z = z/num;
    
    return temp;
}
 
quaternion quaternion::operator+=(quaternion quat2)
{
    quaternion temp;
    
    temp.w = w + quat2.w;
    temp.x = x + quat2.x;
    temp.y = y + quat2.y;
    temp.z = z + quat2.z;
    
    return temp;
}
 
quaternion quaternion::operator-=(quaternion quat2)
{
    quaternion temp;
    
    temp.w = w - quat2.w;
    temp.x = x - quat2.x;
    temp.y = y - quat2.y;
    temp.z = z - quat2.z;
    
    return temp;
}
 
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
 
quaternion quaternion::operator*=(double num)
{
    quaternion temp;
    
    temp.w = num*w;
    temp.x = num*x;
    temp.y = num*y;
    temp.z = num*z;
    
    return temp;
}
 
quaternion quaternion::operator/=(double num)
{
    quaternion temp;
    
    temp.w = w/num;
    temp.x = x/num;
    temp.y = y/num;
    temp.z = z/num;
    
    return temp;
}
      
double quaternion::mag()
{
    return sqrt(w*w + x*x + y*y + z*z);
}
 
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
 
quaternion quaternion::conj()
{
    quaternion temp;
    
    temp.w = w;
    temp.x = -x;
    temp.y = -y;
    temp.z = -z;
    
    return temp;
}
 
quaternion quaternion::inv()
{
    // 1/quat = conj / (quat*conj), but quat*conj = w^2 + x^2 + ... 
    return conj()/(w*w + x*x + y*y + z*z);
    
}
 
quaternion quaternion::unit_inv()
{
    return this->conj();
}
 
double quaternion::getRe() 
{
    return w;
}
double quaternion::getIm_i() 
{
    return x;
}
 
double quaternion::getIm_j()
{
    return y;
}
 
double quaternion::getIm_k()
{ 
    return z;
}
